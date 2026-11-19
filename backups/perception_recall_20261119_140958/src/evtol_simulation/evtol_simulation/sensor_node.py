"""One process per acquisition group; simulation observations only."""
import argparse
import math
import time
import threading
from collections import deque
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock as RclClock, ClockType
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from builtin_interfaces.msg import Time
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo, Imu, NavSatFix, NavSatStatus, PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from evtol_sim_interfaces.msg import GnssObservations, GnssObservation, FiveGObservations, FiveGLink
from .configuration import load, KINDS
from .backend import AirSimBackend, MockBackend
from .coordinates import NED_ENU, FRD_FLU, vector, orientation, extrinsic, quaternion_matrix
from .gnss_model import GnssRawSimulator, geodetic_to_ecef, ned_to_ecef_delta
from .radio_model import FiveGASimulator, FiveGAState
from .runtime import TimerResolution, RateGate, SensorLease, AcquisitionWait
from .semantic import SemanticDecoder


def stamp(ns):
    return Time(sec=int(ns)//1_000_000_000, nanosec=int(ns)%1_000_000_000)


def header(ns, frame):
    return Header(stamp=stamp(ns), frame_id=frame)


def xyz(target, values):
    target.x, target.y, target.z = map(float, values)


def quat(target, values):
    target.x, target.y, target.z, target.w = map(float, values)


class SensorNode(Node):
    def __init__(self, config, kind):
        super().__init__(kind + '_sensor')
        self.c, self.kind = config, kind
        self.frames = config['frames']
        self.prefix = '/' + config['namespace'].strip('/')
        self.count = self.errors = self.duplicates = self.regressions = 0
        self.last_stamp = self.first_stamp = 0
        self.last_wall = None
        self.first_wall = None
        self.last_error = ''
        self.timings = deque(maxlen=500)
        self.recent = deque(maxlen=10000)
        self.timing_lock = threading.Lock()
        self.stop_worker = threading.Event()
        self.worker = None
        self.pubs = {}
        self.steady = RclClock(clock_type=ClockType.STEADY_TIME)
        self.diag = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_timer(1., self.diagnostics, clock=self.steady)
        self.backend = None
        self.gnss_model = None
        self.radio_model = None
        self.rate_gate = RateGate(config['rates']['imu'], recover_phase=True) if kind == 'imu' and config['backend'] == 'airsim' else None
        self.base = self.frames['base']
        if kind == 'truth':
            self.tf = TransformBroadcaster(self)
            self.static = StaticTransformBroadcaster(self)
            self.clock_pub = self.create_publisher(Clock, '/clock',
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            self._static_frames()
        # Steady-clock timers keep acquisition and diagnostics alive before /clock,
        # during simulation pauses, and when use_sim_time is set by a launch file.
        poll_rate = config['rates'][kind]
        if kind == 'camera' and config['backend'] == 'airsim' and config['camera']['transport'] == 'shared_memory':
            # The producer gates frame rate. Poll with headroom, then block for
            # a new acquisition, avoiding phase aliasing at equal timer periods.
            poll_rate *= 2
        if config['backend'] == 'airsim' and (kind == 'imu' or
                kind == 'lidar' and config['lidar']['transport'] == 'shared_memory'):
            # 1.25 ms retry interval reduces phase misses against the native IMU.
            poll_rate *= 8 if kind == 'imu' else 4
        self.timer = None
        if kind == 'imu' and config['backend'] == 'airsim':
            # Keep the high-rate RPC sampler off the Python ROS executor. Its
            # steady-clock gate still limits publication to new 100 Hz samples.
            self.worker = threading.Thread(target=self._imu_loop, name='imu_acquisition', daemon=True)
            self.worker.start()
        else:
            self.timer = self.create_timer(1./poll_rate, self.tick, clock=self.steady)

    def _imu_loop(self):
        waiter = AcquisitionWait()
        try:
            while not self.stop_worker.is_set():
                self.tick()
                due = self.rate_gate.due
                delay = max(.0002, min(.00125, due-time.perf_counter())) if due else .00125
                waiter.wait(delay, self.stop_worker)
        finally:
            waiter.close()

    def publish(self, suffix, msg, sensor=True):
        if suffix not in self.pubs:
            qos = qos_profile_sensor_data if sensor else QoSProfile(depth=10)
            self.pubs[suffix] = self.create_publisher(type(msg), self.prefix+'/'+suffix, qos)
        self.pubs[suffix].publish(msg)

    def _transform(self, child, parent, translation, rotation, ns=0):
        msg = TransformStamped(header=header(ns, parent), child_frame_id=child)
        xyz(msg.transform.translation, translation)
        quat(msg.transform.rotation, rotation)
        return msg

    def _static_frames(self):
        transforms = []
        for key in ('camera', 'lidar', 'imu', 'gnss'):
            p, q = extrinsic(self.c[key]['xyz_frd_m'], self.c[key]['rpy_deg'])
            transforms.append(self._transform(self.frames[key], self.base, p, q))
        # Camera optical X=right,Y=down,Z=forward expressed in camera FLU.
        transforms.append(self._transform(self.frames['optical'], self.frames['camera'],
                                          [0,0,0], [-.5, .5, -.5, .5]))
        self.static.sendTransform(transforms)

    def tick(self):
        try:
            if self.rate_gate and not self.rate_gate.ready(time.perf_counter()):
                return
            if self.backend is None:
                self.backend = (MockBackend if self.c['backend'] == 'mock' else AirSimBackend)(self.c, self.kind)
                if self.kind == 'camera':
                    self.backend.realtime_camera_callback = self.emit_realtime_camera
            begun = time.perf_counter()
            sample = self.backend.capture()
            captured = time.perf_counter()
            if sample is None:
                return
            ns, data = sample
            if ns <= 0:
                raise ValueError('Missing acquisition timestamp')
            if ns == self.last_stamp:
                self.duplicates += 1
                return
            if ns < self.last_stamp:
                self.regressions += 1
                raise ValueError('Simulation clock reset; restart bridge after UE reset')
            getattr(self, 'emit_' + self.kind)(ns, data)
            wall = time.perf_counter()
            with self.timing_lock:
                self.timings.append(((captured-begun)*1000, (wall-captured)*1000))
                self.recent.append(wall)
            if self.rate_gate:
                self.rate_gate.accepted(time.perf_counter())
            if self.count == 0:
                self.first_stamp, self.first_wall = ns, wall
            self.last_stamp, self.last_wall = ns, wall
            self.count += 1
        except Exception as exc:
            self.errors += 1
            self.last_error = f'{type(exc).__name__}: {exc}'
            if self.errors == 1 or self.errors % 20 == 0:
                self.get_logger().error(self.last_error)

    def emit_realtime_camera(self, ns, images):
        """Latest monotonic RGB for control, before archival reorder/decoding."""
        if ns <= getattr(self, 'realtime_stamp', 0):
            return
        import cv2
        rgb = images[0]
        width = int(self.c['camera'].get('realtime_width', 960))
        scale = min(1., width / max(rgb.shape[:2]))
        if scale < 1.:
            rgb = cv2.resize(rgb, (round(rgb.shape[1]*scale), round(rgb.shape[0]*scale)), interpolation=cv2.INTER_AREA)
        h, w = rgb.shape[:2]
        msg = Image(header=header(ns, self.frames['optical']), height=h, width=w,
                    encoding='bgr8', is_bigendian=0, step=w*3)
        msg.data.frombytes(memoryview(np.ascontiguousarray(rgb)))
        self.publish('camera/rgb/image_realtime', msg)
        self.realtime_stamp = ns

    def emit_camera(self, ns, images):
        # Also covers RPC/mock backends, which do not reorder GPU completions.
        self.emit_realtime_camera(ns, images)
        rgb, semantic = images
        cc = self.c['camera']
        shape = (cc['height'], cc['width'], 3)
        if rgb.shape != shape or semantic.shape != shape:
            raise ValueError('Invalid image shape')
        classes = self.c['source']['semantic_segmentation']
        # Preserve the source palette exactly; class IDs remain 0..6, unknown=255.
        if not hasattr(self, 'semantic_decoder'):
            self.semantic_decoder = SemanticDecoder(classes['classes'], classes['unknown_class_id'])
        labels = self.semantic_decoder(semantic)
        if np.count_nonzero(labels == classes['unknown_class_id']) / labels.size > classes['max_unmapped_fraction']:
            raise ValueError('Excess unknown semantic palette pixels')
        for suffix, array, encoding in [('camera/rgb/image_raw', rgb, 'bgr8'),
                                        ('camera/semantic/image', semantic, 'bgr8'),
                                        ('camera/semantic/class_ids', labels, 'mono8')]:
            msg = Image(header=header(ns, self.frames['optical']), height=cc['height'],
                        width=cc['width'], encoding=encoding, is_bigendian=0,
                        step=cc['width']*(1 if encoding=='mono8' else 3))
            # The generated Image constructor and setter each copy byte sequences.
            # Fill its public array buffer directly: one copy, no temporary bytes.
            msg.data.frombytes(memoryview(np.ascontiguousarray(array)))
            self.publish(suffix, msg)
        focal = cc['width'] / (2*math.tan(math.radians(cc['horizontal_fov_deg'])/2))
        info = CameraInfo(header=header(ns, self.frames['optical']), height=cc['height'], width=cc['width'],
            distortion_model='plumb_bob', d=[0.]*5,
            k=[focal,0.,cc['width']/2,0.,focal,cc['height']/2,0.,0.,1.],
            r=[1.,0.,0.,0.,1.,0.,0.,0.,1.],
            p=[focal,0.,cc['width']/2,0.,0.,focal,cc['height']/2,0.,0.,0.,1.,0.])
        self.publish('camera/rgb/camera_info', info)
        self.publish('camera/semantic/camera_info', info)

    def emit_lidar(self, ns, points):
        points = np.asarray(points, dtype='<f4')
        if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
            raise ValueError('Invalid XYZ point cloud')
        if len(points) > self.c['lidar']['max_points']:
            raise ValueError('Point cloud exceeds configured ring capacity')
        if len(points) and np.max(np.linalg.norm(points, axis=1)) > self.c['lidar']['range_m']*1.01:
            raise ValueError('LiDAR values exceed physical range')
        # Intensity is the source constant placeholder, not measured reflectance.
        data = np.ones((len(points), 4), dtype='<f4') * self.c['lidar']['default_intensity']
        data[:,:3] = points * np.array([1,-1,-1], dtype='<f4')
        fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
                  for i,n in enumerate(('x','y','z','intensity'))]
        self.publish('lidar/points', PointCloud2(header=header(ns, self.frames['lidar']),
            height=1, width=len(points), fields=fields, is_bigendian=False,
            point_step=16, row_step=len(points)*16, data=data.tobytes(), is_dense=True))

    def emit_imu(self, ns, data):
        msg = Imu(header=header(ns, self.frames['imu']))
        quat(msg.orientation, orientation(data['orientation']))
        xyz(msg.angular_velocity, FRD_FLU @ vector(data['angular_velocity']))
        xyz(msg.linear_acceleration, FRD_FLU @ vector(data['linear_acceleration']))
        # Zero covariance means unknown, rather than invented precision.
        self.publish('imu/data', msg)

    def emit_gnss(self, ns, data):
        gps = data['gnss']
        geo = gps['geo_point']
        msg = NavSatFix(header=header(ns, self.frames['gnss']),
            latitude=float(geo['latitude']), longitude=float(geo['longitude']), altitude=float(geo['altitude']))
        msg.status.status = NavSatStatus.STATUS_FIX if gps['fix_type'] >= 2 else NavSatStatus.STATUS_NO_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        # AirSim EPH/EPV fields are exposed as approximate horizontal/vertical accuracy.
        msg.position_covariance = [float(gps['eph'])**2,0.,0.,0.,float(gps['eph'])**2,0.,0.,0.,float(gps['epv'])**2]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.publish('gnss/fix', msg)
        if self.c['source']['gnss_raw']['enabled']:
            home = self.backend.home()
            origin = geodetic_to_ecef(home['latitude'], home['longitude'], home['altitude'])
            if self.gnss_model is None:
                self.gnss_model = GnssRawSimulator(home['latitude'], home['longitude'], origin,
                                                  self.c['source']['gnss_raw'])
            truth_ns, kin = self.backend.truth()
            # Native GPS has a 200 ms latency. Raw synthetic observations are a
            # separate truth-driven sensor, timestamped at their own epoch.
            pos = origin + ned_to_ecef_delta(vector(kin['position']), home['latitude'], home['longitude'])
            vel = ned_to_ecef_delta(vector(kin['linear_velocity']), home['latitude'], home['longitude'])
            observations = self.gnss_model.simulate_epoch(pos, vel)
            raw = GnssObservations(header=header(truth_ns, self.frames['world']),
                                   sequence=self.count, synthetic=True)
            raw.observations = [GnssObservation(**o) for o in observations]
            self.publish('gnss/raw_observations', raw)

    def emit_truth(self, ns, data):
        p = NED_ENU @ vector(data['position'])
        q = orientation(data['orientation'])
        odom = Odometry(header=header(ns, self.frames['world']), child_frame_id=self.base)
        xyz(odom.pose.pose.position, p)
        quat(odom.pose.pose.orientation, q)
        xyz(odom.twist.twist.linear, quaternion_matrix(q).T @ NED_ENU @ vector(data['linear_velocity']))
        xyz(odom.twist.twist.angular, FRD_FLU @ vector(data['angular_velocity']))
        self.publish('ground_truth/odometry', odom)
        if self.c['publish_truth_tf']:
            self.tf.sendTransform(self._transform(self.base, self.frames['world'], p, q, ns))
        self.clock_pub.publish(Clock(clock=stamp(ns)))

    def emit_radio(self, ns, data):
        settings = self.c['source']['five_g_a']
        if self.radio_model is None:
            los = self.backend.line_of_sight if settings['los']['enabled'] else None
            self.radio_model = FiveGASimulator(settings, los,
                cache_los=not bool(getattr(self.backend, 'los_cache', None)))
        state = FiveGAState(self.count, ns, 0. if not self.first_stamp else (ns-self.first_stamp)*1e-9,
                            'simulation', vector(data['position']), vector(data['linear_velocity']))
        msg = FiveGObservations(header=header(ns, self.frames['world']), sequence=self.count, synthetic=True)
        for obs in self.radio_model.simulate_epoch(state):
            link = FiveGLink(gnb_id=obs.gnb_id, prs_rsrp_dbm=float(obs.prs_rsrp_dbm),
                is_los=bool(obs.is_los), range_sigma_m=float(obs.range_sigma_m),
                measured_range_m=float(obs.measured_range_m), measured_rtt_s=float(obs.measured_rtt_s),
                measurement_valid=bool(obs.measurement_valid))
            xyz(link.base_station_enu, NED_ENU @ np.array([obs.gnb_n_m,obs.gnb_e_m,obs.gnb_d_m]))
            msg.observations.append(link)
        self.publish('five_ga/observations', msg)

    def diagnostics(self):
        now = time.perf_counter()
        age = math.inf if self.last_wall is None else now-self.last_wall
        with self.timing_lock:
            while self.recent and self.recent[0] < now-10.:
                self.recent.popleft()
            measured = ((len(self.recent)-1)/(self.recent[-1]-self.recent[0])
                        if len(self.recent)>1 else 0.)
            timing = np.asarray(self.timings) if self.timings else np.zeros((1,2))
        stale = age > self.c['stale_timeout_s']
        duration = 0. if self.first_wall is None else max(1e-9, self.last_wall-self.first_wall)
        values = dict(backend=self.c['backend'], sensor=self.kind, samples=self.count,
            errors=self.errors, duplicate_source_polls=self.duplicates, clock_regressions=self.regressions,
            target_hz=self.c['rates'][self.kind], wall_hz=(self.count-1)/duration if self.count>1 else 0.,
            last_age_s=age, last_error=self.last_error, recent_hz=measured,
            capture_p95_ms=float(np.percentile(timing[:,0],95)),
            emit_p95_ms=float(np.percentile(timing[:,1],95)),
            source_skipped_frames=getattr(self.backend,'source_skipped',0),
            imu_queue_overruns=getattr(getattr(self.backend,'imu_sampler',None),'overruns',0),
            imu_buffered_sim_age_s=getattr(getattr(self.backend,'imu_sampler',None),'buffered_sim_age_s',0.),
            schedule_dropped_slots=getattr(self.rate_gate,'dropped_slots',0))
        status = DiagnosticStatus(name=self.prefix+'/'+self.kind, hardware_id=self.c['airsim']['vehicle'],
            level=DiagnosticStatus.ERROR if stale or self.errors else DiagnosticStatus.OK,
            message=self.last_error if self.errors else ('stale' if stale else 'streaming'),
            values=[KeyValue(key=k, value=str(v)) for k,v in values.items()])
        self.diag.publish(DiagnosticArray(header=Header(stamp=self.get_clock().now().to_msg()), status=[status]))

    def close(self):
        self.stop_worker.set()
        if self.worker:
            self.worker.join(timeout=self.c['airsim']['timeout_s']+2.)
        if self.timer:
            self.timer.cancel()
        if self.backend:
            self.backend.close()


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--kind', required=True, choices=KINDS)
    parser.add_argument('--backend', choices=('airsim','mock'))
    opts, ros_args = parser.parse_known_args(args)
    config = load(opts.config)
    if opts.backend:
        config['backend'] = opts.backend
    lease = SensorLease(config, opts.kind)
    resolution = TimerResolution(high_rate=opts.kind == 'imu')
    rclpy.init(args=ros_args)
    node = SensorNode(config, opts.kind)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        resolution.close()
        lease.close()
