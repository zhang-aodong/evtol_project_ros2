"""Acquisition adapters. No ROS, file recording, localization or mapping here."""
import math
import time
import heapq
import numpy as np
from .rpc import AirSimRPC, image_request
from .camera_ring import SharedMemoryCameraReader
from .lidar_ring import LidarSharedMemory
from .runtime import AsyncRaycasts, BufferedImu
from .imu_ring import SharedMemoryImu
from .coordinates import vector


def v(x=0., y=0., z=0.):
    return dict(x_val=float(x), y_val=float(y), z_val=float(z))


def check_camera_projection(info, camera):
    # UE4.27 SceneCaptureRendering::BuildProjectionMatrix preserves horizontal
    # FOV for portrait renders too. AirSim's diagnostic getProjectionMatrix
    # instead uses a longer-axis multiplier when height > width. Accept that
    # known metadata convention while retaining the actual capture intrinsics
    # fx=width/(2*tan(hfov/2)), fy=fx throughout acquisition and localization.
    matrix = np.asarray(info['proj_mat']['matrix'], dtype=float)
    focal = 1. / math.tan(math.radians(camera['horizontal_fov_deg']) / 2.)
    aspect = camera['width'] / camera['height']
    expected = [focal, -focal * aspect]
    alternatives = [expected]
    if aspect < 1.:
        alternatives.append([focal / aspect, -focal])
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all() or not any(
            np.allclose([matrix[0, 1], matrix[1, 2]], value, rtol=1e-3, atol=1e-3)
            for value in alternatives):
        raise ValueError('AirSim capture projection differs from simulation.yaml')



class MockBackend:
    """Explicit test fixture; never qualifies as real AirSim acceptance."""
    def __init__(self, config, kind):
        self.c, self.kind = config, kind

    def capture(self):
        stamp = time.time_ns()
        c = self.c['camera']
        if self.kind == 'camera':
            image = np.zeros((c['height'], c['width'], 3), np.uint8)
            image[:, :, 1] = (stamp // 50_000_000) % 255
            semantic = np.zeros_like(image)
            semantic[20:80, 20:80] = [36, 13, 81]  # source drone class palette BGR
            return stamp, (image, semantic)
        if self.kind == 'lidar':
            angles = np.linspace(-math.pi/2, math.pi/2, 2048)
            return stamp, np.column_stack([10*np.cos(angles), 10*np.sin(angles),
                                            np.zeros(len(angles))]).astype('<f4')
        if self.kind == 'imu':
            return stamp, dict(orientation=dict(v(), w_val=1.),
                               angular_velocity=v(), linear_acceleration=v(0, 0, -9.81))
        if self.kind == 'gnss':
            point = self.home()
            if self.c.get('localization', {}).get('enabled', False):
                # The mock radio and kinematics are stationary at NED (0, 0, -10).
                point['altitude'] += 10.0
            return stamp, dict(gnss=dict(geo_point=point, eph=0.5, epv=0.8, fix_type=3))
        return stamp, dict(position=v(0, 0, -10), linear_velocity=v(),
                          orientation=dict(v(), w_val=1.), angular_velocity=v())

    def home(self):
        return dict(latitude=47.641468, longitude=-122.140165, altitude=122.)

    def truth(self):
        return time.time_ns(), dict(position=v(0, 0, -10), linear_velocity=v(),
                    orientation=dict(v(), w_val=1.), angular_velocity=v())

    def line_of_sight(self, a, b):
        return True

    def close(self):
        pass


class AirSimBackend:
    def __init__(self, config, kind):
        self.c, self.kind = config, kind
        connection = config['airsim']
        self.rpc = AirSimRPC(connection['host'], connection['port'], connection['timeout_s'])
        self.vehicle = connection['vehicle']
        self.camera_readers = []
        self.lidar = None
        self.started_camera = False
        self.last_frame_id = 0
        self.frame_offset = None
        self.home_geo = None
        self.source_skipped = 0
        self.los_cache = None
        self.los_rpc = None
        self.imu_sampler = None
        self.camera_pending = []
        self.camera_latest_ns = self.camera_emitted_ns = 0
        if not self.rpc.call('ping'):
            raise RuntimeError('AirSim ping failed')
        if kind == 'camera':
            self._configure_segmentation()
            info = self.rpc.call('simGetCameraInfo', config['camera']['name'], self.vehicle, False)
            check_camera_projection(info, config['camera'])
            if config['camera']['transport'] == 'shared_memory':
                try:
                    self._start_camera()
                except Exception:
                    self.close()
                    raise
        elif kind == 'lidar' and config['lidar']['transport'] == 'shared_memory':
            lc = config['lidar']
            self.lidar = LidarSharedMemory(lc['mapping_name'], lc['max_points'], lc['slots'])
        elif kind == 'imu':
            if config['imu'].get('transport','rpc')=='shared_memory':
                self.imu_sampler=SharedMemoryImu('AirSimImu_'+self.vehicle+'_'+config['imu']['name'],
                    config['rates']['imu'],float(config['imu'].get('buffer_latency_s',.1)))
            else:
                self.imu_sampler = BufferedImu(self._read_imu,
                    latency=float(config['imu'].get('buffer_latency_s', .05)),
                    poll_interval=1./max(400., config['rates']['imu']*8),
                    close_timeout=connection['timeout_s']+2.)
        elif kind == 'radio' and config['source']['five_g_a']['los']['enabled']:
            self.home()
            self.los_rpc = AirSimRPC(connection['host'], connection['port'],
                                    min(connection['timeout_s'], 1.))
            settings = config['source']['five_g_a']
            interval = float(settings['los']['update_interval_s'])
            self.los_cache = AsyncRaycasts(self._query_line_of_sight,
                [station['position_ned_m'] for station in settings['base_stations']],
                interval, max(2.5*interval, .5))

    def _configure_segmentation(self):
        semantic = self.c['source']['semantic_segmentation']
        if not self.rpc.call('simSetSegmentationObjectID', semantic['reset_object_regex'],
                             semantic['default_class_id'], True):
            raise ValueError('Segmentation reset matched no objects')
        exact = []
        for item in semantic['classes']:
            for rule in item['object_regexes']:
                matched = self.rpc.call('simSetSegmentationObjectID', rule, item['id'], True)
                if not matched and semantic['fail_on_unmatched_rule']:
                    raise ValueError(f'Segmentation rule matched no mesh: {rule}')
                if rule.startswith('^') and rule.endswith('$'):
                    name = rule[1:-1]
                    if all(ch.isalnum() or ch in '_-' for ch in name):
                        exact.append((name, item['id']))
        for name, class_id in exact:
            if self.rpc.call('simGetSegmentationObjectID', name.lower()) != class_id:
                raise ValueError(f'Segmentation assignment conflict: {name}')

    def _start_camera(self):
        c = self.c['camera']
        name = f"shm_push_start:{self.vehicle}:{c['name']}:{self.c['rates']['camera']:.6f}"
        requests = [image_request(name, i) for i in (0, 5)]
        for camera in c.get('surround_cameras', []):
            if camera==c['name']:continue  # the primary view was started above
            key = f"shm_push_start:{self.vehicle}:{camera}:{self.c['rates']['camera']:.6f}"
            requests.append(image_request(key, 0))
            if c.get('surround_record_truth',False):requests.append(image_request(key, 5))
        responses = self.rpc.call('simGetImages', requests, self.vehicle, False)
        self.started_camera = True
        if len(responses) != len(requests):
            raise RuntimeError('Missing camera shared-memory start responses')
        for response in responses:
            if not str(response.get('message', '')).startswith('shared_memory|'):
                raise RuntimeError('UE multi-camera push plugin missing: '+str(response.get('message')))
        for response in responses[:2]:
            message = response.get('message', '')
            if isinstance(message, bytes):
                message = message.decode('utf8')
            parts = message.split('|')
            if len(parts) != 3 or parts[0] != 'shared_memory':
                raise RuntimeError('UE push plugin missing: ' + message)
            if (response['width'], response['height']) != (c['width'], c['height']):
                raise ValueError('Image dimensions differ from simulation.yaml')
            self.camera_readers.append(SharedMemoryCameraReader(parts[1], c['width'],
                                                                 c['height'], c['slots']))

    def capture(self):
        sample = self._capture()
        if sample is None or self.kind != 'camera' or not self.camera_readers:
            return sample
        ns, images = sample
        # Control perception must not wait for the archival readback reorder
        # window. This callback keeps original acquisition stamps and may skip
        # late completions; the ordered raw RGB/semantic stream below is intact.
        callback = getattr(self, 'realtime_camera_callback', None)
        if callback is not None:
            latest = self.camera_readers[0].read_latest()
            if latest.timestamp_ns > getattr(self, 'realtime_emitted_ns', 0):
                cc = self.c['camera']
                rgb = np.frombuffer(latest.bgr_bytes, np.uint8).reshape(cc['height'], cc['width'], 3)
                if self.c['source']['semantic_segmentation']['vertical_flip']:rgb = np.flipud(rgb)
                callback(latest.timestamp_ns, (rgb,))
                self.realtime_emitted_ns = latest.timestamp_ns
        if self.camera_latest_ns and ns < self.camera_latest_ns-1_000_000_000:
            raise RuntimeError('Camera acquisition clock reset; restart acquisition after UE reset')
        self.camera_latest_ns = max(self.camera_latest_ns, ns)
        heapq.heappush(self.camera_pending, (ns, self.last_frame_id, images))
        # GPU slots complete in slot order, which can differ from capture order.
        # Hold one readback-ring window before publishing by acquisition time.
        window = self.c['camera'].get('reorder_window_s', .2)
        if self.camera_pending[0][0] > self.camera_latest_ns-int(window*1e9):
            return None
        stamp, _, data = heapq.heappop(self.camera_pending)
        if stamp <= self.camera_emitted_ns:
            raise RuntimeError('Camera completion exceeded reorder window; increase camera.reorder_window_s')
        self.camera_emitted_ns = stamp
        return stamp, data

    def _capture(self):
        c = self.c
        if self.kind == 'camera':
            cc = c['camera']
            if self.camera_readers:
                rgb = self.camera_readers[0].wait_for_new_frame(
                    self.last_frame_id, cc['timeout_s'])
                if self.frame_offset is None:
                    # Match startup by acquisition timestamp, not unrelated latest IDs.
                    latest = self.camera_readers[1].latest_frame_id()
                    candidates = []
                    for i in range(max(1, latest-cc['slots']+1), latest+1):
                        frame = self.camera_readers[1]._try_read_frame(i)
                        if frame and frame.timestamp_ns == rgb.timestamp_ns:
                            candidates.append(frame)
                    if not candidates:
                        self.last_frame_id = rgb.frame_id
                        return None
                    sem = candidates[-1]
                    self.frame_offset = sem.frame_id-rgb.frame_id
                else:
                    sem = self.camera_readers[1].read_frame(
                        rgb.frame_id+self.frame_offset, cc['timeout_s'])
                if self.last_frame_id:
                    self.source_skipped += max(0, rgb.frame_id-self.last_frame_id-1)
                self.last_frame_id = rgb.frame_id
                if rgb.timestamp_ns != sem.timestamp_ns:
                    raise ValueError('RGB/semantic producer timestamps differ')
                values = (rgb.bgr_bytes, sem.bgr_bytes)
                stamp = rgb.timestamp_ns
            else:
                responses = self.rpc.call('simGetImages', [image_request(cc['name'], i)
                        for i in (0, 5)], self.vehicle, False)
                if len(responses) != 2:
                    raise ValueError('Missing image response')
                if any((r['width'], r['height']) != (cc['width'], cc['height']) for r in responses):
                    raise ValueError('Image dimensions differ from configuration')
                stamp = int(responses[0]['time_stamp'])
                if stamp != int(responses[1]['time_stamp']):
                    raise ValueError('RPC RGB/semantic timestamps differ')
                values = [r['image_data_uint8'] for r in responses]
                values = [x.wire if hasattr(x, 'wire') else x.encode('latin1')
                          if isinstance(x, str) else bytes(x) for x in values]
            shape = cc['height'], cc['width'], 3
            images = tuple(np.frombuffer(x, dtype=np.uint8).reshape(shape) for x in values)
            if c['source']['semantic_segmentation']['vertical_flip']:
                images = tuple(np.flipud(x) for x in images)
            return stamp, images
        if self.kind == 'lidar':
            if self.lidar is not None:
                frame = self.lidar.read_next(self.last_frame_id)
                if frame is None:
                    return None
                if not frame.scan_complete:
                    # Do not disguise partial source scans with Foxglove trails.
                    self.last_frame_id = frame.frame_id
                    raise ValueError('Incomplete LiDAR frame: apply ue_support, rebuild UE and enable CompleteScanPerUpdate')
                if self.last_frame_id:
                    self.source_skipped += max(0, frame.frame_id-self.last_frame_id-1)
                self.last_frame_id = frame.frame_id
                return frame.timestamp_ns, np.frombuffer(frame.xyz_bytes, '<f4').reshape(-1, 3)
            data = self.rpc.call('getLidarData', c['lidar']['name'], self.vehicle)
            return int(data['time_stamp']), np.asarray(data['point_cloud'], dtype='<f4').reshape(-1, 3)
        if self.kind == 'imu':
            return self.imu_sampler.read()
        if self.kind == 'gnss':
            data = self.rpc.call('getGpsData', c['gnss']['name'], self.vehicle)
            return int(data['time_stamp']), data
        sample = self.truth()
        if self.los_cache:
            self.los_cache.update(vector(sample[1]['position']))
            if not self.los_cache.ready():
                return None  # Initial real raycasts must complete before observations.
        return sample

    def _read_imu(self):
        data = self.rpc.call('getImuData', self.c['imu']['name'], self.vehicle)
        return int(data['time_stamp']), data

    def truth(self):
        # Ground-truth kinematics has no timestamp field. Bracket the RPC using
        # AirSim's state clock, report midpoint; reject wide sampling intervals.
        if self.c.get('native_kinematics',False):
            responses=self.rpc.call('simGetImages',[image_request('kinematics_state:'+self.vehicle,0)],self.vehicle,False)
            if len(responses)!=1:raise ValueError('Incomplete native ego kinematics')
            response=responses[0];parts=response.get('message','').split('|')
            if len(parts)!=3 or parts[0]!='kinematics_state_v1':raise ValueError('Native kinematics plugin missing: rebuild UE')
            before,after=map(int,parts[1:]);values=np.asarray(response['image_data_float'],float)
            if values.shape!=(13,) or not np.isfinite(values).all() or not 0<=after-before<=self.c['max_truth_bracket_ms']*1e6:
                raise ValueError('Invalid native ego kinematics bracket')
            return int(response['time_stamp']),dict(position=v(*values[:3]),
                orientation=dict(v(*values[3:6]),w_val=float(values[6])),
                linear_velocity=v(*values[7:10]),angular_velocity=v(*values[10:13]))
        before = int(self.rpc.call('getMultirotorState', self.vehicle)['timestamp'])
        data = self.rpc.call('simGetGroundTruthKinematics', self.vehicle)
        after = int(self.rpc.call('getMultirotorState', self.vehicle)['timestamp'])
        if after < before or after-before > self.c['max_truth_bracket_ms'] * 1e6:
            raise ValueError('Unbounded ground-truth acquisition time')
        return (before+after)//2, data

    def home(self):
        if self.home_geo is None:
            self.home_geo = self.rpc.call('getHomeGeoPoint', self.vehicle)
        return self.home_geo

    def line_of_sight(self, a, b):
        if self.los_cache:
            return self.los_cache.lookup(b)
        return self._query_line_of_sight(a, b)

    def _query_line_of_sight(self, a, b):
        home = self.home()
        def geo(ned):
            settings = self.c['source']['five_g_a']['los']
            radius = settings['earth_radius_m']
            cos_lat = max(abs(math.cos(math.radians(home['latitude']))), settings['min_cos_latitude'])
            return dict(latitude=home['latitude']+math.degrees(ned[0]/radius),
                longitude=home['longitude']+math.degrees(ned[1]/(radius*cos_lat)),
                altitude=home['altitude']-float(ned[2]))
        # Fail visibly on API errors; do not silently turn unknown NLoS into LoS.
        return bool((self.los_rpc or self.rpc).call('simTestLineOfSightBetweenPoints', geo(a), geo(b)))

    def close(self):
        if self.imu_sampler:
            self.imu_sampler.close()
        if self.los_cache:
            self.los_cache.close()
        if self.los_rpc:
            self.los_rpc.close()
        for reader in self.camera_readers:
            reader.close()
        self.camera_readers.clear()
        if self.lidar:
            self.lidar.close()
        if self.started_camera:
            try:
                name = f"shm_push_stop:{self.vehicle}:{self.c['camera']['name']}"
                self.rpc.call('simGetImages', [image_request(name, 0)], self.vehicle, False)
            except Exception:
                pass
        self.rpc.close()
