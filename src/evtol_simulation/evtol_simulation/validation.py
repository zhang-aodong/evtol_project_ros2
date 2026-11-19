"""Measure subscriber-side unique acquisition stamps, wall rate and payload quality.

Exit 0 only on PASS; 1 on failed acceptance; 2 on invalid invocation/runtime.
Mock results require --allow-mock and are labelled PASS_MOCK, never PASS_AIRSIM.
"""
import argparse
import json
import math
import time
from collections import deque
from functools import partial
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, Imu, NavSatFix, PointCloud2
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from tf2_msgs.msg import TFMessage
from evtol_sim_interfaces.msg import GnssObservations, FiveGObservations
from .configuration import load, KINDS
from .runtime import TimerResolution


def ns(time_msg):
    return int(time_msg.sec)*1_000_000_000 + int(time_msg.nanosec)


def finite(values):
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Non-finite numerical payload')


def check_payload(topic, msg, c):
    if isinstance(msg, Image):
        channels = 1 if msg.encoding == 'mono8' else 3
        if msg.encoding not in ('bgr8','mono8') or msg.is_bigendian:
            raise ValueError('Unexpected image encoding')
        if (msg.width,msg.height) != (c['camera']['width'],c['camera']['height']):
            raise ValueError('Unexpected image dimensions')
        if msg.step != msg.width*channels or len(msg.data) != msg.height*msg.step:
            raise ValueError('Invalid image byte length')
        if topic.endswith('class_ids'):
            values = np.frombuffer(msg.data, np.uint8)
            allowed = [x['id'] for x in c['source']['semantic_segmentation']['classes']]
            if sorted(allowed) == list(range(max(allowed)+1)):
                invalid = values > max(allowed)
            else:
                lut = np.zeros(256, dtype=bool)
                lut[allowed] = True
                invalid = ~lut[values]
            if np.count_nonzero(invalid)/len(values) > c['source']['semantic_segmentation']['max_unmapped_fraction']:
                raise ValueError('Invalid semantic IDs')
    elif isinstance(msg, CameraInfo):
        finite(msg.k)
        if msg.k[0] <= 0 or msg.k[4] <= 0 or msg.k[8] != 1.:
            raise ValueError('Invalid camera calibration')
    elif isinstance(msg, PointCloud2):
        navigation=topic=='navigation/lidar/points'
        names=('x','y','z') if navigation else ('x','y','z','intensity')
        step=4*len(names)
        if msg.point_step != step or msg.height != 1 or msg.row_step != msg.width*step or len(msg.data) != msg.row_step:
            raise ValueError('Invalid PointCloud2 layout')
        if [(f.name,f.offset,f.datatype,f.count) for f in msg.fields] != [(n,i*4,7,1) for i,n in enumerate(names)]:
            raise ValueError('Invalid PointCloud2 fields')
        points = np.frombuffer(msg.data, '<f4').reshape(-1,len(names))
        if not np.isfinite(points).all():
            raise ValueError('Non-finite LiDAR points')
        radius=c['navigation_lidar']['range_m'] if navigation else c['lidar']['range_m']
        if len(points) and np.max(np.linalg.norm(points[:,:3],axis=1)) > radius*1.01:
            raise ValueError('LiDAR outside range')
    elif isinstance(msg, (Imu, Odometry)):
        q = msg.orientation if isinstance(msg,Imu) else msg.pose.pose.orientation
        finite([q.x,q.y,q.z,q.w])
        if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) > .01:
            raise ValueError('Invalid quaternion norm')
        if isinstance(msg, Imu):
            for v in (msg.angular_velocity,msg.linear_acceleration):
                finite([v.x,v.y,v.z])
        else:
            for v in (msg.pose.pose.position,msg.twist.twist.linear,msg.twist.twist.angular):
                finite([v.x,v.y,v.z])
    elif isinstance(msg, NavSatFix):
        finite([msg.latitude,msg.longitude,msg.altitude,*msg.position_covariance])
        if not -90<=msg.latitude<=90 or not -180<=msg.longitude<=180 or msg.status.status<0:
            raise ValueError('GNSS has no valid fix')
    elif isinstance(msg, GnssObservations):
        if not msg.synthetic or len(msg.observations) != c['source']['gnss_raw']['satellite_count']:
            raise ValueError('Missing synthetic GNSS observations')
        ids = [o.prn for o in msg.observations]
        if len(set(ids)) != len(ids):
            raise ValueError('Repeated satellite PRN')
        for o in msg.observations:
            finite([o.pseudorange_m,o.carrier_phase_cycles,o.doppler_hz,o.snr_dbhz,o.elevation_deg,o.azimuth_deg])
            if o.pseudorange_m <= 0:
                raise ValueError('Invalid pseudorange')
    elif isinstance(msg, FiveGObservations):
        if not msg.synthetic or {o.gnb_id for o in msg.observations} != {o['id'] for o in c['source']['five_g_a']['base_stations']}:
            raise ValueError('Missing 5G-A links')
        for o in msg.observations:
            finite([o.prs_rsrp_dbm,o.base_station_enu.x,o.base_station_enu.y,o.base_station_enu.z])
            if o.measurement_valid:
                finite([o.measured_range_m,o.measured_rtt_s,o.range_sigma_m])
                if o.measured_range_m<=0 or o.measured_rtt_s<=0 or o.range_sigma_m<=0:
                    raise ValueError('Invalid 5G-A range')


class StreamStats:
    def __init__(self, rate):
        self.rate = rate
        self.stamps = []
        self.walls = []
        self.errors = []
        self.empty = 0

    def add(self, stamp, wall):
        if stamp<=0 or self.stamps and stamp<=self.stamps[-1]:
            self.errors.append('duplicate/regressing/zero timestamp')
            return
        self.stamps.append(stamp)
        self.walls.append(wall)

    def report(self, duration, limits):
        count = len(self.stamps)
        rate = count/duration
        rtf = ((self.stamps[-1]-self.stamps[0])*1e-9/(self.walls[-1]-self.walls[0])) if count>1 else 0.
        gaps = np.diff(self.stamps)*1e-9 if count>1 else np.array([math.inf])
        wall_gaps = np.diff(self.walls) if count>1 else np.array([math.inf])
        rate_passed = limits['min_rate_ratio']*self.rate <= rate <= limits['max_rate_ratio']*self.rate
        passed = (not self.errors and limits['min_rate_ratio']*self.rate <= rate <= limits['max_rate_ratio']*self.rate
            and limits['min_realtime_factor']<=rtf<=limits['max_realtime_factor']
            and float(np.max(gaps))<=limits['max_gap_periods']/self.rate
            and float(np.max(wall_gaps))<=limits['max_gap_periods']/self.rate)
        return dict(passed=bool(passed), rate_passed=bool(rate_passed), samples=count, target_hz=self.rate, wall_hz=rate,
            realtime_factor=rtf, max_sim_gap_s=float(np.max(gaps)), max_wall_gap_s=float(np.max(wall_gaps)),
            empty_clouds=self.empty, errors=self.errors[:20])


class Validator(Node):
    def __init__(self, c, warmup, scope='all'):
        super().__init__('sensor_acceptance')
        self.c = c
        self.start = time.perf_counter()+warmup
        self.stats = {}
        self.health = {}
        self.frames_seen = {}
        self.static_frames = set()
        self.tf_seen = False
        self.pairs = {k:deque(maxlen=200) for k in ('camera/rgb/image_raw','camera/semantic/image','lidar/points')}
        self.subs = []
        # Independent callback groups keep 2K image checks from holding the
        # 100 Hz IMU callback behind large image deserializations/checks.
        self.groups = []
        specs = [('camera/rgb/image_raw',Image,'camera'),('camera/semantic/image',Image,'camera'),
            ('camera/semantic/class_ids',Image,'camera'),('camera/rgb/camera_info',CameraInfo,'camera'),
            ('camera/semantic/camera_info',CameraInfo,'camera'),('lidar/points',PointCloud2,'lidar'),
            ('imu/data',Imu,'imu'),('gnss/fix',NavSatFix,'gnss'),
            ('ground_truth/odometry',Odometry,'truth'),('five_ga/observations',FiveGObservations,'radio')]
        if c['source']['gnss_raw']['enabled']:
            specs.append(('gnss/raw_observations',GnssObservations,'gnss'))
        if c.get('navigation_lidar',{}).get('enabled'):
            specs.append(('navigation/lidar/points',PointCloud2,'navigation_lidar'))
        camera_topic = {'rgb':'camera/rgb/image_raw', 'semantic':'camera/semantic/image',
                        'labels':'camera/semantic/class_ids'}.get(scope)
        specs = [s for s in specs if scope=='all' or
                 (scope=='imu' and s[2]=='imu') or (scope=='without-imu' and s[2]!='imu') or
                 (camera_topic and s[0]==camera_topic) or
                 (scope=='core' and s[2]!='imu' and (s[2]!='camera' or s[0].endswith('camera_info')))]
        self.kinds = {kind for _,_,kind in specs}
        for suffix,typ,kind in specs:
            group = MutuallyExclusiveCallbackGroup()
            self.groups.append(group)
            self.stats[suffix] = StreamStats(c['navigation_lidar']['rate_hz'] if kind=='navigation_lidar' else c['rates'][kind])
            self.subs.append(self.create_subscription(typ,'/'+c['namespace']+'/'+suffix,
                                partial(self.receive,suffix),qos_profile_sensor_data,callback_group=group))
        self.stats['/clock'] = StreamStats(c['rates']['truth'])
        self.subs.append(self.create_subscription(Clock,'/clock',self.receive_clock,qos_profile_sensor_data))
        self.subs.append(self.create_subscription(DiagnosticArray,'/diagnostics',self.diagnostics,10))
        self.subs.append(self.create_subscription(TFMessage,'/tf',self.transforms,100))
        from rclpy.qos import QoSProfile, DurabilityPolicy
        self.subs.append(self.create_subscription(TFMessage,'/tf_static',self.transforms,
            QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL)))

    def receive(self, suffix, msg):
        wall = time.perf_counter()
        if wall < self.start:
            return
        stat = self.stats[suffix]
        t = ns(msg.header.stamp)
        stat.add(t,wall)
        self.frames_seen[suffix] = msg.header.frame_id
        if suffix in self.pairs:
            self.pairs[suffix].append(t)
        try:
            check_payload(suffix,msg,self.c)
            if isinstance(msg,PointCloud2) and msg.width==0:
                stat.empty += 1
        except Exception as exc:
            stat.errors.append(str(exc))

    def receive_clock(self, msg):
        wall = time.perf_counter()
        if wall>=self.start:
            self.stats['/clock'].add(ns(msg.clock),wall)

    def diagnostics(self, msg):
        for s in msg.status:
            values = {v.key:v.value for v in s.values}
            if s.name.startswith('/'+self.c['namespace']+'/') and values.get('sensor'):
                self.health[values['sensor']] = (time.perf_counter(),s.level,values)

    def transforms(self, msg):
        for tf in msg.transforms:
            if tf.header.frame_id==self.c['frames']['world'] and tf.child_frame_id==self.c['frames']['base']:
                self.tf_seen = True
            self.static_frames.add(tf.child_frame_id)

    def report(self, duration, allow_mock):
        limits = self.c['acceptance']
        streams = {k:v.report(duration,limits) for k,v in self.stats.items()}
        if limits['require_nonempty_lidar'] and 'lidar/points' in streams and streams['lidar/points']['empty_clouds']:
            streams['lidar/points']['passed'] = False
        problems = []
        now = time.perf_counter()
        for kind in self.kinds:
            health = self.health.get(kind)
            if not health or now-health[0]>3. or health[1]!=DiagnosticStatus.OK or int(health[2].get('errors',1)):
                problems.append(f'{kind}: missing, stale or failed diagnostics')
            elif health[2].get('backend') != ('mock' if allow_mock else 'airsim'):
                problems.append(f'{kind}: wrong backend for this acceptance run')
        frames = self.c['frames']
        for topic,actual in self.frames_seen.items():
            expected = (frames['optical'] if topic.startswith('camera/') else
                frames['lidar'] if topic.startswith('lidar/') else
                frames['base'] if topic.startswith('navigation/lidar/') else
                frames['imu'] if topic.startswith('imu/') else
                frames['gnss'] if topic=='gnss/fix' else frames['world'])
            if actual!=expected:
                problems.append(f'{topic}: frame_id {actual} != {expected}')
        if self.c['publish_truth_tf'] and not self.tf_seen:
            problems.append('Missing ground-truth dynamic TF')
        if not {frames[k] for k in ('camera','optical','lidar','imu','gnss')} <= self.static_frames:
            problems.append('Missing static extrinsic TF')
        rgb,sem,lidar = [list(self.pairs[k]) for k in self.pairs]
        if {'camera/rgb/image_raw','camera/semantic/image'} <= self.stats.keys() and (not rgb or not sem or len(set(rgb)&set(sem)) < .95*min(len(rgb),len(sem))):
            problems.append('RGB/semantic exact timestamp pairing failed')
        # Ignore edge samples where a neighbour may be outside the recorded window.
        interior = [t for t in rgb if lidar and lidar[0]<=t<=lidar[-1]]
        max_delta = max((min(abs(t-l) for l in lidar)*1e-6 for t in interior),default=math.inf)
        if 'camera/rgb/image_raw' in self.stats and 'lidar/points' in self.stats and max_delta>limits['max_sync_delta_ms']:
            problems.append('Camera/LiDAR acquisition synchronization exceeds tolerance')
        passed = all(v['passed'] for v in streams.values()) and not problems
        return dict(result=('PASS_MOCK' if allow_mock else 'PASS_AIRSIM') if passed else 'FAIL',
            passed=passed, backend='mock' if allow_mock else 'airsim', duration_s=duration,
            streams=streams, problems=problems, max_camera_lidar_delta_ms=max_delta,
            frequency_passed=all(v['rate_passed'] for v in streams.values()),
            publisher_diagnostics={kind: health[2] for kind, health in self.health.items()})


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    parser.add_argument('--duration', type=float, default=60.)
    parser.add_argument('--warmup', type=float, default=10.)
    parser.add_argument('--allow-mock', action='store_true')
    parser.add_argument('--scope', choices=('all','imu','without-imu','core','rgb','semantic','labels'), default='all')
    parser.add_argument('--output',type=Path,default=Path('reports/acceptance.json'))
    opts,ros_args = parser.parse_known_args(args)
    if opts.duration<5 or opts.warmup<0:
        parser.error('duration must be >=5 and warmup >=0')
    resolution=TimerResolution(high_rate=opts.scope=='imu')
    rclpy.init(args=ros_args)
    node = Validator(load(opts.config),opts.warmup,opts.scope)
    interrupted = False
    deadline = node.start+opts.duration
    executor = MultiThreadedExecutor(num_threads=4) if opts.scope in ('all','without-imu') else SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and time.perf_counter()<deadline:
            executor.spin_once(timeout_sec=min(.05,max(0.,deadline-time.perf_counter())))
    except KeyboardInterrupt:
        interrupted = True
    executor.shutdown()
    report = node.report(opts.duration,opts.allow_mock)
    # Used only to merge independently observed streams; these are original
    # acquisition stamps, not fabricated pairing/clock corrections.
    report['pairing_stamps_ns'] = {k:list(v) for k,v in node.pairs.items()}
    if interrupted:
        report.update(passed=False,result='FAIL')
        report['problems'].append('Acceptance interrupted')
    opts.output.parent.mkdir(parents=True,exist_ok=True)
    # JSON forbids infinity; stringify missing-data markers as null.
    def clean(obj):
        if isinstance(obj,dict): return {k:clean(v) for k,v in obj.items()}
        if isinstance(obj,list): return [clean(v) for v in obj]
        return None if isinstance(obj,float) and not math.isfinite(obj) else obj
    opts.output.write_text(json.dumps(clean(report),indent=2,ensure_ascii=False),encoding='utf8')
    print(json.dumps(clean(report),indent=2,ensure_ascii=False))
    node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()
    resolution.close()
    raise SystemExit(0 if report['passed'] else 1)
