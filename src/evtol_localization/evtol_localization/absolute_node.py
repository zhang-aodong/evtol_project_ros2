"""ROS 2 adapter for AirSim ego GNSS/IMU and optional 5G-A localization."""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.clock import Clock as RclClock, ClockType
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, NavSatFix
from evtol_perception_interfaces.msg import ReferenceOrigin
from evtol_sim_interfaces.msg import FiveGObservations
from evtol_simulation.coordinates import matrix_quaternion
from evtol_targeting.common import args_config, Health, LATCHED, header, ns, put, xyz

from .absolute import ImuSample, RangeLink, SessionMap, Settings, Timeline, geodetic, vector3


class AbsoluteNode(Node):
    def __init__(self, config):
        super().__init__('ego_localization')
        self.root = '/' + config['namespace'].strip('/')
        self.base_frame = self.root[1:] + '/base_link'
        self.mode = config['localization']['mode']
        if self.mode not in ('gnss_imu', 'gnss_imu_5g'):
            raise ValueError('unsupported absolute localization mode')
        self.cfg = Settings(**config['localization'].get('absolute', {}))
        self.timeline = Timeline(self.cfg)
        self.timeout = float(config['state']['timeout_s'])
        self.origin = self.home = self.frame = None
        self.session = self.raw_frame = ''
        self.epoch = self.sim_now = self.published = 0
        self.last_imu_wall = None
        self.clock_wall = self.last_stalled_report = time.monotonic()
        self.resets = 0
        self.last_reset = ''
        self.health = Health(self, self.root + '/ego_localization')
        self.pub = self.create_publisher(Odometry, config['state']['fused_topic'], qos_profile_sensor_data)
        self.create_subscription(ReferenceOrigin, '/simulation/reference_origin', self.on_origin, LATCHED)
        self.create_subscription(NavSatFix, self.root + '/gnss/home', self.on_home, LATCHED)
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        self.create_subscription(Imu, self.root + '/imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, self.root + '/gnss/fix', self.on_fix, qos_profile_sensor_data)
        if self.mode == 'gnss_imu_5g':
            self.create_subscription(FiveGObservations, self.root + '/five_ga/observations',
                                     self.on_radio, qos_profile_sensor_data)
        self.create_timer(.2, self.report, clock=RclClock(clock_type=ClockType.STEADY_TIME))

    def reset(self, reason):
        self.timeline = Timeline(self.cfg)
        self.published = 0
        self.last_imu_wall = None
        self.resets += 1
        self.last_reset = reason
        self.health.last = None

    def on_origin(self, msg):
        try:
            origin = vector3(xyz(msg.source_origin_enu))
            epoch = ns(msg.header.stamp)
            if not msg.session_id or not msg.source_frame or msg.header.frame_id != 'map_enu':
                raise ValueError('invalid session origin')
            if self.origin is not None and (
                self.session != msg.session_id or self.raw_frame != msg.source_frame
                or self.epoch != epoch or not np.array_equal(self.origin, origin)
            ):
                self.reset('session_origin_changed')
            self.session, self.epoch = msg.session_id, epoch
            self.origin, self.raw_frame = origin, msg.source_frame
            if self.home is not None:
                self.frame = SessionMap(self.home, self.origin)
        except ValueError as exc:
            self.health.error(exc)

    def on_home(self, msg):
        try:
            home = geodetic([msg.latitude, msg.longitude, msg.altitude])
            if msg.status.status < 0:
                raise ValueError('invalid GNSS georeference status')
            if self.home is not None and self.home != home:
                self.reset('geographic_origin_changed')
            self.home = home
            if self.origin is not None:
                self.frame = SessionMap(self.home, self.origin)
        except ValueError as exc:
            self.health.error(exc)

    def on_clock(self, msg):
        now = ns(msg.clock)
        if now < self.sim_now:
            self.reset('simulation_clock_rewind')
            if self.epoch > now:
                self.origin = self.frame = None
                self.epoch = 0
                self.session = self.raw_frame = ''
        if now != self.sim_now:
            self.clock_wall = time.monotonic()
        self.sim_now = now

    def add(self, kind, msg, data):
        t = ns(msg.header.stamp)
        if t < self.epoch:
            self.timeline.counts[kind + '/before_session'] += 1
            return False
        inserted = self.timeline.add(kind, t, data)
        if not inserted:
            self.health.dropped += 1
        return inserted

    def on_imu(self, msg):
        try:
            if msg.angular_velocity_covariance[0] < 0 or msg.linear_acceleration_covariance[0] < 0:
                raise ValueError('missing angular velocity or specific force')
            q = msg.orientation
            orientation = np.array([q.x, q.y, q.z, q.w])
            if (msg.orientation_covariance[0] < 0 or not np.isfinite(orientation).all()
                    or np.linalg.norm(orientation) < 1e-9):
                orientation = None
            sample = ImuSample(xyz(msg.angular_velocity), xyz(msg.linear_acceleration), orientation)
            previous = self.timeline.latest_imu
            if self.add('imu', msg, sample) and self.timeline.latest_imu > previous:
                self.last_imu_wall = time.monotonic()
                s = self.timeline.state
                if s.fixed and self.frame is not None and s.t > self.published:
                    if self.sim_now and (self.sim_now - s.t) * 1e-9 > self.timeout:
                        self.timeline.counts['output/stale'] += 1
                    else:
                        self.publish_state(s)
        except (ValueError, np.linalg.LinAlgError) as exc:
            self.health.error(exc)

    def on_fix(self, msg):
        try:
            if self.frame is None:
                data = 'waiting_reference'
            elif msg.status.status < 0:
                data = 'no_fix'
            else:
                if msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
                    covariance = np.eye(3) * self.cfg.gnss_unknown_sigma_m ** 2
                else:
                    covariance = np.asarray(msg.position_covariance).reshape(3, 3)
                data = self.frame.fix([msg.latitude, msg.longitude, msg.altitude], covariance)
        except ValueError:
            data = 'invalid_coordinates_or_covariance'
        self.add('fix', msg, data)

    def on_radio(self, msg):
        if self.frame is None or msg.header.frame_id != self.raw_frame:
            self.timeline.counts['radio/waiting_or_wrong_frame'] += 1
            return
        links = []
        for obs in msg.observations:
            # An invalid station should not discard other links in the epoch.
            station = self.frame.station(xyz(obs.base_station_enu))
            links.append(RangeLink(obs.gnb_id, station, obs.measured_range_m, obs.range_sigma_m,
                                   obs.prs_rsrp_dbm, obs.measurement_valid))
        self.add('radio', msg, links)

    def publish_state(self, s):
        msg = Odometry(header=header(s.t, 'map_enu'), child_frame_id=self.base_frame)
        put(msg.pose.pose.position, s.pos)
        q = msg.pose.pose.orientation
        q.x, q.y, q.z, q.w = map(float, matrix_quaternion(s.rot))
        # White gyro density converted to instantaneous sampled rate variance.
        keys = self.timeline.keys
        previous = next((t for t, p in reversed(keys) if p == 0 and t < s.imu_t), 0)
        period = max((s.imu_t - previous) * 1e-9, 1e-4) if previous else .01
        linear, angular, pose_cov, twist_cov = s.output(self.cfg.gyro_noise_density ** 2 / period)
        put(msg.twist.twist.linear, linear)
        put(msg.twist.twist.angular, angular)
        msg.pose.covariance = pose_cov.ravel().tolist()
        msg.twist.covariance = twist_cov.ravel().tolist()
        self.pub.publish(msg)
        self.published = s.t
        self.health.ok()

    def report(self):
        s = self.timeline.state
        age = (self.sim_now - self.published) * 1e-9 if self.sim_now and self.published else None
        filter_age = (self.sim_now - s.t) * 1e-9 if self.sim_now and s.fixed else None
        if self.origin is None:
            status = 'waiting_session_origin'
        elif self.home is None:
            status = 'waiting_geographic_origin'
        elif self.last_imu_wall is None or not s.fixed:
            status = 'waiting_imu_attitude_and_valid_fix'
        elif time.monotonic() - self.last_imu_wall > self.timeout or (filter_age is not None and filter_age > self.timeout):
            status = 'stale'
        elif (s.t - max(s.last_fix, s.last_range)) * 1e-9 > self.timeout:
            status = 'inertial_only'
        else:
            status = 'tracking'
        self.health.details = dict(mode=self.mode, filter_status=status, session=self.session,
                                   state_stamp_ns=self.published, state_age_sim_s=age, filter_age_sim_s=filter_age,
                                   accepted_fix_stamp_ns=s.last_fix, accepted_range_stamp_ns=s.last_range,
                                   fix_correction_m=s.fix_jump, resets=self.resets, last_reset=self.last_reset)
        self.health.details.update({'received_' + k + '_ns': v for k, v in self.timeline.last_received.items()})
        self.health.details.update({k: v for k, v in self.timeline.counts.items() if v})
        if status in ('stale', 'inertial_only'):
            self.health.last_error = status
        # The shared Health timer uses node time. Keep diagnostics alive when
        # the ROS clock pauses or its replay publisher vanishes.
        now = time.monotonic()
        if (self.get_parameter('use_sim_time').value and now - self.clock_wall > 1.
                and now - self.last_stalled_report > 1.):
            self.health.emit()
            self.last_stalled_report = now


def main(args=None):
    config, ros_args = args_config(args)
    rclpy.init(args=ros_args)
    node = AbsoluteNode(config)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
