"""ROS 2 -> official Foxglove SDK. Native Windows; CDR bytes stay unchanged."""
import argparse
from queue import SimpleQueue
import time

import foxglove
from foxglove.websocket import Capability, ServerListener
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from .schemas import expand_schema


class Listener(ServerListener):
    def __init__(self):
        self.requests = SimpleQueue()

    def on_subscribe(self, client, channel):
        # SDK callbacks are on another thread; interact with ROS/cache on executor only.
        self.requests.put(channel.topic)


class Bridge(Node):
    def __init__(self, namespace='drone1', host='127.0.0.1', port=8765, services=None):
        super().__init__('foxglove_visualization_bridge')
        root = '/' + namespace.strip('/')
        self.allowed = {'/tf', '/tf_static', '/clock', '/diagnostics'} | {
            root+'/'+suffix for suffix in (
                'preview/rgb/image/compressed', 'preview/semantic/image/compressed',
                'preview/lidar/points', 'imu/data', 'gnss/fix', 'gnss/raw_observations',
                'five_ga/observations', 'ground_truth/odometry', 'ground_truth/local_odometry',
                'localization/odometry','localization/status','localization/metrics','localization/error_enu','localization/error_3d','localization/path','localization/truth_path',
                'state/odometry','state/status','perception/semantic_preview/compressed',
                'perception/surround/rgb/compressed','perception/surround/semantic_preview/compressed',
                'perception/surround/detections','perception/surround/detections_preview/compressed',
                'perception/lidar/points',
                'perception/targets_measurements','perception/tracks','perception/markers',
                'perception/lidar_obstacles','perception/lidar_obstacle_markers',
                'safety/risk','safety/risks','warning/lidar','warning/fusion','control/desired_velocity','control/safe_velocity',
                'navigation/lidar/points','navigation/path','navigation/waypoint_path','navigation/traveled_path','navigation/reference_path',
                'mission/status','control/flight_status')}
        self.allowed.update(root+'/perception/roi_lidar/surround_'+view+'/points'
            for view in ('front','right','back','left','up','down'))
        self.allowed.update(root+'/perception/roi_lidar/'+view+'/points' for view in ('front_left','front_center','front_right'))
        self.path_topics={root+'/navigation/waypoint_path',root+'/navigation/traveled_path',root+'/navigation/reference_path',root+'/localization/path',root+'/localization/truth_path'}
        self.path_data={}
        self.allowed.update({'/simulation/reference_origin','/simulation/geodetic_reference','/simulation/truth/targets','/simulation/truth/status',
            '/evaluation/detection_metrics','/evaluation/target_matches','/evaluation/encounter_events','/evaluation/warning_metrics','/evaluation/markers'})
        self.channels, self.subs, self.static_transforms = {}, {}, {}
        self.origin_data = None
        self.geodetic_data = None
        self.sim_ns = 0
        self.listener = Listener()
        self.server = foxglove.start_server(name='EVTOL ROS 2', host=host, port=port,
            capabilities=[Capability.Time]+([Capability.Services] if services else []), server_listener=self.listener,
            services=services, supported_encodings=['json'] if services else None,
            message_backlog_size=64)
        self.create_timer(1., self.discover)
        self.create_timer(.05, self.replay_static)
        self.get_logger().info(f'Foxglove WebSocket: ws://{host}:{self.server.port} ({"console services enabled" if services else "read only"})')
        self.discover()

    def discover(self):
        for topic, types in self.get_topic_names_and_types():
            if topic not in self.allowed or topic in self.channels or len(types) != 1:
                continue
            try:
                cls = get_message(types[0])
                channel = foxglove.Channel(topic, message_encoding='cdr', schema=foxglove.Schema(
                    name=types[0], encoding='ros2msg', data=expand_schema(types[0])))
                qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
                if topic in self.path_topics or topic in ('/tf_static','/simulation/reference_origin','/simulation/geodetic_reference') or topic.endswith(('/mission/status','/control/flight_status')):
                    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
                self.channels[topic] = channel
                self.subs[topic] = self.create_subscription(cls, topic,
                    lambda data, t=topic: self.forward(t, data), qos, raw=True)
                self.get_logger().info('Exposing '+topic+' ['+types[0]+']')
            except Exception as exc:
                if topic in self.channels:
                    self.channels.pop(topic).close()
                self.get_logger().error('Cannot expose '+topic+': '+str(exc))

    def forward(self, topic, data):
        if topic == '/clock':
            stamp = deserialize_message(data, Clock).clock
            ns = stamp.sec*1_000_000_000+stamp.nanosec
            if self.sim_ns and ns < self.sim_ns:
                self.server.clear_session()
                self.origin_data = None
                self.geodetic_data = None
                self.path_data.clear()
                self.get_logger().warning('Simulation clock reset: cleared Foxglove session')
            self.sim_ns = ns
            self.server.broadcast_time(ns)
        if topic == '/tf_static':
            for transform in deserialize_message(data, TFMessage).transforms:
                self.static_transforms[transform.child_frame_id] = transform
            self.publish_static()
            return
        if topic == '/simulation/reference_origin':
            self.origin_data = data
        if topic == '/simulation/geodetic_reference':self.geodetic_data = data
        if topic in self.path_topics:
            self.path_data[topic]=data
        channel = self.channels[topic]
        if channel.has_sinks():
            channel.log(data, log_time=self.sim_ns or time.time_ns())

    def publish_static(self):
        channel = self.channels.get('/tf_static')
        if channel and channel.has_sinks() and self.static_transforms:
            msg = TFMessage(transforms=list(self.static_transforms.values()))
            channel.log(serialize_message(msg), log_time=self.sim_ns or time.time_ns())

    def replay_static(self):
        while not self.listener.requests.empty():
            topic = self.listener.requests.get()
            if topic == '/tf_static':
                self.publish_static()
            elif topic == '/simulation/reference_origin' and self.origin_data:
                self.channels[topic].log(self.origin_data, log_time=self.sim_ns or time.time_ns())
            elif topic == '/simulation/geodetic_reference' and self.geodetic_data:
                self.channels[topic].log(self.geodetic_data, log_time=self.sim_ns or time.time_ns())
            elif topic in self.path_data:
                self.channels[topic].log(self.path_data[topic], log_time=self.sim_ns or time.time_ns())

    def destroy_node(self):
        self.server.stop()
        for channel in self.channels.values(): channel.close()
        return super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--namespace', default='drone1')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8765)
    opts, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = Bridge(opts.namespace, opts.host, opts.port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
