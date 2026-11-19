"""Observe existing publishers; optionally verify the real Foxglove wire protocol."""
import argparse
import json
from pathlib import Path
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, CompressedImage
from tf2_ros import Buffer, TransformListener, TransformException


class Probe(Node):
    def __init__(self):
        super().__init__('visualization_diagnostic')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.counts, self.pending, self.errors = {}, [], []
        self.tf_ok = self.tf_bad = self.clock_resets = 0
        self.last_clock = 0
        self.create_subscription(Clock, '/clock', self.clock, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/drone1/lidar/points', self.cloud, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, '/drone1/preview/rgb/image/compressed',
                                 lambda m: self.count('preview_rgb'), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/drone1/preview/lidar/points',
                                 lambda m: self.count('preview_cloud'), qos_profile_sensor_data)

    def count(self, name):
        self.counts[name] = self.counts.get(name, 0)+1

    def clock(self, msg):
        ns = msg.clock.sec*1_000_000_000+msg.clock.nanosec
        if self.last_clock and ns < self.last_clock: self.clock_resets += 1
        self.last_clock = ns
        self.count('clock')

    def cloud(self, msg):
        self.count('source_cloud')
        self.pending.append((time.perf_counter(), msg.header))

    def check(self):
        while self.pending and time.perf_counter()-self.pending[0][0] > .3:
            _, header = self.pending.pop(0)
            try:
                self.buffer.lookup_transform('map', header.frame_id, Time.from_msg(header.stamp))
                self.tf_ok += 1
            except TransformException as exc:
                self.tf_bad += 1
                if len(self.errors) < 3: self.errors.append(str(exc))


def websocket_check(url, duration):
    from websockets.sync.client import connect
    channels, subscribed, counts, sizes, schemas = {}, {}, {}, {}, {}
    with connect(url, subprotocols=['foxglove.sdk.v1', 'foxglove.websocket.v1'],
                 max_size=16*1024*1024, proxy=None, compression=None) as sock:
        deadline = time.perf_counter()+duration
        while time.perf_counter() < deadline:
            try:
                frame = sock.recv(timeout=1)
            except TimeoutError:
                continue
            if isinstance(frame, str):
                packet = json.loads(frame)
                if packet.get('op') != 'advertise': continue
                for channel in packet['channels']:
                    cid = channel['id']
                    if cid in channels: continue
                    if channel['encoding'] != 'cdr' or channel['schemaEncoding'] != 'ros2msg':
                        raise ValueError('Unexpected encoding: '+str(channel))
                    # ros2msg is textual (binary schema encodings such as protobuf use base64).
                    definition = channel['schema']
                    # The CDR decoder on the viewer needs nested message definitions.
                    if channel['schemaName'] != 'rosgraph_msgs/msg/Clock' and 'MSG: ' not in definition:
                        raise ValueError('Missing expanded schema: '+channel['topic'])
                    channels[cid] = channel
                    schemas[channel['topic']] = len(definition)
                    sid = len(subscribed)+1
                    subscribed[sid] = channel
                    sock.send(json.dumps({'op':'subscribe','subscriptions':[{'id':sid,'channelId':cid}]}))
            elif frame[0] == 1:
                sid, timestamp = struct.unpack_from('<IQ', frame, 1)
                channel = subscribed[sid]
                topic = channel['topic']
                msg = deserialize_message(frame[13:], get_message(channel['schemaName']))
                counts[topic] = counts.get(topic, 0)+1
                sizes[topic] = len(frame)-13
                if isinstance(msg, CompressedImage):
                    import cv2
                    import numpy as np
                    pixels = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
                    if pixels is None or pixels.shape[1] > 640:
                        raise ValueError('Invalid preview image')
                if isinstance(msg, PointCloud2) and (not msg.width or msg.width*msg.height > 16000):
                    raise ValueError('Invalid preview point cloud')
    required = ['/tf', '/tf_static', '/clock', '/drone1/preview/rgb/image/compressed',
                '/drone1/preview/semantic/image/compressed', '/drone1/preview/lidar/points',
                '/drone1/imu/data', '/drone1/gnss/raw_observations', '/drone1/five_ga/observations']
    missing = [topic for topic in required if not counts.get(topic)]
    return {'passed':not missing, 'duration_s':duration, 'missing':missing,
            'received':counts, 'last_payload_bytes':sizes, 'schema_bytes':schemas}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=8.)
    parser.add_argument('--websocket')
    parser.add_argument('--report', default='reports/visualization_diagnostic.json')
    opts = parser.parse_args()
    rclpy.init()
    node = Probe()
    # Let DDS discovery deliver transient-local TF before scoring transforms.
    warmup = time.perf_counter()+3.
    while time.perf_counter() < warmup:
        rclpy.spin_once(node, timeout_sec=.02)
    node.counts.clear()
    node.pending.clear()
    start = time.perf_counter()
    while time.perf_counter()-start < opts.duration:
        rclpy.spin_once(node, timeout_sec=.02)
        node.check()
    result = {'counts':node.counts, 'observed_hz':{k:round(v/opts.duration, 2) for k,v in node.counts.items()},
              'tf_at_cloud_stamp':{'ok':node.tf_ok,'failed':node.tf_bad,'errors':node.errors},
              'clock_resets':node.clock_resets}
    result['passed'] = node.tf_ok > 0 and node.tf_bad == 0 and node.clock_resets == 0
    node.destroy_node()
    rclpy.shutdown()
    if opts.websocket:
        try:
            result['websocket'] = websocket_check(opts.websocket, opts.duration)
        except Exception as exc:
            result['websocket'] = {'passed':False, 'error':str(exc)}
        result['passed'] &= result['websocket']['passed']
    target = Path(opts.report)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['passed'] else 1)


if __name__ == '__main__': main()
