"""Bounded visualization derivatives. Original topics and timestamps stay intact."""
import argparse
from array import array
import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CompressedImage, PointCloud2


QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


def resize_image(msg, width, semantic=False):
    if msg.encoding not in ('rgb8', 'bgr8', 'mono8'):
        raise ValueError('Unsupported preview encoding: ' + msg.encoding)
    channels = 1 if msg.encoding == 'mono8' else 3
    if msg.width <= 0 or msg.height <= 0 or msg.step < msg.width * channels:
        raise ValueError('Invalid image dimensions or row stride')
    shape = (msg.height, msg.width) if channels == 1 else (msg.height, msg.width, channels)
    strides = (msg.step, 1) if channels == 1 else (msg.step, channels, 1)
    pixels = np.ndarray(shape, dtype=np.uint8, buffer=msg.data, strides=strides)
    target_width = min(width, msg.width)
    target_height = max(1, round(msg.height * target_width / msg.width))
    resized = cv2.resize(pixels, (target_width, target_height),
                         interpolation=cv2.INTER_NEAREST if semantic else cv2.INTER_AREA)
    out = Image(header=msg.header, height=target_height, width=target_width,
                encoding=msg.encoding, is_bigendian=msg.is_bigendian,
                step=target_width * channels)
    out.data = array('B', resized.tobytes())
    return out, resized


def thin_cloud(msg, max_points):
    if msg.point_step <= 0 or msg.row_step < msg.width * msg.point_step:
        raise ValueError('Invalid point cloud stride')
    # Respect organized clouds and row padding; preserve every field, including intensity.
    points = np.ndarray((msg.height, msg.width, msg.point_step), dtype=np.uint8,
                        buffer=msg.data, strides=(msg.row_step, msg.point_step, 1))
    flat = points.reshape(-1, msg.point_step)
    stride = max(1, math.ceil(len(flat) / max_points))
    selected = flat[::stride].copy()
    out = PointCloud2(header=msg.header, height=1, width=len(selected), fields=msg.fields,
                      is_bigendian=msg.is_bigendian, point_step=msg.point_step,
                      row_step=len(selected)*msg.point_step, is_dense=msg.is_dense)
    out.data = array('B', selected.tobytes())
    return out


class Preview(Node):
    def __init__(self, namespace='drone1', hz=5., width=640, max_points=16000):
        super().__init__('visualization_preview')
        self.period, self.width, self.max_points = 1./hz, width, max_points
        self.last = {}
        self.subscriptions_owned = []
        root = '/' + namespace.strip('/')
        for suffix, semantic in [('camera/rgb/image_raw', False), ('camera/semantic/image', True)]:
            stem = 'semantic' if semantic else 'rgb'
            raw = self.create_publisher(Image, root+'/preview/'+stem+'/image', QOS)
            compressed = self.create_publisher(CompressedImage, root+'/preview/'+stem+'/image/compressed', QOS)
            def callback(msg, key=stem, sem=semantic, pub=raw, comp=compressed):
                if not (pub.get_subscription_count() or comp.get_subscription_count()) or not self.ready(key):
                    return
                try:
                    reduced, pixels = resize_image(msg, self.width, sem)
                    if pub.get_subscription_count():
                        pub.publish(reduced)
                    if comp.get_subscription_count():
                        if msg.encoding == 'rgb8':
                            pixels = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
                        ext = '.png' if sem else '.jpg'
                        ok, encoded = cv2.imencode(ext, pixels,
                            [cv2.IMWRITE_PNG_COMPRESSION, 1] if sem else [cv2.IMWRITE_JPEG_QUALITY, 80])
                        if not ok:
                            raise ValueError('Image compression failed')
                        format_ = f'{msg.encoding}; png compressed {"bgr8" if msg.encoding == "rgb8" else msg.encoding}' if sem else f'{msg.encoding}; jpeg compressed bgr8'
                        comp.publish(CompressedImage(header=msg.header, format=format_, data=encoded.tobytes()))
                except (ValueError, cv2.error) as exc:
                    self.get_logger().error(str(exc))
            self.subscriptions_owned.append(self.create_subscription(Image, root+'/'+suffix, callback, QOS))
        cloud_pub = self.create_publisher(PointCloud2, root+'/preview/lidar/points', QOS)
        def cloud(msg):
            if cloud_pub.get_subscription_count() and self.ready('cloud'):
                try:
                    cloud_pub.publish(thin_cloud(msg, self.max_points))
                except ValueError as exc:
                    self.get_logger().error(str(exc))
        self.subscriptions_owned.append(self.create_subscription(PointCloud2, root+'/lidar/points', cloud, QOS))
        self.get_logger().info(f'Preview <= {hz:g} Hz, width <= {width}, points <= {max_points}; source stamps preserved')

    def ready(self, key):
        now = time.perf_counter()
        if now < self.last.get(key, 0) + self.period:
            return False
        self.last[key] = now
        return True


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--namespace', default='drone1')
    parser.add_argument('--hz', type=float, default=5.)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--max-points', type=int, default=16000)
    opts, ros_args = parser.parse_known_args(args)
    if opts.hz <= 0 or opts.width < 1 or opts.max_points < 1:
        parser.error('hz, width and max-points must be positive')
    rclpy.init(args=ros_args)
    node = Preview(opts.namespace, opts.hz, opts.width, opts.max_points)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
