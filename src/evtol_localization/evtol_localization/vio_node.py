"""ROS adapter for the independently published, startup-relative ego VIO."""
from collections import OrderedDict

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu

from evtol_simulation.coordinates import matrix_quaternion
from evtol_targeting.common import Health, args_config, header, ns, put, xyz
from .vio import Calibration, LocalVio, gray_image


class VioNode(Node):
    def __init__(self, config):
        super().__init__("vio_node")
        self.vehicle = config["namespace"].strip("/")
        root = "/" + self.vehicle
        options = config.get("localization", {}).get("vio", {})
        interval = float(options.get("keyframe_interval_s", .25))
        self.max_width = int(options.get("max_image_width", 640))
        if not .05 <= interval <= 1. or not 96 <= self.max_width <= 2048:
            raise ValueError("VIO keyframe interval or maximum width out of range")
        self.interval = round(interval * 1e9)
        camera = config["extrinsics"]["camera"]
        self.estimator = LocalVio(camera["xyz_frd_m"], camera["rpy_deg"])
        self.body = self.vehicle + "/base_link"
        self.imu_frame = self.vehicle + "/imu_link"
        self.optical = self.vehicle + "/camera_optical_frame"
        self.last_image = None
        self.last_received_image = self.last_received_info = None
        self.images = OrderedDict()
        self.infos = OrderedDict()
        self.health = Health(self, root + "/localization/vio")
        self.pub = self.create_publisher(Odometry, root + "/localization/vio/odometry", 10)
        imu_qos = QoSProfile(depth=128, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, root + "/imu/data", self.on_imu, imu_qos)
        self.create_subscription(CameraInfo, root + "/camera/rgb/camera_info", self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, root + "/camera/rgb/image_raw", self.on_image, qos_profile_sensor_data)

    def on_imu(self, msg):
        if msg.header.frame_id != self.imu_frame:
            self.health.error("Unexpected IMU frame")
            return
        est = self.estimator
        prior_resets = est.resets
        if not est.add_imu(ns(msg.header.stamp), xyz(msg.linear_acceleration), xyz(msg.angular_velocity)):
            self.health.dropped += 1
            return
        if est.resets != prior_resets:
            self.last_image = None
            self.last_received_image = self.last_received_info = None
            self.images.clear()
            self.infos.clear()
            self.health.last_error = f"{est.last_reset}: waiting for static IMU ({est.frame})"
            self.get_logger().warn(self.health.last_error)
        self.process()

    def on_info(self, msg):
        self.last_received_info = ns(msg.header.stamp)
        if msg.header.frame_id != self.optical:
            self.health.error("Unexpected CameraInfo frame")
            return
        try:
            self.infos[ns(msg.header.stamp)] = Calibration.from_info(msg)
        except ValueError as exc:
            self.health.error(exc)
            return
        while len(self.infos) > 64:
            self.infos.popitem(last=False)
        self.process()

    def on_image(self, msg):
        stamp = ns(msg.header.stamp)
        self.last_received_image = stamp
        if self.estimator.cursor is None:
            return
        if self.last_image is not None and stamp - self.last_image < self.interval:
            return
        if msg.header.frame_id != self.optical:
            self.health.error("Unexpected image frame")
            return
        try:
            gray = gray_image(msg, self.max_width)
        except (ValueError, cv2.error) as exc:
            self.health.error(exc)
            return
        self.last_image = stamp
        self.images[stamp] = (gray, msg.width, msg.height)
        while len(self.images) > 4:
            self.images.popitem(last=False)
            self.health.dropped += 1
        self.process()

    def process(self):
        est = self.estimator
        self.health.details = {"tracking": est.reason, "accepted_edges": est.accepted,
                               "image_stamp_ns": self.last_received_image,
                               "camera_info_stamp_ns": self.last_received_info,
                               "imu_stamp_ns": est.last_stamp,
                               "frame": est.frame, "last_reset": est.last_reset, "resets": est.resets}
        if est.cursor is None:
            return
        for stamp in list(self.images):
            if stamp < est.cursor.stamp:
                self.images.pop(stamp)
                self.health.dropped += 1
                continue
            if stamp > est.last_stamp:
                break
            if stamp not in self.infos:
                continue
            gray, width, height = self.images.pop(stamp)
            cal = self.infos.pop(stamp)
            if (width, height) != (cal.width, cal.height):
                self.health.error("Image and CameraInfo dimensions differ")
                continue
            try:
                accepted = est.image(stamp, gray, cal)
            except (ValueError, cv2.error, np.linalg.LinAlgError) as exc:
                self.health.error(exc)
                continue
            self.health.details.update(tracking=est.reason, accepted_edges=est.accepted)
            if accepted:
                self.publish_odometry(stamp)
                self.health.ok()
            else:
                self.health.last_error = est.reason

    def publish_odometry(self, stamp):
        est = self.estimator
        msg = Odometry(header=header(stamp, est.frame), child_frame_id=self.body)
        put(msg.pose.pose.position, est.x[:3])
        q = msg.pose.pose.orientation
        q.x, q.y, q.z, q.w = map(float, matrix_quaternion(est.rotation))
        put(msg.twist.twist.linear, est.rotation.T @ est.x[3:6])
        put(msg.twist.twist.angular, est.cursor.gyro - est.gyro_bias)
        # Conditional translation covariances omit attitude errors. Attitude and
        # angular-rate uncertainty are not estimated by this direction filter.
        pose_cov, twist_cov = np.eye(6) * 1.e6, np.eye(6) * 1.e6
        pose_cov[:3, :3] = est.cov[:3, :3]
        twist_cov[:3, :3] = est.rotation.T @ est.cov[3:6, 3:6] @ est.rotation
        msg.pose.covariance = pose_cov.ravel().tolist()
        msg.twist.covariance = twist_cov.ravel().tolist()
        self.pub.publish(msg)


def main(args=None):
    config, ros = args_config(args)
    cv2.setNumThreads(1)
    rclpy.init(args=ros)
    node = VioNode(config)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
