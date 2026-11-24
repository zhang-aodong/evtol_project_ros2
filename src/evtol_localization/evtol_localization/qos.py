"""Shared acquisition/evaluation QoS, independent of estimator implementation."""
from rclpy.qos import QoSProfile, ReliabilityPolicy

SENSOR_QOS = QoSProfile(depth=128, reliability=ReliabilityPolicy.BEST_EFFORT)
