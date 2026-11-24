"""Read-only live evaluator. Only this process subscribes to reference truth."""
import argparse
import json
import math
from pathlib import Path
import re
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args

from .metrics import summarize
from .settings import MODULES


def main(args=None):
    import sys
    argv = sys.argv if args is None else ["accuracy", *args]
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=MODULES, required=True)
    parser.add_argument("--namespace", default="drone1")
    parser.add_argument("--estimate-topic")
    parser.add_argument("--seconds", type=float, default=60.)
    parser.add_argument("--output", type=Path)
    opts = parser.parse_args(remove_ros_args(argv)[1:])
    if not math.isfinite(opts.seconds) or opts.seconds <= 0:
        parser.error("--seconds must be positive and finite")
    local = opts.mode == "camera_imu"
    root = "/" + opts.namespace.strip("/")
    suffix = "/localization/vio/odometry" if local else "/localization/odometry"
    topics = [root + "/ground_truth/local_odometry", opts.estimate_topic or root + suffix]
    if topics[0] == topics[1]:
        parser.error("Estimate and reference must be different topics")
    frames = ["map_enu", "vio_odom" if local else "map_enu"]
    tracks = [[], []]

    def record(index, msg):
        if index == 1 and local and not tracks[1]:
            if re.fullmatch(r"vio_odom(?:_epoch_[1-9][0-9]*)?", msg.header.frame_id):
                frames[1] = msg.header.frame_id
        if msg.header.frame_id != frames[index] or msg.child_frame_id != root[1:] + "/base_link":
            raise ValueError(f"Frame changed or unexpected on {topics[index]}: {msg.header.frame_id}, "
                             f"{msg.child_frame_id}. Evaluate each local VIO epoch separately.")
        t = int(msg.header.stamp.sec)*1_000_000_000 + msg.header.stamp.nanosec
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        if tracks[index]:
            if t == tracks[index][-1][0]:
                return
            if t < tracks[index][-1][0]:
                raise ValueError("Time moved backwards; evaluate one session at a time")
        tracks[index].append((t, (p.x, p.y, p.z), yaw))

    rclpy.init(args=argv[1:])
    node = Node("ego_localization_accuracy")
    try:
        for idx, topic in enumerate(topics):
            node.create_subscription(Odometry, topic, lambda m, i=idx: record(i, m), qos_profile_sensor_data)
        until = time.monotonic() + opts.seconds
        while rclpy.ok() and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=min(0.2, max(0., until - time.monotonic())))
        if any(len(track) < 2 for track in tracks):
            raise SystemExit(f"Insufficient samples on {topics}: {[len(t) for t in tracks]}")
        arrays = [tuple(np.asarray(values) for values in zip(*track)) for track in tracks]
        scores = summarize(*arrays, local)
        report = json.dumps(dict(mode=opts.mode, topics=topics, frames=frames, **scores), indent=2, allow_nan=False)
        print(report)
        if opts.output:
            opts.output.parent.mkdir(parents=True, exist_ok=True)
            opts.output.write_text(report + "\n", encoding="utf8")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
