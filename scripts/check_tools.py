"""Exercise RViz startup and actual MCAP recording/playback using explicit mock data.

Run in the activated project environment. Does not certify UE sensor performance.
"""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
import rosbag2_py
from sensor_msgs.msg import Image, Imu, PointCloud2, NavSatFix
from nav_msgs.msg import Odometry
from evtol_sim_interfaces.msg import GnssObservations, FiveGObservations


TOPICS = {
    '/drone1/camera/rgb/image_raw': Image,
    '/drone1/camera/semantic/image': Image,
    '/drone1/camera/semantic/class_ids': Image,
    '/drone1/lidar/points': PointCloud2,
    '/drone1/imu/data': Imu,
    '/drone1/gnss/fix': NavSatFix,
    '/drone1/gnss/raw_observations': GnssObservations,
    '/drone1/five_ga/observations': FiveGObservations,
    '/drone1/ground_truth/odometry': Odometry,
}
EXTRA = ['/clock', '/tf', '/tf_static', '/diagnostics',
         '/drone1/camera/rgb/camera_info', '/drone1/camera/semantic/camera_info']


def stop_tree(process):
    if process is not None and process.poll() is None:
        subprocess.run(['taskkill', '/PID', str(process.pid), '/T', '/F'],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
        process.wait(timeout=10)


def spin_for(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.05)


def main():
    root = Path(__file__).resolve().parents[1]
    reports = root / 'reports'
    reports.mkdir(exist_ok=True)
    bag = reports / ('tools_bag_' + time.strftime('%Y%m%d_%H%M%S'))
    ros = shutil.which('ros2')
    rviz_exe = shutil.which('rviz2')
    if not ros or not rviz_exe:
        raise RuntimeError('Activate scripts/activate.ps1 first')
    os.environ['ROS_DOMAIN_ID'] = '74'
    os.environ['OPENBLAS_NUM_THREADS'] = '1'
    processes, logs = [], []
    result = {'backend': 'mock', 'bag': str(bag), 'passed': False}

    def start(name, command):
        log = (reports / (name + '.log')).open('w', encoding='utf8')
        logs.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   creationflags=subprocess.CREATE_NO_WINDOW)
        processes.append(process)
        return process

    rclpy.init(domain_id=74)
    node = rclpy.create_node('tools_smoke_observer')
    counts = {topic: 0 for topic in TOPICS}
    def receive(topic):
        def callback(_message):
            counts[topic] += 1
        return callback
    subscriptions = [node.create_subscription(cls, topic, receive(topic), qos_profile_sensor_data)
                     for topic, cls in TOPICS.items()]
    try:
        launch = start('tools_mock', [ros, 'launch', 'evtol_bringup', 'simulation.launch.py',
                                     'backend:=mock', 'rviz:=false'])
        spin_for(node, 6)
        rviz = start('tools_rviz', [rviz_exe, '-d',
            str(root/'src'/'evtol_bringup'/'rviz'/'simulation.rviz'),
            '--ros-args', '-p', 'use_sim_time:=true'])
        endpoints = {}
        ready_deadline = time.monotonic()+30
        while time.monotonic() < ready_deadline and rviz.poll() is None:
            spin_for(node, .2)
            for topic in ['/drone1/lidar/points', '/drone1/camera/rgb/image_raw',
                          '/drone1/camera/semantic/image', '/drone1/ground_truth/odometry']:
                endpoints[topic] = [x.node_name for x in node.get_subscriptions_info_by_topic(topic)
                                    if x.node_name.startswith('rviz')]
            if all(endpoints.values()):
                break
        result['rviz_running'] = rviz.poll() is None
        result['rviz_subscriptions'] = endpoints
        recorder = start('tools_record', [sys.executable, str(root/'scripts'/'record_for.py'),
                          '--output', str(bag), '--duration', '8'])
        end = time.monotonic()+25
        while time.monotonic() < end and recorder.poll() is None:
            rclpy.spin_once(node, timeout_sec=.05)
        if recorder.poll() != 0:
            raise RuntimeError('Recorder failed to stop gracefully; inspect tools_record.log')
        if not (bag/'metadata.yaml').is_file():
            raise RuntimeError('Recorder did not finalize metadata.yaml')
        stop_tree(recorder)
        result['live_counts'] = dict(counts)
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                    rosbag2_py.ConverterOptions('', ''))
        recorded = {topic: 0 for topic in [*TOPICS, *EXTRA]}
        while reader.has_next():
            topic, _, _ = reader.read_next()
            recorded[topic] = recorded.get(topic, 0) + 1
        del reader
        result['recorded_counts'] = recorded
        # Remove the only live source before replay, so counts prove bag delivery.
        stop_tree(launch)
        spin_for(node, 1)
        for topic in counts:
            counts[topic] = 0
        player = start('tools_play', [ros, 'bag', 'play', str(bag),
            '--qos-profile-overrides-path', str(root/'src'/'evtol_bringup'/'config'/'bag_qos.yaml')])
        end = time.monotonic()+25
        while time.monotonic() < end and player.poll() is None:
            rclpy.spin_once(node, timeout_sec=.05)
        spin_for(node, .5)
        result['replayed_counts'] = dict(counts)
        result['playback_exit_code'] = player.poll()
        result['passed'] = (result['rviz_running'] and all(endpoints.values())
                            and all(recorded.values()) and all(counts.values())
                            and player.poll() == 0)
    except Exception as exc:
        result['error'] = f'{type(exc).__name__}: {exc}'
    finally:
        for process in reversed(processes):
            stop_tree(process)
        for log in logs:
            log.close()
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
        (reports/'tools_acceptance.json').write_text(json.dumps(result, indent=2), encoding='utf8')
        print(json.dumps(result, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
