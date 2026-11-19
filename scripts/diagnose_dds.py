"""Run the sensor/algorithm stack in shadow mode; never arm or start actors."""
import argparse
from collections import Counter
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds', type=float, default=60.)
    p.add_argument('--warmup', type=float, default=15.)
    p.add_argument('--domain', type=int, default=0)
    p.add_argument('--profile', type=Path)
    p.add_argument('--output', type=Path, default=Path('reports/dds_stability.json'))
    a = p.parse_args()
    if a.seconds <= 0 or a.warmup < 0 or not 0 <= a.domain <= 232:
        p.error('Invalid duration or ROS domain')
    root = Path(__file__).resolve().parents[1]
    os.chdir(root)
    a.output = a.output.resolve()
    a.output.parent.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_DOMAIN_ID'] = str(a.domain)
    if a.profile:
        os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(a.profile.resolve(strict=True))
        os.environ['FASTDDS_DEFAULT_PROFILES_FILE'] = str(a.profile.resolve(strict=True))
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, Imu, NavSatFix, PointCloud2
    from nav_msgs.msg import Odometry
    from evtol_sim_interfaces.msg import GnssObservations, FiveGObservations
    from evtol_perception_interfaces.msg import SemanticFrame
    rclpy.init()
    node = Node('dds_stability_audit')
    counts = Counter()
    arrivals = {}
    sizes = {}
    begin = time.perf_counter()
    def receive(topic, msg):
        now = time.perf_counter()
        if now - begin < a.warmup:
            return
        counts[topic] += 1
        arrivals.setdefault(topic, []).append(now)
        if isinstance(msg, Image):
            sizes[topic] = [msg.width, msg.height, len(msg.data)]
    topics = [
        ('ground_truth/odometry', Odometry), ('camera/rgb/image_raw', Image),
        ('camera/semantic/image', Image), ('lidar/points', PointCloud2),
        ('navigation/lidar/points', PointCloud2), ('imu/data', Imu),
        ('gnss/fix', NavSatFix), ('gnss/raw_observations', GnssObservations),
        ('five_ga/observations', FiveGObservations), ('state/odometry', Odometry),
        ('perception/semantic', SemanticFrame)]
    subscriptions = [node.create_subscription(cls, '/drone1/' + topic,
        lambda msg, topic=topic: receive(topic, msg), qos_profile_sensor_data)
        for topic, cls in topics]
    sim = str(root / 'src/evtol_simulation/config/simulation.perception_2k.yaml')
    config = str(root / 'config/targeting.yaml')
    specs = [(kind + '_sensor', 'evtol_simulation.sensor_node',
        ['--config', sim, '--kind', kind, '--backend', 'airsim'])
        for kind in ('truth', 'camera', 'lidar', 'imu', 'gnss', 'radio')]
    specs += [(name, module, ['--config', config] + extra +
        ['--ros-args', '-p', 'use_sim_time:=true']) for name, module, extra in [
        ('navigation_lidar', 'evtol_simulation.safety_lidar', []),
        ('state', 'evtol_targeting.state_node', []),
        ('trajectory', 'evtol_targeting.trajectory_node', []),
        ('vision', 'evtol_targeting.vision_node', []),
        ('localizer', 'evtol_targeting.localizer_node', []),
        ('safety', 'evtol_safety.node', []),
        ('gateway_shadow', 'evtol_safety.gateway', []),
        ('target_truth', 'evtol_targeting.evaluation', ['--truth']),
        ('evaluation', 'evtol_targeting.evaluation', [])]]
    children = []
    logs = []
    failed = {}
    try:
        for name, module, extra in specs:
            log = a.output.with_name(a.output.stem + '_' + name + '.log').open('w', encoding='utf8')
            logs.append(log)
            command = [sys.executable, '-u', '-X', 'faulthandler', '-c',
                'from ' + module + ' import main; main()'] + extra
            child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                creationflags=subprocess.CREATE_NO_WINDOW)
            children.append((name, child))
        last = begin
        while time.perf_counter() - begin < a.warmup + a.seconds:
            rclpy.spin_once(node, timeout_sec=.1)
            failed = {name: child.returncode for name, child in children if child.poll() is not None}
            if failed:
                print('Node exited: ' + json.dumps(failed), flush=True)
                break
            if time.perf_counter() - last >= 10:
                print(json.dumps(dict(elapsed_s=round(time.perf_counter()-begin, 1),
                    samples=dict(counts))), flush=True)
                last = time.perf_counter()
        duration = max(0., time.perf_counter() - begin - a.warmup)
        report = dict(domain=a.domain, profile=os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE'),
            rmw=os.environ.get('RMW_IMPLEMENTATION', 'default'),
            measurement_s=duration, node_exits=failed, image_sizes=sizes, topics={})
        for topic, _ in topics:
            times = arrivals.get(topic, [])
            report['topics'][topic] = dict(samples=counts[topic],
                hz=(len(times)-1)/(times[-1]-times[0]) if len(times)>1 else 0.)
        # Stability is distinct from all-frequency and flight acceptance.
        report['result'] = ('PASS_STABILITY' if not failed and duration >= a.seconds
            and all(counts[topic] > 0 for topic, _ in topics) else 'FAIL_STABILITY')
        a.output.write_text(json.dumps(report, indent=2), encoding='utf8')
        print(json.dumps(report, indent=2), flush=True)
        return 0 if report['result'] == 'PASS_STABILITY' else 1
    finally:
        # Only this harness's shadow process trees; no vehicle control or UE.
        for name, child in reversed(children):
            if child.poll() is None:
                subprocess.run(['taskkill', '/PID', str(child.pid), '/T', '/F'],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
            child.wait(timeout=10)
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
