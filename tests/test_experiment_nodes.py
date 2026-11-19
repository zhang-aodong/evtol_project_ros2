"""Real DDS transport and async worker contracts. No UE or control RPC."""
import copy
import json
from pathlib import Path
import tempfile
import time
import unittest
import numpy as np
import yaml
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from evtol_perception_interfaces.msg import ReferenceOrigin,TargetArray,Target
from evtol_targeting.common import header,stamp,put,LATCHED
from evtol_targeting.experiment_node import ExperimentEvaluator
from evtol_safety.async_navigation import AsyncNavigator
from evtol_safety.dynamic import from_targets,DEFAULTS as DYNAMIC

ROOT=Path(__file__).resolve().parents[1]


class AsyncWorker(unittest.TestCase):
    def test_stale_and_invalid_fused_states_cannot_reenter_controller(self):
        t=1_000_000_000;target=Target(track_id=1,state='confirmed',last_measurement_stamp=stamp(t))
        put(target.position,[5,0,0]);put(target.size,[1,1,.5]);target.covariance=(np.eye(6)*.1).ravel().tolist()
        self.assertEqual(len(from_targets([target],t,t,np.zeros(3),DYNAMIC)[0]),1)
        self.assertEqual(from_targets([target],t,t+1_000_000_000,np.zeros(3),DYNAMIC)[1],1)
        target.covariance[0]=-1.
        self.assertEqual(from_targets([target],t,t,np.zeros(3),DYNAMIC)[1],1)
    def test_spawned_process_returns_route_without_blocking_request(self):
        nav=AsyncNavigator(dict(resolution_m=.5,planning_radius_m=10,clearance_m=1,max_nodes=1000,
            search_budget_s=.05,lookahead_m=1.5,route_timeout_s=.75))
        try:
            start=time.monotonic();nav.plan(np.zeros(3),[5,0,0],[[0,5,0]])
            self.assertLess(time.monotonic()-start,1.,'Spawn or submission blocked the safety caller')
            deadline=time.monotonic()+8;aim=None
            while time.monotonic()<deadline and aim is None:
                time.sleep(.03);aim=nav.plan(np.zeros(3),[5,0,0],[[0,5,0]])
            self.assertIsNotNone(aim);self.assertGreater(aim[0],0)
            # A changed goal must not reuse the previous route.
            self.assertIsNone(nav.plan(np.zeros(3),[-5,0,0],[[0,5,0]]))
        finally:nav.close()
        self.assertIsNone(nav.process)


class DDSAudit(unittest.TestCase):
    @classmethod
    def setUpClass(cls):rclpy.init(domain_id=219)
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()

    def test_fixed_rate_statistics_flow_over_real_ros_topics(self):
        with tempfile.TemporaryDirectory() as tmp:
            c=yaml.safe_load((ROOT/'config/targeting.yaml').read_text(encoding='utf8'))
            c['_root']=ROOT;c['evaluation']['output_dir']=tmp
            c['experiment']=dict(buffer_s=.15,truth_max_gap_s=.2,score_only_flying=False)
            actor=yaml.safe_load((ROOT/'config/actors.yaml').read_text(encoding='utf8'))
            actor['interference_drones']['agents']=actor['interference_drones']['agents'][:2]
            path=Path(tmp)/'actors.yaml';path.write_text(yaml.safe_dump(actor),encoding='utf8');c['actors_config']=str(path)
            node=ExperimentEvaluator(c);source=rclpy.create_node('audit_fixture_source')
            executor=SingleThreadedExecutor();executor.add_node(node);executor.add_node(source)
            messages=[];subscription=source.create_subscription(String,'/evaluation/detection_metrics',lambda m:messages.append(json.loads(m.data)),10)
            origin=source.create_publisher(ReferenceOrigin,'/simulation/reference_origin',LATCHED)
            ego=source.create_publisher(Odometry,'/drone1/state/odometry',qos_profile_sensor_data)
            truth=source.create_publisher(TargetArray,'/simulation/truth/targets',qos_profile_sensor_data)
            detections=source.create_publisher(TargetArray,'/drone1/perception/targets_measurements',qos_profile_sensor_data)
            try:
                session='range-real-dds';origin.publish(ReferenceOrigin(session_id=session))
                deadline=time.monotonic()+3.;last=0.
                while time.monotonic()<deadline:
                    if time.monotonic()-last>.045:
                        last=time.monotonic();t=source.get_clock().now().nanoseconds
                        odom=Odometry(header=header(t));odom.pose.pose.orientation.w=1.;ego.publish(odom)
                        arr=TargetArray(header=header(t),session_id=session)
                        for i,x in [(1,5.),(2,10.)]:
                            target=Target(track_id=i,class_id=6,state='truth',measured=True,last_measurement_stamp=stamp(t))
                            put(target.position,[x,0,0]);arr.targets.append(target)
                        truth.publish(arr)
                        observed=copy.deepcopy(arr);observed.targets=observed.targets[:1];observed.lidar_stamp=stamp(t);observed.image_stamp=stamp(t)
                        observed.targets[0].track_id=99;detections.publish(observed)
                    executor.spin_once(timeout_sec=.01)
                valid=[m for m in messages if m['valid'] and m['detected_count']==1]
                self.assertTrue(valid,'No valid measured result traversed DDS')
                self.assertEqual(valid[-1]['truth_count'],2);self.assertEqual(valid[-1]['missed_count'],1)
                self.assertAlmostEqual(valid[-1]['recall'],.5)
                # Stopped perception, truth still arriving: the audit must count misses.
                until=time.monotonic()+.9
                while time.monotonic()<until:
                    t=source.get_clock().now().nanoseconds
                    odom=Odometry(header=header(t));odom.pose.pose.orientation.w=1.;ego.publish(odom)
                    arr.header=header(t)
                    for target in arr.targets:target.last_measurement_stamp=stamp(t)
                    truth.publish(arr);executor.spin_once(timeout_sec=.02);time.sleep(.02)
                self.assertEqual([m for m in messages if m['valid']][-1]['detected_count'],0)
                self.assertTrue((node.output/'metrics.jsonl').stat().st_size>0)
            finally:
                node.close();executor.shutdown();node.destroy_node();source.destroy_node()


if __name__=='__main__':unittest.main()
