"""ROS node contracts in an isolated domain; no RPC and no inference substitutes."""
from pathlib import Path
import sys
import time
import unittest
import copy
import json
from unittest.mock import patch
import yaml
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2,PointField,CameraInfo
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from evtol_perception_interfaces.msg import ReferenceOrigin,SemanticFrame,StateStatus,VelocityCommand

ROOT=Path(__file__).resolve().parents[1]
for package in ('evtol_targeting','evtol_safety'):
    sys.path.insert(0,str(ROOT/'src'/package))
from evtol_targeting.common import header
from evtol_targeting.localizer_node import Localizer
from evtol_targeting.configuration import validate_config
from evtol_safety.node import SafetyNode
from evtol_safety.gateway import Gateway


class Capture:
    def __init__(self):self.messages=[]
    def publish(self,msg):self.messages.append(copy.deepcopy(msg))


def cloud(t,points):
    points=np.asarray(points,dtype='<f4').reshape(-1,3)
    return PointCloud2(header=header(t,'drone1/lidar_link'),height=1,width=len(points),
        fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z'))],
        point_step=12,row_step=len(points)*12,data=points.tobytes())


class NodeContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):rclpy.init(domain_id=227)
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()

    def config(self):
        c=yaml.safe_load((ROOT/'config/targeting.yaml').read_text(encoding='utf8'));c['_root']=ROOT
        # Legacy front-only contracts are separate from the full-coverage
        # navigation/mission contracts below.
        c.get('navigation_lidar',{})['enabled']=False
        return c

    def test_configuration_rejects_unsafe_budgets(self):
        for section,key,value in [('control','duration_s',1.),('model','max_hz',0.),('safety','observed_half_pitch_deg',90.)]:
            c=self.config();c[section][key]=value
            with self.assertRaises(ValueError):validate_config(c)

    def test_node_localization_and_unconfirmed_track_contract(self):
        c=self.config();c['extrinsics']['camera']['xyz_frd_m']=[0.,0.,0.];c['extrinsics']['camera']['rpy_deg']=[0.,0.,0.]
        node=Localizer(c)
        try:
            node.measurements=Capture();node.tracks=Capture();node.markers=Capture()
            t=node.get_clock().now().nanoseconds
            node.receive_origin(ReferenceOrigin(session_id='isolated-test'))
            pose=Odometry(header=header(t));pose.pose.pose.orientation.w=1.;node.receive_pose(pose)
            node.receive_info(CameraInfo(width=200,height=200,k=[100.,0.,100.,0.,100.,100.,0.,0.,1.]))
            node.receive_cloud(cloud(t,[[10.,-.1,0.],[10.,0.,0.],[10.,.1,0.]]))
            mask=SemanticFrame(header=header(t),width=200,height=200)
            mask.class_ids.frombytes(np.full((200,200),6,dtype='<i4').tobytes())
            node.receive_mask(mask);node.process()
            result=node.measurements.messages[-1]
            self.assertEqual(result.status,'OK');self.assertEqual(len(result.targets),1)
            self.assertAlmostEqual(result.targets[0].position.x,10.,places=4)
            self.assertEqual(result.header.frame_id,'map_enu');self.assertEqual(result.targets[0].state,'tentative')
            node.predict();self.assertEqual(node.tracks.messages[-1].targets[0].state,'tentative')
        finally:node.destroy_node()

    def test_safety_keeps_checking_paused_clock_and_empty_cloud_stops(self):
        c=self.config();c['mission']['waypoints_enu_m']=[[20.,0.,0.]]
        node=SafetyNode(c)
        try:
            node.pub=Capture();node.risk_pub=Capture();node.desired_pub=Capture()
            node.receive_origin(ReferenceOrigin(session_id='isolated-test'))
            node.set_parameters([Parameter('use_sim_time',value=True)])
            # Let DDS discovery and the first diagnostic/log call complete
            # before measuring the paused-clock timer contract.
            warmup=time.perf_counter()
            while time.perf_counter()-warmup<.5:rclpy.spin_once(node,timeout_sec=.02)
            node.pub.messages.clear()
            start=time.perf_counter()
            while time.perf_counter()-start<.2:rclpy.spin_once(node,timeout_sec=.02)
            self.assertGreaterEqual(len(node.pub.messages),3)
            self.assertEqual(node.get_clock().now().nanoseconds,0)
            node.set_parameters([Parameter('use_sim_time',value=False)])
            t=node.get_clock().now().nanoseconds
            pose=Odometry(header=header(t));pose.pose.pose.orientation.w=1.;node.receive_pose(pose)
            node.receive_status(StateStatus(session_id='isolated-test',valid=True))
            node.receive_cloud(cloud(t,[[10.,0.,0.]]));node.tick()
            self.assertIn(node.pub.messages[-1].reason,('clear','avoidance'))
            node.receive_cloud(cloud(t+1,[]));node.tick()
            cmd=node.pub.messages[-1]
            self.assertIn('Empty LiDAR',cmd.reason)
            self.assertEqual((cmd.velocity.x,cmd.velocity.y,cmd.velocity.z),(0.,0.,0.))
        finally:node.destroy_node()

    def test_shadow_gateway_rejects_replayed_cross_session_and_unsafe_commands(self):
        node=Gateway(self.config(),execute=False)
        try:
            self.assertIsNone(node.worker);self.assertIsNone(node.lease)
            node.receive_origin(ReferenceOrigin(session_id='isolated-test'))
            cmd=VelocityCommand(header=header(node.get_clock().now().nanoseconds),session_id='isolated-test',sequence=1)
            cmd.velocity.x=1.;node.receive(cmd);self.assertEqual(node.sequence,1)
            for sequence,session,speed in [(1,'isolated-test',2.),(2,'other-test',1.),(2,'isolated-test',float('nan')),(2,'isolated-test',3.)]:
                bad=copy.deepcopy(cmd);bad.sequence=sequence;bad.session_id=session;bad.velocity.x=speed
                node.receive(bad);self.assertEqual(node.sequence,1);self.assertEqual(node.command.velocity.x,1.)
        finally:node.close();node.destroy_node()

    def navigation_node(self,goal,position=(0.,0.,3.),points=((8.,0.,0.),),phase='flying'):
        c=self.config();c['navigation_lidar']['enabled']=True;c['mission']['waypoints_enu_m']=[goal]
        c['navigation_lidar']['minimum_sweep_s']=0.
        node=SafetyNode(c);node.pub=Capture();node.risk_pub=Capture();node.desired_pub=Capture()
        node.receive_origin(ReferenceOrigin(session_id='navigation-test'))
        t=node.get_clock().now().nanoseconds
        pose=Odometry(header=header(t));pose.pose.pose.orientation.w=1.
        pose.pose.pose.position.x,pose.pose.pose.position.y,pose.pose.pose.position.z=map(float,position)
        node.receive_pose(pose);node.receive_status(StateStatus(session_id='navigation-test',valid=True))
        node.receive_flight(String(data=json.dumps(dict(session_id='navigation-test',phase=phase))))
        msg=cloud(t,points);msg.header.frame_id='drone1/base_link';node.receive_cloud(msg)
        return node

    def test_full_navigation_climbs_descends_and_flies_rearward(self):
        for goal in ([0.,0.,6.],[0.,0.,1.5],[-4.,0.,3.],[2.,-3.,4.]):
            node=self.navigation_node(goal)
            try:
                node.tick();cmd=node.pub.messages[-1]
                velocity=np.array([cmd.velocity.x,cmd.velocity.y,cmd.velocity.z])
                self.assertIn(cmd.reason,('clear','avoidance'))
                self.assertGreater(velocity@(np.asarray(goal)-[0.,0.,3.]),0.)
            finally:node.destroy_node()

    def test_navigation_takeoff_excludes_only_starting_support(self):
        node=self.navigation_node([0.,0.,3.],position=(0.,0.,0.),points=((0.,0.,-.2),(5.,0.,2.)),phase='takeoff')
        try:
            node.tick();self.assertGreater(node.pub.messages[-1].velocity.z,0.)
            self.assertEqual(node.mission.index,0)
        finally:node.destroy_node()

    def test_landing_requires_observed_support_and_clear_corridor(self):
        for points,expected in [(((0.,0.,-1.4),),'mission_land'),(((5.,0.,0.),),'landing_surface_not_observed'),
                (((0.,0.,-1.4),(.2,0.,-.5)),'landing_corridor_obstructed')]:
            node=self.navigation_node([0.,0.,0.],position=(0.,0.,1.2),points=points,phase='landing')
            try:
                node.tick();self.assertEqual(node.pub.messages[-1].reason,expected)
            finally:node.destroy_node()

    def test_stale_flight_status_holds_navigation(self):
        node=self.navigation_node([0.,0.,6.]);node.flight_wall=time.perf_counter()-2.
        try:
            node.tick();cmd=node.pub.messages[-1]
            self.assertEqual(cmd.reason,'mission_waiting');self.assertEqual(cmd.velocity.z,0.)
        finally:node.destroy_node()

    def test_partial_sweeps_keep_obstacle_from_previous_frame(self):
        node=self.navigation_node([0.,0.,5.]);node.c['navigation_lidar']['minimum_sweep_s']=.04
        try:
            node.history.samples.clear();node.cloud_history.clear();node.last_cloud_ns=0
            t=node.get_clock().now().nanoseconds
            for timestamp,points in ((t-50_000_000,[[0.,0.,2.]]),(t,[[0.,0.,-3.]])):
                pose=Odometry(header=header(timestamp));pose.pose.pose.orientation.w=1.;pose.pose.pose.position.z=3.
                node.receive_pose(pose);msg=cloud(timestamp,points);msg.header.frame_id='drone1/base_link';node.receive_cloud(msg)
            node.tick();self.assertEqual(node.pub.messages[-1].reason,'goal_obstructed')
        finally:node.destroy_node()

    def test_gateway_controlled_takeoff_and_verified_touchdown(self):
        self.gateway_flight(False)

    def test_gateway_refuses_disarm_if_land_returns_above_ground(self):
        self.gateway_flight(True)

    def gateway_flight(self,above_ground):
        c=self.config();c['navigation_lidar']['enabled']=True;c['control']['ttl_s']=1.
        c['mission']['waypoints_enu_m']=[[0.,0.,3.],[0.,0.,0.]]
        node=Gateway(c,execute=False);calls=[]
        def refresh(reason):
            t=node.get_clock().now().nanoseconds
            from evtol_targeting.common import stamp
            node.command=VelocityCommand(header=header(t),session_id='gateway-test',sequence=1,reason=reason,valid_until=stamp(t+int(1e9)))
            node.received=node.state_wall=time.perf_counter();node.state_ok=True
        node.execute=True;node.phase='waiting';node.receive_origin(ReferenceOrigin(session_id='gateway-test'));refresh('mission_waiting')
        transition=node.transition
        def phase(value):
            transition(value)
            if value=='flying':refresh('mission_land')
        node.transition=phase
        class RPC:
            def __init__(self,*args):pass
            def call(self,method,*args):
                calls.append((method,args))
                if method=='getMultirotorState':
                    z=-1.5 if node.phase=='takeoff' else (-2. if above_ground and node.phase=='landing' else 0.)
                    return dict(landed_state=0,kinematics_estimated=dict(position=dict(x_val=0.,y_val=0.,z_val=z),linear_velocity=dict(x_val=0.,y_val=0.,z_val=0.)))
                return True
            def close(self):pass
        try:
            with patch('evtol_safety.gateway.AirSimRPC',RPC):node.run()
            self.assertFalse(any(name=='takeoff' for name,args in calls))
            if above_ground:
                self.assertEqual(node.phase,'failed');self.assertIn('refusing airborne disarm',node.failure)
                self.assertNotIn(('armDisarm',(False,'Drone1')),calls)
            else:
                self.assertEqual(node.phase,'landed')
                self.assertIn(('armDisarm',(False,'Drone1')),calls)
                self.assertIn(('enableApiControl',(False,'Drone1')),calls)
        finally:node.destroy_node()


if __name__=='__main__':unittest.main()
