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
            node.receive_cloud(cloud(t,[[10.,-.1,1.],[10.,0.,1.],[10.,.1,1.]]))
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

    def navigation_node(self,goal,position=(0.,0.,3.),points=((8.,0.,0.),),phase='flying',raw_radius=None):
        c=self.config();c['navigation_lidar']['enabled']=True;c['navigation']['async_planning']=False;c['mission']['waypoints_enu_m']=[goal]
        c['navigation_lidar']['minimum_sweep_s']=0.
        if raw_radius is not None:c['safety']['raw_emergency']['radius_m']=raw_radius
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

    def test_async_wait_keeps_sampled_speed_reference_and_velocity_safety(self):
        from evtol_safety.async_navigation import AsyncNavigator
        class Pending(AsyncNavigator):
            def plan(self,*args):self.reason='waiting_for_async_plan';return None
        node=self.navigation_node([20.,0.,3.])
        try:
            node.navigator=Pending(node.c['navigation'])
            node.tick()
            self.assertTrue(node.risk_pub.messages[-1].valid)
            desired=node.desired_pub.messages[-1].velocity
            self.assertAlmostEqual(np.linalg.norm([desired.x,desired.y,desired.z]),node.c['mission']['speed_mps'])
            self.assertEqual(node.safety_data['reference_fallback'],'local_velocity_while_planning')
            self.assertLessEqual(np.linalg.norm([node.pub.messages[-1].velocity.x,node.pub.messages[-1].velocity.y,node.pub.messages[-1].velocity.z]),node.c['mission']['speed_mps'])
        finally:node.destroy_node()

    def test_planning_is_two_hz_but_safety_runs_on_every_tick(self):
        node=self.navigation_node([20.,0.,3.])
        class Planner:
            path=[];reason='direct'
            def __init__(self):self.calls=0
            def plan(self,p,goal,points):
                self.calls+=1;self.path=[p.copy(),goal.copy()];return goal.copy()
        planner=Planner();node.navigator=planner;node.c['navigation']['replan_period_s']=.5
        base=time.perf_counter()
        try:
            for i in range(21):
                wall=base+i*.05
                with patch('evtol_safety.node.time.perf_counter',return_value=wall):
                    node.pose_wall=wall;node.cloud_wall=wall;node.flight_wall=wall
                    node.cloud_history=type(node.cloud_history)([(node.cloud,wall)],maxlen=8)
                    node.tick()
            self.assertEqual(planner.calls,3)
            self.assertEqual(len(node.pub.messages),21)
            self.assertTrue(all(m.valid for m in node.risk_pub.messages))
        finally:node.destroy_node()

    def test_navigation_takeoff_excludes_support_and_keeps_raised_obstacles(self):
        node=self.navigation_node([0.,0.,3.],position=(0.,0.,0.),points=((0.,0.,-.2),(5.,0.,2.)),phase='takeoff')
        try:
            node.tick();self.assertGreater(node.pub.messages[-1].velocity.z,0.)
            self.assertEqual(node.mission.index,0)
        finally:node.destroy_node()

    def test_takeoff_recovers_after_evasive_side_step_outside_old_cylinder(self):
        node=self.navigation_node([0.,0.,3.],position=(0.,2.5,.1),
            points=((1.,0.,-.3),(0.,.5,-.3),(5.,0.,2.)),phase='takeoff')
        node.mission.takeoff_target=np.array([0.,0.,1.5])
        try:
            node.tick();self.assertTrue(node.risk_pub.messages[-1].valid)
            self.assertGreater(node.pub.messages[-1].velocity.z,0.)
        finally:node.destroy_node()
        node=self.navigation_node([0.,0.,3.],position=(0.,0.,0.),points=((0.,0.,1.5),(0.,0.,-.2)),phase='takeoff')
        try:
            node.tick();self.assertEqual(node.pub.messages[-1].reason,'goal_obstructed')
            self.assertEqual(node.pub.messages[-1].velocity.z,0.)
        finally:node.destroy_node()

    def test_localizer_rejects_ground_patch_but_keeps_raised_and_upright_returns(self):
        for heights,expected in [((-.2,-.2,-.2),0),((1.,1.,1.),1),((-.2,.3,.8),1)]:
            c=self.config();c['extrinsics']['camera']['xyz_frd_m']=[0.,0.,0.];c['extrinsics']['camera']['rpy_deg']=[0.,0.,0.]
            node=Localizer(c)
            try:
                node.measurements=Capture();t=node.get_clock().now().nanoseconds
                node.receive_origin(ReferenceOrigin(session_id='ground-test'))
                pose=Odometry(header=header(t));pose.pose.pose.orientation.w=1.;node.receive_pose(pose)
                node.receive_info(CameraInfo(width=200,height=200,k=[100.,0.,100.,0.,100.,100.,0.,0.,1.]))
                node.receive_cloud(cloud(t,[[10.,y,z] for y,z in zip((-.1,0.,.1),heights)]))
                mask=SemanticFrame(header=header(t),width=200,height=200)
                mask.class_ids.frombytes(np.full((200,200),3,dtype='<i4').tobytes())
                node.receive_mask(mask);node.process()
                self.assertEqual(len(node.measurements.messages[-1].targets),expected)
            finally:node.destroy_node()

    def test_completed_mission_corrects_hover_altitude_drift(self):
        node=self.navigation_node([0.,0.,3.],position=(0.,0.,2.8))
        try:
            node.mission.completed=True;node.tick();cmd=node.pub.messages[-1]
            self.assertGreater(cmd.velocity.z,0.)
            self.assertEqual(cmd.reason,'mission_hold')
        finally:node.destroy_node()

    def test_completed_airborne_mission_keeps_side_collision_avoidance(self):
        from evtol_perception_interfaces.msg import TargetArray,Target
        from evtol_targeting.common import stamp
        node=self.navigation_node([0.,0.,3.])
        try:
            node.mission.completed=True
            t=node.get_clock().now().nanoseconds
            target=Target(track_id=1,class_id=6,state='confirmed',
                last_measurement_stamp=stamp(t),reference_kind='vision_lidar')
            target.position.y=5.;target.position.z=3.;target.velocity.y=-2.
            target.size.x=target.size.y=1.;target.size.z=.5
            msg=TargetArray(header=header(t),session_id='navigation-test',image_stamp=stamp(t),lidar_stamp=stamp(t),targets=[target])
            node.receive_tracks(msg);node.tick()
            self.assertEqual(node.pub.messages[-1].reason,'mission_complete')
            # With the short avoidance horizon, act once the measured side
            # approach actually threatens the next second of flight.
            target.position.y=3.4;msg.targets=[target];node.receive_lidar_tracks(msg);node.tracks=None;node.tick()
            self.assertEqual(node.pub.messages[-1].reason,'mission_complete')
            node.receive_tracks(msg);node.tick()
            cmd=node.pub.messages[-1]
            self.assertIn(cmd.reason,('dynamic_evasive_velocity','fused_dynamic_avoidance'),
                (node.safety_data,node.dynamic,node.health.details,node.risk_pub.messages[-1]))
            self.assertGreater(np.linalg.norm([cmd.velocity.x,cmd.velocity.y,cmd.velocity.z]),.05)
            node.flight_phase='stopped';node.tick()
            cmd=node.pub.messages[-1]
            np.testing.assert_allclose([cmd.velocity.x,cmd.velocity.y,cmd.velocity.z],0.)
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
            node.tick()
            self.assertTrue(node.safety_data['lidar_emergency']['active'])
            self.assertAlmostEqual(node.safety_data['lidar_emergency']['nearest_m'],2.)
            self.assertLessEqual(node.pub.messages[-1].velocity.z,0.)
        finally:node.destroy_node()

    def test_side_lidar_fallback_while_hovering_only_after_takeoff(self):
        node=self.navigation_node([0.,0.,3.],points=((0.,2.3,0.),),phase='takeoff',raw_radius=2.5)
        try:
            node.tick();self.assertFalse(node.safety_data['lidar_emergency']['armed'])
            self.assertEqual(node.planner.radius,2.)
            node.mission.completed=True
            node.receive_flight(String(data=json.dumps(dict(session_id='navigation-test',phase='flying'))))
            node.previous_wall=time.perf_counter()-.05
            node.tick()
            cmd=node.pub.messages[-1]
            self.assertEqual(cmd.reason,'lidar_emergency_escape')
            self.assertLess(cmd.velocity.y,0.)
            self.assertTrue(node.risk_pub.messages[-1].warning)
            self.assertEqual(node.flight_planner.radius,2.5)
            self.assertGreater(node.planner.radius,2.5)
            self.assertLessEqual(node.planner.speed_limit,node.c['control']['max_speed_mps'])
            self.assertAlmostEqual(node.risk_pub.messages[-1].nearest_obstacle_m,2.3,places=5)
            node.flight_wall=time.perf_counter()-2.
            node.tick();self.assertFalse(node.safety_data['lidar_emergency']['armed'])
            self.assertEqual(node.pub.messages[-1].velocity.y,0.)
        finally:node.destroy_node()

    def test_raw_risk_serializes_and_publishes_unsigned_track_id(self):
        from rclpy.serialization import serialize_message,deserialize_message
        from evtol_perception_interfaces.msg import Risk
        node=self.navigation_node([0.,20.,3.],points=((0.,.7,0.),))
        publisher=node.risk_pub
        # navigation_node uses captures. Also exercise a real ROS publisher:
        # Python setters alone did not detect the negative uint64 in Jazzy.
        real=node.create_publisher(Risk,'/isolated_raw_risk',10)
        try:
            node.tick();risk=publisher.messages[-1]
            self.assertTrue(risk.warning);self.assertEqual(risk.track_id,0)
            self.assertEqual(deserialize_message(serialize_message(risk),Risk).track_id,0)
            real.publish(risk)
        finally:node.destroy_node()

    def test_emergency_freezes_planner_and_waypoint_then_replans_same_goal(self):
        node=self.navigation_node([0.,20.,3.],points=((0.,1.8,0.),))
        try:
            with patch.object(node.navigator,'plan',wraps=node.navigator.plan) as planning:
                with patch.object(node.mission,'reference',wraps=node.mission.reference) as advance:
                    node.tick()
                    self.assertEqual(planning.call_count,0);self.assertEqual(advance.call_count,0)
            self.assertEqual(node.safety_data['control_mode'],'lidar_emergency')
            self.assertEqual(node.mission.index,0);self.assertTrue(node.safety_data['planning_paused'])
            self.assertNotEqual(node.pub.messages[-1].reason,'mission_land')
            # Repeated complete clear scans start and then satisfy release hold.
            node.raw_guard.o['release_hold_s']=.1
            for i in range(4):
                t=node.get_clock().now().nanoseconds
                pose=Odometry(header=header(t));pose.pose.pose.orientation.w=1.;pose.pose.pose.position.z=3.
                node.receive_pose(pose)
                msg=cloud(t,[[8.,0.,0.]]);msg.header.frame_id='drone1/base_link';node.receive_cloud(msg)
                node.receive_flight(String(data=json.dumps(dict(session_id='navigation-test',phase='flying'))))
                node.tick();time.sleep(.04)
            self.assertFalse(node.safety_data['planning_paused'])
            self.assertGreater(node.plan_count,0);self.assertEqual(node.mission.index,0)
            np.testing.assert_allclose(node.plan_goal,[0.,20.,3.])
        finally:node.destroy_node()

    def test_emergency_near_landing_handoff_never_requests_land(self):
        node=self.navigation_node([0.,0.,0.],position=(0.,0.,1.2),points=((0.,1.8,0.),(0.,0.,-1.2)))
        try:
            node.mission.settled_since=time.perf_counter()-1.
            node.mission.landing_started=True
            node.tick()
            self.assertEqual(node.mission.index,0);self.assertFalse(node.mission.landing_started)
            self.assertEqual(node.mission.phase,'emergency')
            self.assertNotEqual(node.pub.messages[-1].reason,'mission_land')
        finally:node.destroy_node()

    def test_normal_route_uses_fused_geometry_before_raw_emergency_range(self):
        from evtol_perception_interfaces.msg import TargetArray,Target
        from evtol_targeting.common import stamp
        node=self.navigation_node([0.,20.,3.])
        try:
            t=node.get_clock().now().nanoseconds
            target=Target(track_id=42,class_id=6,state='confirmed',last_measurement_stamp=stamp(t))
            target.position.y=4.;target.position.z=3.
            target.size.x=target.size.y=1.;target.size.z=.5
            msg=TargetArray(header=header(t),session_id='navigation-test',image_stamp=stamp(t),lidar_stamp=stamp(t),targets=[target])
            node.receive_tracks(msg)
            with patch.object(node.navigator,'plan',wraps=node.navigator.plan) as planning:
                node.tick()
            self.assertTrue(planning.called)
            geometry=planning.call_args.args[2]
            self.assertGreater(len(geometry),0)
            self.assertTrue(np.any(np.linalg.norm(geometry-[0.,0.,3.],axis=1)>2.))
            self.assertFalse(node.safety_data['planning_paused'])
        finally:node.destroy_node()

    def test_gateway_watchdog_holds_fixed_altitude_until_control_recovers(self):
        node=Gateway(self.config(),execute=False);node.phase='flying';calls=[]
        class RPC:
            def call(self,name,*args):
                calls.append((name,args))
                if name=='getMultirotorState':return dict(kinematics_estimated=dict(position=dict(x_val=0.,y_val=35.,z_val=-10.)))
                return True
        try:
            rpc=RPC();node.velocity_task(rpc,None,False);node.velocity_task(rpc,None,False)
            self.assertEqual(sum(n=='getMultirotorState' for n,_ in calls),1)
            self.assertEqual([a[2] for n,a in calls if n=='moveByVelocityZ'],[-10.,-10.])
            self.assertFalse(any(n in ('land','enableApiControl','armDisarm') for n,_ in calls))
            cmd=VelocityCommand();cmd.velocity.y=1.
            node.velocity_task(rpc,cmd,True)
            self.assertEqual(calls[-1][0],'moveByVelocity');self.assertIsNone(node.watchdog_z)
        finally:node.destroy_node()

    def test_gateway_controlled_takeoff_and_verified_touchdown(self):
        self.gateway_flight(False)

    def test_navigation_incomplete_or_stale_sweeps_hold_and_reset_readiness(self):
        node=self.navigation_node([0.,0.,5.]);node.c['navigation_lidar']['minimum_sweep_s']=.04
        try:
            node.tick();self.assertIn('complete navigation',node.pub.messages[-1].reason)
            self.assertIsNone(node.navigation_ready_since)
            node.c['navigation_lidar']['minimum_sweep_s']=0.
            node.tick();self.assertIsNotNone(node.navigation_ready_since)
            node.cloud_wall=time.perf_counter()-1.;node.tick()
            self.assertEqual(node.pub.messages[-1].reason,'LiDAR stale')
            self.assertIsNone(node.navigation_ready_since)
            self.assertFalse(node.risk_pub.messages[-1].valid)
            np.testing.assert_allclose([node.pub.messages[-1].velocity.x,node.pub.messages[-1].velocity.y,node.pub.messages[-1].velocity.z],0.)
        finally:node.destroy_node()

    def test_navigation_reuses_pose_compensated_frames_and_only_required_sweep(self):
        node=self.navigation_node([0.,0.,5.]);node.c['navigation_lidar']['minimum_sweep_s']=.04
        try:
            node.history.samples.clear();node.cloud_history.clear();node.last_cloud_ns=0
            t=node.get_clock().now().nanoseconds
            for offset in (150_000_000,100_000_000,50_000_000,0):
                at=t-offset;pose=Odometry(header=header(at));pose.pose.pose.orientation.w=1.;pose.pose.pose.position.z=3.
                node.receive_pose(pose);msg=cloud(at,[[8.,0.,0.]]);msg.header.frame_id='drone1/base_link';node.receive_cloud(msg)
            node.tick();self.assertEqual(node.safety_data['scan_frames'],2)
            self.assertAlmostEqual(node.safety_data['scan_span_s'],.05)
            from evtol_targeting.geometry import cloud_xyz
            with patch('evtol_safety.node.cloud_xyz',wraps=cloud_xyz) as decode:
                node.tick();self.assertLessEqual(decode.call_count,1)
            self.assertGreater(node.pub.messages[-1].velocity.z,0.)
        finally:node.destroy_node()

    def test_stalled_navigation_uses_temporary_goal_without_changing_mission(self):
        node=self.navigation_node([0.,20.,3.])
        try:
            node.tick()
            node.recovery.key=('waypoint',0)
            node.recovery.history.clear();node.recovery.history.append((time.perf_counter()-4.,0.))
            node.last_plan=0.
            node.tick()
            self.assertTrue(node.safety_data['recovery']['active'])
            self.assertIsNotNone(node.safety_data['recovery']['temporary_goal_enu_m'])
            self.assertEqual(node.mission.index,0)
            np.testing.assert_array_equal(node.mission.points[0],[0,20,3])
            self.assertTrue(node.risk_pub.messages[-1].valid)
        finally:node.destroy_node()

    def test_gateway_refuses_disarm_if_land_returns_above_ground(self):
        self.gateway_flight(True)

    def test_complete_navigation_snapshot_does_not_need_second_clock_bracket(self):
        node=self.navigation_node([0.,0.,5.]);node.c['navigation_lidar']['minimum_sweep_s']=0.
        try:
            # A complete scan must remain usable if two captures happen only
            # 3 ms apart in simulation time after fixed-step catch-up.
            node.history.samples.clear();node.cloud_history.clear();node.last_cloud_ns=0
            t=node.get_clock().now().nanoseconds
            for offset in (3_000_000,0):
                at=t-offset;pose=Odometry(header=header(at));pose.pose.pose.orientation.w=1.;pose.pose.pose.position.z=3.
                node.receive_pose(pose);msg=cloud(at,[[8.,0.,0.]]);msg.header.frame_id='drone1/base_link';node.receive_cloud(msg)
            node.tick()
            self.assertEqual(node.safety_data['scan_frames'],1)
            self.assertTrue(node.risk_pub.messages[-1].valid)
            self.assertGreater(node.pub.messages[-1].velocity.z,0.)
        finally:node.destroy_node()

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
