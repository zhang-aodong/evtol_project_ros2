"""Read-only visualization contracts; no AirSim connection or flight commands."""
import copy
from pathlib import Path
import time
import unittest
import yaml
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry,Path as NavPath
from rclpy.serialization import serialize_message,deserialize_message
from evtol_perception_interfaces.msg import ReferenceOrigin
from evtol_targeting.common import LATCHED,header,ns
from evtol_targeting.configuration import validate_config
from evtol_targeting.trajectory_node import TrajectoryNode

ROOT=Path(__file__).resolve().parents[1]


class Capture:
    def __init__(self):self.messages=[]
    def publish(self,msg):self.messages.append(copy.deepcopy(msg))


def origin(session='one',stamp=1_000_000_000):
    return ReferenceOrigin(header=header(stamp),session_id=session)


def pose(stamp,x=0.,y=0.,z=0.,frame='map_enu'):
    msg=Odometry(header=header(stamp,frame))
    msg.pose.pose.position.x=float(x);msg.pose.pose.position.y=float(y);msg.pose.pose.position.z=float(z)
    msg.pose.pose.orientation.w=1.
    return msg


class TrajectoryContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):rclpy.init(domain_id=226)
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()

    def config(self):return yaml.safe_load((ROOT/'config/targeting.yaml').read_text(encoding='utf-8-sig'))

    def node(self):
        c=self.config();c['mission']['waypoints_enu_m']=[[0.,0.,3.],[0.,30.,3.],[0.,30.,0.]]
        n=TrajectoryNode(c);n.waypoint_pub=Capture();n.traveled_pub=Capture()
        self.addCleanup(n.destroy_node)
        return n

    def test_configured_waypoints_preserve_order_enu_and_altitude(self):
        n=self.node();n.receive_origin(origin())
        route=n.waypoint_pub.messages[-1]
        self.assertEqual(route.header.frame_id,'map_enu')
        self.assertEqual([(p.pose.position.x,p.pose.position.y,p.pose.position.z) for p in route.poses],
                         [(0.,0.,3.),(0.,30.,3.),(0.,30.,0.)])
        self.assertTrue(all(p.header.frame_id=='map_enu' and p.pose.orientation.w==1. for p in route.poses))
        self.assertEqual(deserialize_message(serialize_message(route),NavPath),route)

    def test_actual_path_uses_observed_positions_and_original_stamps(self):
        n=self.node()
        n.receive_pose(pose(1_000_000_000,x=99))
        self.assertIsNone(n.latest)
        n.receive_origin(origin())
        samples=[pose(1_000_000_000),pose(1_050_000_000,y=1.),pose(1_100_000_000,y=1.,z=1.)]
        for p in samples:n.receive_pose(p)
        n.publish_traveled();path=n.traveled_pub.messages[-1]
        self.assertEqual([ns(p.header.stamp) for p in path.poses],[ns(p.header.stamp) for p in samples])
        self.assertEqual(path.poses[-1].pose,samples[-1].pose.pose)
        n.receive_pose(samples[-1]);n.publish_traveled()
        self.assertEqual(len(n.traveled_pub.messages),2)  # session clear + one update
        bad=pose(1_150_000_000,x=float('nan'));n.receive_pose(bad)
        n.receive_pose(pose(1_150_000_000,x=20,frame='map'))
        self.assertEqual(n.last_ns,1_100_000_000)

    def test_sampling_keeps_start_latest_and_full_session_extent(self):
        n=self.node();n.max_points=4;n.receive_origin(origin())
        for i in range(30):n.receive_pose(pose(1_000_000_000+i*50_000_000,x=i))
        n.receive_pose(pose(2_500_000_000,x=29.001))
        n.publish_traveled();path=n.traveled_pub.messages[-1]
        self.assertLessEqual(len(path.poses),n.max_points+1)
        self.assertEqual(path.poses[0].pose.position.x,0.)
        self.assertAlmostEqual(path.poses[-1].pose.position.x,29.001)
        self.assertGreater(n.spacing,n.initial_spacing)

    def test_new_session_and_clock_reset_never_join_runs(self):
        n=self.node();n.receive_origin(origin())
        n.receive_pose(pose(2_000_000_000,x=5));n.publish_traveled()
        n.receive_origin(origin('two',3_000_000_000))
        self.assertFalse(n.points);self.assertFalse(n.traveled_pub.messages[-1].poses)
        n.receive_pose(pose(2_000_000_000,x=99));self.assertIsNone(n.latest)
        n.receive_pose(pose(3_000_000_000,x=1));n.receive_pose(pose(1_000_000_000,x=2))
        self.assertTrue(n.waiting_for_origin);self.assertFalse(n.waypoint_pub.messages[-1].poses)
        n.receive_pose(pose(4_000_000_000,x=99));self.assertIsNone(n.latest)
        n.receive_origin(origin('three'));n.receive_pose(pose(1_000_000_000,x=3));n.publish_traveled()
        self.assertEqual(len(n.traveled_pub.messages[-1].poses),1)
        self.assertEqual(n.traveled_pub.messages[-1].poses[0].pose.position.x,3.)

    def test_late_ros_subscribers_receive_both_cached_paths(self):
        c=self.config();c['namespace']='trajectory_contract'
        n=TrajectoryNode(c);self.addCleanup(n.destroy_node)
        n.receive_origin(origin());n.receive_pose(pose(1_000_000_000,z=2));n.publish_traveled()
        observer=rclpy.create_node('trajectory_late_observer');self.addCleanup(observer.destroy_node)
        results={}
        subscriptions=[observer.create_subscription(NavPath,'/trajectory_contract/navigation/'+suffix,
            lambda msg,key=suffix:results.update({key:msg}),LATCHED)
            for suffix in ('waypoint_path','traveled_path')]
        end=time.perf_counter()+4
        while len(results)<2 and time.perf_counter()<end:rclpy.spin_once(observer,timeout_sec=.05)
        self.assertEqual(set(results),{'waypoint_path','traveled_path'})
        self.assertEqual(results['traveled_path'].poses[0].pose.position.z,2.)

    def test_invalid_trajectory_budgets_are_rejected(self):
        for field,value in [('publish_hz',0),('min_distance_m',float('nan')),('max_points',3),('max_points',4.5)]:
            c=self.config();c.setdefault('trajectory',{})[field]=value
            with self.assertRaises(ValueError):validate_config(c)

    def test_ros_state_flows_through_trajectory_and_foxglove_bridge(self):
        import sys
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        sys.path.insert(0,str(ROOT/'.visualization_deps'))
        from evtol_visualization.bridge import Bridge
        c=self.config();c['namespace']='trajectory_integration'
        trajectory=TrajectoryNode(c);bridge=Bridge(c['namespace'],port=0)
        source=rclpy.create_node('trajectory_fixture_source')
        executor=SingleThreadedExecutor()
        for node in (trajectory,bridge,source):executor.add_node(node)
        origin_pub=source.create_publisher(ReferenceOrigin,'/simulation/reference_origin',LATCHED)
        state_pub=source.create_publisher(Odometry,'/trajectory_integration/state/odometry',qos_profile_sensor_data)
        origin_pub.publish(origin())
        expected={f'/trajectory_integration/navigation/{name}' for name in ('waypoint_path','traveled_path')}
        try:
            end=time.perf_counter()+6
            route=None
            while time.perf_counter()<end:
                state_pub.publish(pose(1_000_000_000,x=4,y=5,z=6))
                executor.spin_once(timeout_sec=.03)
                if expected<=bridge.path_data.keys():
                    route=deserialize_message(bridge.path_data['/trajectory_integration/navigation/traveled_path'],NavPath)
                    if route.poses:break
            self.assertTrue(expected<=bridge.path_data.keys())
            self.assertIsNotNone(route)
            self.assertTrue(route.poses)
            self.assertEqual(route.header.frame_id,'map_enu')
            self.assertEqual((route.poses[-1].pose.position.x,route.poses[-1].pose.position.y,route.poses[-1].pose.position.z),(4.,5.,6.))
        finally:
            executor.shutdown()
            for node in (trajectory,bridge,source):node.destroy_node()
