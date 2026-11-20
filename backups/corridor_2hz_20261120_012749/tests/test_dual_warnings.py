import copy
import json
from pathlib import Path
import unittest
import numpy as np
import yaml
from evtol_safety.warnings import proximity,predict_target,in_hemisphere,DEFAULTS
from evtol_safety.navigation import HorizontalFirstNavigator
from evtol_targeting.experiment import RangeAudit
from evtol_targeting.surround_geometry import optical_pose,intrinsics
from evtol_targeting.geometry import compensated_projection
from evtol_targeting.onnx_detector import decode,letterbox
from evtol_console.experiment_config import upgrade
from evtol_console.configuration import artifacts

ROOT=Path(__file__).resolve().parents[1]


class DualGeometry(unittest.TestCase):
    def test_two_views_cover_horizontal_100_and_keep_side_vertical_blind_spots(self):
        s=yaml.safe_load((ROOT/'config/targeting.yaml').read_text(encoding='utf8'))['surround']
        self.assertEqual(len(s['cameras']),2)
        angles=np.radians(np.linspace(-90,90,361))
        pts=np.vstack((np.column_stack((10*np.cos(angles),10*np.sin(angles),np.zeros(len(angles)))),[0,0,10],[0,0,-10],[-10,0,0]))
        pose=(np.zeros(3),np.eye(3));seen=set()
        for v in s['cameras']:
            _,projection=compensated_projection(pts,pose,pose,(np.zeros(3),[0,0,0,1]),optical_pose(v),intrinsics(s),s['width'],s['height'])
            seen.update(projection.point_indices.tolist())
        covered=np.flatnonzero(abs(np.degrees(angles))<49.9)
        outside=np.flatnonzero(abs(np.degrees(angles))>50.1)
        self.assertTrue(set(covered).issubset(seen))
        self.assertFalse(set(outside)&seen)
        self.assertFalse(any(i in seen for i in (361,362,363)))

    def test_letterbox_recovers_original_coordinates_and_drops_invalid_rows(self):
        im=np.zeros((100,200,3),np.uint8);_,transform=letterbox(im,400)
        output=np.array([[[20,120,100,180,.8,0],[0,0,0,0,.9,0],[20,20,30,30,.1,0]]])
        boxes=decode(output,[transform],.25,128)
        self.assertEqual(len(boxes),1);np.testing.assert_allclose(boxes[0]['xyxy'],[10,10,50,40])
        with self.assertRaises(ValueError):decode(np.zeros((1,6,300)),[transform],.25,128)


class WarningGeometry(unittest.TestCase):
    def test_any_single_point_triggers_only_above_five_meters_agl(self):
        for agl in (None,0,4.99,5.):self.assertFalse(proximity([[0,4.9,0]],agl)['warning'])
        self.assertTrue(proximity([[0,4.9,0]],5.01)['warning'])
        self.assertTrue(proximity([[0,0,-5]],6)['warning'])
        self.assertFalse(proximity([[0,0,0],[6,0,0],[float('nan'),0,0]],6)['warning'])

    def test_forward_hemisphere_includes_top_bottom_boundary_and_turns_with_ego(self):
        for p in ([0,0,10],[0,0,-10],[0,10,0],[10,0,0]):self.assertTrue(in_hemisphere(p,[1,0,0],10))
        self.assertFalse(in_hemisphere([-1,0,0],[1,0,0],10))
        self.assertFalse(in_hemisphere([10.01,0,0],[1,0,0],10))
        self.assertTrue(in_hemisphere([-1,0,0],[-1,0,0],10))

    def test_prediction_does_not_expand_current_warning_range(self):
        d=predict_target([15,0,0],[-3,0,0],np.zeros(3),np.zeros(3),[1,0,0])
        self.assertFalse(d['warning']);self.assertEqual(d['predicted_entry_s'],2.)
        self.assertAlmostEqual(d['closing_speed_mps'],3.)
        self.assertEqual(d['prediction'][-1]['distance_m'],6.)
        d=predict_target([8,0,0],[2,0,0],np.zeros(3),np.zeros(3),[1,0,0])
        self.assertTrue(d['warning']);self.assertLess(d['closing_speed_mps'],0)

    def audit(self):
        a=RangeAudit([1,2,3])
        for t in np.arange(0,2.01,.05):
            a.add_ego(t,[0,0,0],forward=[1,0,0])
            a.add_truth(1,t,[5,0,0]);a.add_truth(2,t,[-5,0,0]);a.add_truth(3,t,[0,0,9])
        return a

    def test_blind_spot_counts_as_failure_and_rear_target_is_excluded(self):
        a=self.audit();a.snapshot(.1)
        a.measure(.15,.2,[dict(track_id=42,position=[5,0,0])]);m=a.snapshot(.2)
        self.assertEqual(m['truth_count'],2);self.assertEqual(m['detected_count'],1)
        m=a.snapshot(.7);self.assertEqual(m['timely_success_rate'],.5)
        self.assertEqual(m['encounters']['missed'],1)

    def test_late_or_duplicate_observations_cannot_inflate_success(self):
        a=self.audit();a.snapshot(.1)
        a.measure(.15,.8,[dict(track_id=42,position=[5,0,0]),dict(track_id=43,position=[5,0,0])])
        m=a.snapshot(.7);self.assertEqual(m['detected_count'],0);self.assertEqual(m['timely_success_rate'],0.)

    def test_missing_or_zero_orientation_invalidates_scoring(self):
        a=self.audit();a.forward.clear()
        m=a.snapshot(.1)
        self.assertFalse(m['valid']);self.assertEqual(m['totals']['invalid'],1)
        a._add(a.forward,.2,[0,0,0],[0,0,0])
        m=a.snapshot(.2)
        self.assertFalse(m['valid']);self.assertEqual(m['totals']['invalid'],2)

    def test_migration_preserves_user_paths_but_switches_to_new_protocol(self):
        original=json.loads((ROOT/'config/console.local.json').read_text(encoding='utf-8-sig'))
        original['version']=2
        upgraded=upgrade(original)
        self.assertEqual(upgraded['mission'],original['mission'])
        self.assertEqual(upgraded['actors'],original['actors'])
        self.assertEqual(upgraded['experiment']['radius_m'],10)
        self.assertEqual(upgraded['experiment']['region'],'front_hemisphere')
        self.assertEqual(upgraded['sensors']['width'],1536)
        self.assertEqual(upgraded['version'],4)
        self.assertEqual(upgraded['sensors']['horizontal_fov_deg'],70)
        upgraded['sensors']['horizontal_fov_deg']=75
        self.assertEqual(upgrade(upgraded)['sensors']['horizontal_fov_deg'],75)
        _,c,_,ue,_=artifacts(upgraded,ROOT)
        self.assertEqual(set(ue['Vehicles']['Drone1']['Cameras']),{'front_left','front_right'})
        self.assertEqual(c['model']['backend'],'onnx')

    def test_horizontal_astar_routes_around_wall_without_climbing(self):
        wall=np.array([[3,y,z] for y in np.arange(-1,1.01,.2) for z in np.arange(-2,2.01,.2)])
        planner=HorizontalFirstNavigator(dict(resolution_m=.5,planning_radius_m=9,clearance_m=.6,search_budget_s=1.))
        aim=planner.plan(np.zeros(3),np.array([6.,0,0]),wall)
        self.assertIsNotNone(aim,planner.reason);self.assertTrue(planner.reason.startswith('horizontal'))
        route=np.asarray(planner.path);self.assertTrue(np.allclose(route[:,2],0))
        self.assertGreater(abs(route[:,1]).max(),1.5)


class WarningNodeContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import rclpy
        rclpy.init(domain_id=231)

    @classmethod
    def tearDownClass(cls):
        import rclpy
        rclpy.shutdown()

    def node(self):
        from evtol_safety.warning_node import WarningNode
        from evtol_perception_interfaces.msg import ReferenceOrigin,StateStatus,TargetArray,Target
        from evtol_targeting.common import header,put,stamp
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2,PointField
        c=yaml.safe_load((ROOT/'config/targeting.yaml').read_text(encoding='utf8'));c['_root']=ROOT
        node=WarningNode(c);self.addCleanup(node.destroy_node)
        node.receive_origin(ReferenceOrigin(session_id='warning-test'));node.valid_state=True
        now=node.get_clock().now().nanoseconds
        pose=Odometry(header=header(now),child_frame_id='drone1/base_link');pose.pose.pose.position.z=6;pose.pose.pose.orientation.w=1.
        node.receive_pose(pose)
        points=np.array([[0,0,-6],[.1,0,-6],[-.1,0,-6],[0,4,0]],dtype='<f4')
        node.receive_cloud(PointCloud2(header=header(now,'drone1/base_link'),height=1,width=len(points),
            fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z'))],
            point_step=12,row_step=len(points)*12,data=points.tobytes()))
        target=Target(track_id=1,class_id=6,last_measurement_stamp=stamp(now),measured=True)
        put(target.position,[8,0,6]);put(target.velocity,[-2,0,0])
        node.receive_tracks(TargetArray(header=header(now),session_id='warning-test',image_stamp=stamp(now),targets=[target]))
        lidar=[];fusion=[]
        class Capture:
            def __init__(self,destination):self.destination=destination
            def publish(self,msg):self.destination.append(json.loads(msg.data))
        node.lidar_pub=Capture(lidar);node.fusion_pub=Capture(fusion)
        return node,lidar,fusion

    def test_lidar_and_fusion_publish_independent_warnings_and_predictions(self):
        node,lidar,fusion=self.node();node.tick()
        self.assertTrue(lidar[-1]['warning']);self.assertAlmostEqual(lidar[-1]['agl_m'],6.)
        self.assertTrue(fusion[-1]['warning']);self.assertEqual(fusion[-1]['targets'][0]['relative_speed_mps'],2)
        self.assertFalse(any('truth' in s.topic_name for s in node.subscriptions))

    def test_stale_lidar_does_not_hide_fresh_fusion_warning(self):
        node,lidar,fusion=self.node();node.cloud_wall-=10;node.tick()
        self.assertFalse(lidar[-1]['valid']);self.assertTrue(fusion[-1]['warning'])

    def test_stale_measurements_and_session_reset_cannot_replay_warning(self):
        from evtol_perception_interfaces.msg import ReferenceOrigin
        node,lidar,fusion=self.node();node.track_wall-=10;node.tick()
        self.assertFalse(fusion[-1]['valid']);self.assertFalse(fusion[-1]['warning'])
        node.receive_origin(ReferenceOrigin(session_id='new'));node.tick()
        self.assertFalse(lidar[-1]['valid']);self.assertFalse(fusion[-1]['valid'])


if __name__=='__main__':unittest.main()
