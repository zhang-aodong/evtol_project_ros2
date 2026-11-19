"""Geometry, real clustering/tracking and configurable actor path regressions."""
from pathlib import Path
import sys
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
for package in ('evtol_targeting','evtol_safety','evtol_scenario'):
    sys.path.insert(0,str(ROOT/'src'/package))
from evtol_targeting.geometry import PoseHistory,compensated_projection,cloud_xyz
from evtol_targeting.projection import ProjectionResult
from evtol_targeting.clustering import extract_visible_targets
from evtol_targeting.tracking import MultiTargetTracker
from evtol_safety.velocity import LocalVelocityAvoidance
from evtol_safety.node import closest_approach
from evtol_scenario.paths import validate,vehicle_path,pedestrian_path,advance


class TargetingTests(unittest.TestCase):
    def test_pose_history_interpolates_rotation_and_never_extrapolates(self):
        h=PoseHistory(max_gap_s=.2)
        h.add(1_000_000_000,[0,0,0],[0,0,0,1])
        h.add(1_100_000_000,[2,0,0],[0,0,1,0])
        p,r=h.at(1_050_000_000)
        np.testing.assert_allclose(p,[1,0,0]);np.testing.assert_allclose(r@[1,0,0],[0,1,0],atol=1e-8)
        self.assertIsNone(h.at(1_200_000_000));self.assertIsNone(h.at(999_000_000))
        h.add(500_000_000,[0,0,0],[0,0,0,1]);self.assertEqual(len(h.samples),1)

    def test_ego_motion_compensated_projection_with_optical_axes(self):
        identity=([0,0,0],np.eye(3));camera=([0,1,0],np.eye(3))
        body=([0,0,0],[0,0,0,1]);optical=([0,0,0],[-.5,.5,-.5,.5])
        k=np.array([[100,0,100],[0,100,100],[0,0,1]])
        world,projection=compensated_projection(np.array([[10.,0,0]]),identity,camera,body,optical,k,200,200)
        np.testing.assert_allclose(world,[[10,0,0]])
        np.testing.assert_array_equal(projection.pixels_uv,[[110,100]])

    def test_depth_occlusion_and_two_same_class_clusters(self):
        points=np.array([[0,0,0],[.1,0,0],[0,.1,0],[4,0,0],[4.1,0,0],[4,.1,0],[20,0,0]])
        projection=ProjectionResult(np.arange(7),np.array([[1,1],[2,1],[1,2],[6,1],[7,1],[6,2],[1,1]]),np.array([10,10,10,10,10,10,30]))
        labels=np.full((10,10),6,np.int32)
        found=extract_visible_targets(points,projection,labels,np.full((10,10),np.nan),[6],min_lidar_points=3,min_mask_pixels=1,cluster_radius_m=.5)
        self.assertEqual(len(found),2)
        self.assertTrue(all(6 not in t.point_indices for t in found))

    def test_no_lidar_never_invents_3d_target(self):
        found=extract_visible_targets(np.empty((0,3)),ProjectionResult(np.array([],int),np.empty((0,2),int),np.array([])),
            np.full((10,10),6),np.full((10,10),np.nan),[6],min_mask_pixels=1)
        self.assertEqual(found,[])

    def test_tracker_ordering_duplicate_scan_and_class_gate(self):
        tracker=MultiTargetTracker(max_age_s=.5)
        first,_=tracker.update([dict(class_id=6,position_enu_m=[0.,0,0]),dict(class_id=6,position_enu_m=[5.,0,0])],1_000_000_000)
        next_,_=tracker.update([dict(class_id=6,position_enu_m=[5.1,0,0]),dict(class_id=6,position_enu_m=[.1,0,0])],1_100_000_000)
        self.assertEqual(first[0]['track_id'],next_[1]['track_id'])
        hits=tracker.tracks[1]['hits'];tracker.update([dict(class_id=6,position_enu_m=[.1,0,0])],1_100_000_000)
        self.assertEqual(hits,tracker.tracks[1]['hits'])
        tracker.update([],2_000_000_000);self.assertEqual(tracker.tracks,{})

    def test_cloud_organized_padding_and_big_endian(self):
        from sensor_msgs.msg import PointCloud2,PointField
        import struct
        fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z'))]
        payload=struct.pack('>3f',1,2,3)+bytes(4)+struct.pack('>3f',4,5,6)+bytes(4)
        msg=PointCloud2(height=2,width=1,fields=fields,point_step=12,row_step=16,is_bigendian=True,data=payload)
        np.testing.assert_allclose(cloud_xyz(msg),[[1,2,3],[4,5,6]])

    def test_stopping_budget_respects_sensor_range(self):
        planner=LocalVelocityAvoidance(dict(detection_radius_m=5.,clearance_m=1.,max_speed_mps=7.,braking_acceleration_mps2=3.),sensor_timeout_s=.2)
        self.assertLess(planner.speed_limit,7.)
        decision=planner.command(np.array([[.2,0,0]]),[2,0,0],[2,0,0],.02)
        self.assertLess(np.linalg.norm(decision.velocity_enu_mps),2.)

    def test_cpa_crossing_and_parallel_motion(self):
        t,d=closest_approach(np.array([10.,0,0]),np.array([-5.,0,0]),3.)
        self.assertEqual(t,2.);self.assertEqual(d,0.)
        t,d=closest_approach(np.array([10.,0,0]),np.zeros(3),3.)
        self.assertEqual(t,0.);self.assertEqual(d,10.)

    def test_legacy_obstacle_spawn_offset_and_initial_enu_paths(self):
        a=dict(spawn_offset_ned_m=[30,0,0],waypoints_frd_m=[[30,0,-10]],coordinate_mode='relative_heading')
        np.testing.assert_allclose(vehicle_path(a,np.zeros(3),np.eye(3),np.zeros(3)),[[0,0,-10]])
        a.update(coordinate_mode='initial_enu',waypoints_frd_m=[[0,30,10]])
        np.testing.assert_allclose(vehicle_path(a,np.zeros(3),np.eye(3),np.zeros(3)),[[0,0,-10]])

    def test_pedestrian_initial_offset_and_no_overshoot(self):
        a=dict(coordinate_mode='relative_initial',waypoints=[[8,0,0],[0,0,0]])
        path=pedestrian_path(a,np.array([3,4,5]),np.zeros(3))
        np.testing.assert_allclose(path,[[11,4,5],[3,4,5]])
        p,reached=advance(np.zeros(3),np.ones(3),10.)
        self.assertTrue(reached);np.testing.assert_allclose(p,np.ones(3))

    def test_actor_config_rejects_ego_control(self):
        settings=dict(interference_drones=dict(agents=[dict(vehicle_name='Drone1')]),pedestrians=dict(agents=[],update_rate_hz=30))
        with self.assertRaises(ValueError):validate(settings,'Drone1')


if __name__=='__main__':unittest.main()
