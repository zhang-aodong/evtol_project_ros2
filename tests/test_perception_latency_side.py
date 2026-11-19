"""Regression cases for pre-reorder RGB and side-crossing LiDAR motion."""
import copy
import unittest
from types import SimpleNamespace
import numpy as np
from evtol_simulation.backend import AirSimBackend
from evtol_simulation.sensor_node import SensorNode
from evtol_targeting.clustering import extract_visible_targets
from evtol_targeting.projection import ProjectionResult
from evtol_targeting.lidar_obstacles import LidarMotionTracker
from evtol_safety.dynamic import Obstacle,DEFAULTS,candidate_clearance
from evtol_safety.velocity import LocalVelocityAvoidance


class RealtimeCamera(unittest.TestCase):
    def test_control_rgb_precedes_reordering_without_changing_archival_stamps(self):
        b=AirSimBackend.__new__(AirSimBackend);b.kind='camera';b.camera_readers=[SimpleNamespace(read_latest=lambda:SimpleNamespace(timestamp_ns=b.latest_ns,bgr_bytes=bytes(12)))]
        b.c={'camera':{'reorder_window_s':.2,'height':2,'width':2},'source':{'semantic_segmentation':{'vertical_flip':False}}};b.camera_pending=[]
        b.camera_latest_ns=b.camera_emitted_ns=0;received=[]
        b.realtime_camera_callback=lambda t,images:received.append(t)
        times=iter([1_000_000_000,1_100_000_000,1_050_000_000,1_250_000_000])
        def capture():
            b.last_frame_id=getattr(b,'last_frame_id',0)+1
            b.latest_ns=next(times)
            return b.latest_ns,('rgb','semantic')
        b._capture=capture
        self.assertIsNone(b.capture());self.assertEqual(received,[1_000_000_000])
        archival=[b.capture() for _ in range(3)]
        self.assertEqual([v[0] for v in archival if v],[1_000_000_000])
        self.assertEqual(received,[1_000_000_000,1_100_000_000,1_250_000_000])
    def test_realtime_output_is_small_and_skips_regressed_frames(self):
        output=[]
        node=SimpleNamespace(c={'camera':{'realtime_width':960}},frames={'optical':'camera_optical'},
                             publish=lambda topic,msg:output.append((topic,msg)))
        images=(np.zeros((1080,1920,3),np.uint8),None)
        for t in [100,90,100,110]:SensorNode.emit_realtime_camera(node,t,images)
        self.assertEqual(len(output),2)
        self.assertEqual(output[0][0],'camera/rgb/image_realtime')
        self.assertEqual((output[0][1].width,output[0][1].height),(960,540))
        self.assertEqual([m.header.stamp.nanosec for _,m in output],[100,110])


class SemanticExtent(unittest.TestCase):
    def test_aircraft_extent_gate_preserves_small_aircraft_and_other_classes(self):
        pixels=np.array([[x,4] for x in range(25)],np.int32)
        projection=ProjectionResult(np.arange(25),pixels,np.full(25,5.))
        labels=np.full((8,30),6,np.int32);confidence=np.ones_like(labels,dtype=float)
        points=np.column_stack([np.full(25,5.),np.linspace(-3,3,25),np.zeros(25)])
        diagnostics={}
        targets=extract_visible_targets(points,projection,labels,confidence,[6],
            max_extent_by_class={6:4.},diagnostics=diagnostics)
        self.assertEqual(targets,[]);self.assertEqual(diagnostics['extent_rejected_clusters'],1)
        points[:,1]*=.2
        self.assertEqual(len(extract_visible_targets(points,projection,labels,confidence,[6],max_extent_by_class={6:4.})),1)
        labels[:]=3;points[:,1]*=5
        self.assertEqual(len(extract_visible_targets(points,projection,labels,confidence,[3],max_extent_by_class={6:4.})),1)


class SideMotion(unittest.TestCase):
    def test_side_target_velocity_is_measured_without_camera_or_truth(self):
        tracker=LidarMotionTracker();offsets=np.array([[x,y,z] for x in [-.2,0,.2] for y in [-.2,0,.2] for z in [-.1,.1]])
        for i in range(20):
            t=i*.1;tracker.update(offsets+[0,8-2*t,3],1_000_000_000+int(t*1e9),[0,0,3])
        self.assertEqual(len(tracker.tracker.tracks),1)
        track=next(iter(tracker.tracker.tracks.values()))
        self.assertEqual(track['class_id'],0)
        np.testing.assert_allclose(track['x'][3:],[0,-2,0],atol=.25)
        self.assertGreaterEqual(track['hits'],15)
        self.assertIn(track['id'],tracker.motion)
        np.testing.assert_allclose(tracker.motion[track['id']][0],[0,-2,0],atol=.1)
    def test_stationary_fragments_do_not_acquire_uncertain_dynamic_envelopes(self):
        tracker=LidarMotionTracker();offsets=np.array([[x,y,z] for x in [-.2,.2] for y in [-.2,.2] for z in [-.1,.1]])
        for i in range(12):
            jitter=np.array([.08*(-1)**i,.04*(-1)**(i//2),0.])
            tracker.update(offsets+[0,5,3]+jitter,1_000_000_000+i*100_000_000,[0,0,3])
        self.assertGreater(len(tracker.tracker.tracks),0)
        self.assertEqual(tracker.motion,{})
    def test_large_wall_does_not_become_small_aircraft_and_repeats_are_not_reused(self):
        tracker=LidarMotionTracker()
        wall=np.array([[5,y,z] for y in np.arange(-4,4,.3) for z in np.arange(1,5,.3)])
        tracker.update(wall,1_000_000_000,[0,0,3])
        self.assertEqual(len(tracker.tracker.tracks),0)
        self.assertGreater(tracker.stats['large_clusters'],0)
        self.assertFalse(tracker.update(wall,1_000_000_000,[0,0,3]))
    def test_ground_fragments_do_not_exhaust_airborne_track_budget(self):
        tracker=LidarMotionTracker()
        ground=np.array([[x,y,0.] for x in np.arange(-20,20,1.2) for y in np.arange(-20,20,1.2)])
        tracker.update(ground,1_000_000_000,[0,0,3])
        self.assertEqual(len(tracker.tracker.tracks),0)
    def test_hovering_ego_evades_side_collision_when_safe_escape_exists(self):
        obstacle=Obstacle(1,np.array([0.,5.,0.]),np.array([0.,-2.,0.]),1.)
        options=dict(DEFAULTS,sigma_scale=1.)
        self.assertLess(candidate_clearance([[0,0,0]],[0,0,0],[obstacle],3.,options,.2)[0],0)
        self.assertGreater(candidate_clearance([[2,0,0]],[0,0,0],[obstacle],3.,options,.2)[0],0)
        planner=LocalVelocityAvoidance(dict(detection_radius_m=12,clearance_m=.8,max_speed_mps=2,
            max_acceleration_mps2=1,avoidance_acceleration_mps2=3))
        decision=planner.command([], [0,0,0],[0,0,0],.1,obstacles=[obstacle],dynamic_options=options)
        self.assertFalse(decision.emergency_stop)
        self.assertGreater(np.linalg.norm(decision.velocity_enu_mps),.05)


if __name__=='__main__':unittest.main()
