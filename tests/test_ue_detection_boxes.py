"""Detection overlay geometry, freshness and read-only RPC contracts."""
import copy
from pathlib import Path
import time
import unittest
import numpy as np
import rclpy
import yaml
from rosgraph_msgs.msg import Clock
from evtol_perception_interfaces.msg import ReferenceOrigin, Target, TargetArray
from evtol_targeting.common import header, stamp, put
from evtol_targeting.ue_box_geometry import box_edges_ned, to_global_ned, options
from evtol_targeting.ue_detection_boxes import UeDetectionBoxes

ROOT=Path(__file__).resolve().parents[1]
SECOND=1_000_000_000


class Rpc:
    def __init__(self, moving=False, invalid=False):
        self.calls=[];self.moving=moving;self.invalid=invalid;self.samples=0
    def call(self, method, *args):
        self.calls.append((method,args))
        if method=='simGetVehiclePose':
            self.samples+=1
            p=[self.samples if self.moving else 1.,2.,-.2]
        elif method=='simGetObjectPose':
            p=[float('nan'),0.,0.] if self.invalid else [11.,22.,.1]
        else:return
        return {'position':dict(zip(('x_val','y_val','z_val'),p))}


def targets(t=SECOND,session='one',class_id=6):
    target=Target(track_id=3,class_id=class_id,state='confirmed',last_measurement_stamp=stamp(t))
    put(target.position,[4.,5.,6.]);put(target.size,[2.,3.,1.])
    return TargetArray(header=header(t),session_id=session,targets=[target])


class Geometry(unittest.TestCase):
    def test_nonzero_origin_and_local_global_offset(self):
        np.testing.assert_allclose(to_global_ned([1,2,3],[10,20,30],[100,200,-5]),[122,211,-38])
        edges=np.asarray(box_edges_ned([1,2,3],[2,4,6],[10,20,30],[100,200,-5]))
        self.assertEqual(edges.shape,(24,3))
        self.assertEqual(len(set(map(tuple,edges))),8)
        np.testing.assert_allclose(edges.min(axis=0),[120,210,-41])
        np.testing.assert_allclose(edges.max(axis=0),[124,212,-35])
        self.assertTrue(all(np.count_nonzero(b-a)==1 for a,b in edges.reshape(-1,2,3)))
        self.assertEqual(len({tuple(sorted((tuple(a),tuple(b)))) for a,b in edges.reshape(-1,2,3)}),12)

    def test_invalid_options_fail_before_launch(self):
        for key,value in [('rate_hz',0),('line_lifetime_s',2),('color_rgba',[1,0,0,0]),
                          ('padding_m',-1),('max_boxes',0),('global_ned_offset_m',[1,2]),
                          ('class_ids',[6.5]),('min_size_enu_m',[1,0,1])]:
            with self.subTest(key=key),self.assertRaises(ValueError):options({'ue_detection_boxes':{key:value}})


class Overlay(unittest.TestCase):
    @classmethod
    def setUpClass(cls):rclpy.init(domain_id=225)
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()
    def node(self,**options):
        c=yaml.safe_load((ROOT/'config/targeting.yaml').read_text(encoding='utf-8-sig'))
        c['ue_detection_boxes']=options
        n=UeDetectionBoxes(c,start_worker=False);self.addCleanup(n.destroy_node)
        n.receive_origin(ReferenceOrigin(header=header(SECOND),session_id='one'))
        n.receive_clock(Clock(clock=stamp(SECOND)))
        return n

    def test_batches_twelve_edges_per_detection_and_no_control_or_target_truth(self):
        n=self.node();msg=targets();msg.targets.append(copy.deepcopy(msg.targets[0]));msg.targets[-1].track_id=4
        n.receive_targets(msg);rpc=Rpc();n.render_once(rpc)
        self.assertEqual([m for m,a in rpc.calls],['simGetVehiclePose','simGetObjectPose','simGetVehiclePose','simPlotLineList','simPlotStrings'])
        self.assertEqual([a[0] for m,a in rpc.calls[:3]],['Drone1']*3)
        np.testing.assert_allclose(n.spawn,[10,20,.3])
        lines=rpc.calls[3][1];self.assertEqual(len(lines[0]),48);self.assertFalse(lines[-1])
        self.assertEqual(n.last_box_count,2)
        n.render_once(rpc)
        self.assertEqual(len([1 for m,a in rpc.calls if m=='simGetObjectPose']),1)

    def test_minimum_box_class_filter_and_limit(self):
        n=self.node(max_boxes=1,show_labels=False);msg=targets(class_id=3)
        n.receive_targets(msg);r=Rpc();n.render_once(r);self.assertFalse(r.calls)
        msg=targets(t=SECOND+1);put(msg.targets[0].size,[0,0,0]);msg.targets*=3
        n.receive_clock(Clock(clock=stamp(SECOND+1)));n.receive_targets(msg);n.render_once(r)
        self.assertEqual(n.last_box_count,1)
        np.testing.assert_allclose(n.boxes[0].size,[1.3,1.3,.8])
        self.assertNotIn('simPlotStrings',[m for m,a in r.calls])

    def test_reject_wrong_frame_session_nonfinite_and_measurement_time(self):
        n=self.node()
        for field,value in [('header.frame_id','map'),('session_id','other'),
                            ('targets.0.position.x',float('nan')),
                            ('targets.0.last_measurement_stamp',stamp(SECOND+1))]:
            msg=targets();obj=msg;parts=field.split('.')
            for part in parts[:-1]:obj=obj[int(part)] if part.isdigit() else getattr(obj,part)
            setattr(obj,parts[-1],value);n.receive_targets(msg)
            self.assertFalse(n.boxes)
        msg=targets(t=SECOND+1);n.receive_targets(msg)
        empty=targets(t=SECOND);empty.targets=[];n.receive_targets(empty)
        self.assertEqual(len(n.boxes),1)

    def test_empty_wall_timeout_and_paused_clock_stop_rendering(self):
        n=self.node();n.receive_targets(targets());r=Rpc();n.render_once(r)
        calls=len(r.calls)
        n.render_once(r,wall=time.perf_counter()+2);self.assertEqual(len(r.calls),calls)
        msg=targets(t=SECOND+1);msg.targets=[];n.receive_targets(msg);n.render_once(r)
        self.assertEqual(len(r.calls),calls)
        n.receive_targets(targets(t=SECOND+2));n.clock_wall=time.perf_counter()-2
        n.receive_clock(Clock(clock=stamp(SECOND)));n.render_once(r)
        self.assertEqual(len(r.calls),calls)

    def test_sim_age_and_old_measurements_stop_rendering(self):
        n=self.node(measurement_timeout_s=.1)
        n.receive_targets(targets());n.receive_clock(Clock(clock=stamp(SECOND+200_000_000)))
        r=Rpc();n.render_once(r);self.assertFalse(r.calls)
        n.receive_clock(Clock(clock=stamp(SECOND+2*SECOND)));n.render_once(r);self.assertFalse(r.calls)

    def test_session_and_clock_reset_clear_and_recalibrate(self):
        n=self.node();n.receive_targets(targets());n.render_once(Rpc())
        n.receive_origin(ReferenceOrigin(header=header(SECOND+1),session_id='two'))
        self.assertFalse(n.boxes);self.assertIsNone(n.spawn)
        n.receive_targets(targets(t=SECOND+1,session='two'))
        n.receive_clock(Clock(clock=stamp(SECOND-1)))
        self.assertFalse(n.boxes);self.assertIsNone(n.origin)

    def test_bad_calibration_never_draws_and_retry_can_recover(self):
        n=self.node();n.receive_targets(targets())
        for r in (Rpc(moving=True),Rpc(invalid=True)):
            with self.assertRaises(ValueError):n.render_once(r)
            self.assertNotIn('simPlotLineList',[m for m,a in r.calls])
        n.render_once(Rpc());self.assertEqual(n.draw_calls,1)

    def test_explicit_offset_and_global_source_need_no_ego_lookup(self):
        for config in ({'global_ned_offset_m':[10,20,.3]}, {'source_coordinates':'global_ned'}):
            n=self.node(**config);n.receive_targets(targets());r=Rpc();n.render_once(r)
            self.assertEqual([m for m,a in r.calls],['simPlotLineList','simPlotStrings'])

