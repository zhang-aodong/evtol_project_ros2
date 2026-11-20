import copy
import json
from pathlib import Path
import tempfile
import time
import unittest
import numpy as np
from evtol_safety.dynamic import Obstacle,DEFAULTS,candidate_clearance
from evtol_safety.velocity import LocalVelocityAvoidance
from evtol_targeting.experiment import RangeAudit,gated_assignment
from evtol_console.configuration import defaults,artifacts
from evtol_console.experiment_config import upgrade
from evtol_console.engine import Engine
from evtol_targeting.warning_audit import WarningAudit,validate_events

ROOT=Path(__file__).resolve().parents[1]


class DynamicContracts(unittest.TestCase):
    def test_slow_takeoff_can_escape_uncertain_nearby_pedestrian(self):
        obstacle=Obstacle(1,np.array([-1.4,4.97,.4]),np.zeros(3),2.4,.33,.7)
        planner=LocalVelocityAvoidance(dict(detection_radius_m=12,clearance_m=.8,max_speed_mps=2,
            max_acceleration_mps2=1,avoidance_acceleration_mps2=3))
        self.assertLess(candidate_clearance([[0,0,.3]],[0,0,0],[obstacle],3,DEFAULTS,.25)[0],0.)
        decision=planner.command([], [0,0,.3],[0,0,0],.05,obstacles=[obstacle],
            dynamic_options=DEFAULTS,minimum_vertical_velocity_mps=0.)
        self.assertFalse(decision.emergency_stop)
        self.assertTrue(decision.dynamic_escape)
        self.assertGreater(np.linalg.norm(decision.velocity_enu_mps),.3)
        self.assertLessEqual(np.linalg.norm(decision.velocity_enu_mps),2.+1e-9)
        self.assertGreaterEqual(decision.velocity_enu_mps[2],-1e-9)
        self.assertGreaterEqual(candidate_clearance([decision.velocity_enu_mps],[0,0,0],
            [obstacle],3,DEFAULTS,.25)[0],0.)

    def test_takeoff_vertical_bound_applies_after_smoothing_and_emergency_escape(self):
        p=LocalVelocityAvoidance(dict(detection_radius_m=12,clearance_m=.8,max_speed_mps=2))
        free=p.command([], [0,0,-1],[0,0,0],.1)
        self.assertLess(free.velocity_enu_mps[2],0.)
        p.reset()
        constrained=p.command([], [0,0,-1],[0,0,0],.1,minimum_vertical_velocity_mps=0.)
        self.assertGreaterEqual(constrained.velocity_enu_mps[2],-1e-9)
        incoming=Obstacle(2,np.array([0.,0.,3.]),np.array([0.,0.,-2.]),.7)
        p.reset()
        constrained=p.command([], [0,0,0],[0,0,0],.02,obstacles=[incoming],minimum_vertical_velocity_mps=0.)
        self.assertGreaterEqual(constrained.velocity_enu_mps[2],-1e-9)

    def test_async_planner_preserves_real_blocked_reason(self):
        import queue
        from types import SimpleNamespace
        from evtol_safety.async_navigation import AsyncNavigator
        p=AsyncNavigator({});p.process=SimpleNamespace(is_alive=lambda:True)
        p.requests=queue.Queue(maxsize=1);p.results=queue.Queue(maxsize=1)
        p.version=1;p.goal=np.array([0.,0.,1.5]);p.busy=True
        p.results.put((1,p.goal,[],'goal_obstructed',time.monotonic()))
        self.assertIsNone(p.plan(np.zeros(3),p.goal,np.empty((0,3))))
        self.assertEqual(p.reason,'goal_obstructed')

    def test_crossing_future_obstacle_rejects_forward_and_accepts_detour(self):
        o=Obstacle(1,np.array([3.,3.,0.]),np.array([0.,-2.,0.]),.8)
        gap=candidate_clearance([[2,0,0],[0,0,0],[0,-2,0]],[2,0,0],[o],3.,DEFAULTS)
        self.assertLess(gap[0],0);self.assertGreater(gap[1],0)
        planner=LocalVelocityAvoidance(dict(detection_radius_m=12,clearance_m=.8,max_speed_mps=2,
            max_acceleration_mps2=3,avoidance_acceleration_mps2=3))
        decision=planner.command(np.empty((0,3)),[2,0,0],[2,0,0],.2,obstacles=[o],dynamic_options=DEFAULTS)
        self.assertTrue(decision.avoidance_active)
        self.assertGreaterEqual(candidate_clearance([decision.velocity_enu_mps],[2,0,0],[o],3,DEFAULTS,.25)[0],0)
    def test_continuous_interval_catches_fast_crossing_between_sample_times(self):
        o=Obstacle(1,np.array([0.,-10.,0.]),np.array([0.,200.,0.]),.1)
        self.assertLess(candidate_clearance([[0,0,0]],[0,0,0],[o],3.,dict(DEFAULTS,horizon_s=.1,step_s=.1))[0],0)
    def test_multiple_constraints_and_no_safe_stop_are_explicit(self):
        obstacles=[Obstacle(1,np.array([0.,0.,0.]),np.zeros(3),2.)]
        p=LocalVelocityAvoidance(dict(detection_radius_m=12,clearance_m=1,max_speed_mps=2))
        d=p.command([], [2,0,0],[0,0,0],.02,obstacles=obstacles)
        self.assertTrue(d.emergency_stop);np.testing.assert_allclose(d.velocity_enu_mps,0)
    def test_response_delay_changes_safety(self):
        o=Obstacle(1,np.array([2.,0.,0.]),np.zeros(3),.5)
        fast=candidate_clearance([[0,0,0]],[2,0,0],[o],3.,DEFAULTS,0)[0]
        delayed=candidate_clearance([[0,0,0]],[2,0,0],[o],3.,DEFAULTS,1.)[0]
        self.assertGreater(fast,0);self.assertLess(delayed,0)


class AuditContracts(unittest.TestCase):
    def audit(self,n=2,**options):
        a=RangeAudit(list(range(1,n+1)),options)
        for t in np.arange(0,2.01,.05):
            a.add_ego(t,[0,0,0])
            for i in a.expected:a.add_truth(i,t,[i*5,0,0])
        return a
    def test_legal_matching_maximizes_cardinality_before_distance(self):
        # Legal distances [[.1,2],[2,3.9]], diagonal includes an illegal edge.
        d=[dict(position=[0,0,0]),dict(position=[-1.9,0,0])]
        t=[dict(position=[.1,0,0]),dict(position=[2,0,0])]
        self.assertEqual(len(gated_assignment(d,t,2.)),2)
    def test_truth_counts_even_without_any_algorithm_output(self):
        a=self.audit();m=a.snapshot(.1)
        self.assertEqual((m['truth_count'],m['detected_count'],m['missed_count']),(2,0,2));self.assertEqual(m['recall'],0)
        a.snapshot(.7);self.assertEqual(a.completed['missed'],2)
    def test_duplicates_false_positive_and_evidence_expiry(self):
        a=self.audit();row=a.measure(.1,.2,[dict(track_id=i,class_id=6,position=[5,0,0]) for i in (1,2)])
        self.assertEqual(len(row['matches']),1)
        m=a.snapshot(.1);self.assertEqual(m['detected_count'],1);self.assertEqual(m['false_positive_count'],1)
        self.assertEqual(m['recall'],.5);self.assertEqual(m['precision'],.5)
        self.assertEqual(a.snapshot(.5)['detected_count'],0)
    def test_no_truth_does_not_become_zero_denominator_success(self):
        a=RangeAudit([1]);a.add_ego(0,[0,0,0]);m=a.snapshot(0)
        self.assertFalse(m['valid']);self.assertIsNone(m['recall']);self.assertEqual(m['missing'],[1])
        b=self.audit(0);m=b.snapshot(.1);self.assertTrue(m['valid']);self.assertIsNone(m['recall'])
    def test_association_switch_cannot_double_count_a_single_track(self):
        a=self.audit()
        a.measure(.1,.2,[dict(track_id=1,class_id=6,position=[5,0,0])])
        a.measure(.2,.3,[dict(track_id=1,class_id=6,position=[10,0,0])])
        self.assertEqual(a.snapshot(.2)['detected_count'],1)
    def test_late_output_never_backfills_score(self):
        a=self.audit(1);a.measure(.1,.8,[dict(track_id=1,class_id=6,position=[5,0,0])])
        self.assertEqual(a.snapshot(.1)['detected_count'],0)
    def test_pending_timely_and_late_events_use_availability_time(self):
        a=self.audit(1);a.snapshot(0);a.measure(.1,.2,[dict(track_id=1,class_id=6,position=[5,0,0])]);m=a.snapshot(.2)
        self.assertEqual(m['encounters']['success'],1)
        b=self.audit(1);b.snapshot(0);b.measure(.4,.7,[dict(track_id=1,class_id=6,position=[5,0,0])]);m=b.snapshot(.4)
        self.assertEqual(m['encounters']['late'],1)


class ConfigRoundtrip(unittest.TestCase):
    def test_v1_migration_and_full_configuration_roundtrip(self):
        d=defaults(ROOT);self.assertEqual(d['version'],2)
        restored=artifacts(json.loads(json.dumps(d)),ROOT)[0];self.assertEqual(d,restored)
        old=copy.deepcopy(d);old['version']=1
        for k in ('experiment','dynamic','sensors','ui'):old.pop(k)
        self.assertEqual(upgrade(old)['version'],2)
        _,c,scenario,ue,_=artifacts(d,ROOT)
        self.assertEqual(c['model']['imgsz'],960);self.assertEqual(ue['Vehicles']['Drone1']['Cameras']['front_camera']['CaptureSettings'][0]['FOV_Degrees'],70)
    def test_32_and_64_aircraft_files_roundtrip_without_ego_sensor_duplication(self):
        import sys
        sys.path.insert(0,str(ROOT/'scripts'))
        from generate_experiment import generate
        with tempfile.TemporaryDirectory() as tmp:
            for count in (32,64):
                p=generate(ROOT,count,Path(tmp)/(str(count)+'.json'))
                d=json.loads(p.read_text(encoding='utf8'));_,_,scenario,ue,_=artifacts(d,ROOT)
                self.assertEqual(len(ue['Vehicles']),count+1)
                self.assertEqual(len(scenario['interference_drones']['agents']),count)
                self.assertFalse(d['run']['execute'])
                self.assertTrue(all('Sensors' not in v for name,v in ue['Vehicles'].items() if name!='Drone1'))
    def test_unknown_keys_and_invalid_budgets_rejected(self):
        for key,value in [('radius_m',float('nan')),('truth_hz',1000),('evidence_s',10)]:
            d=defaults(ROOT);d['experiment'][key]=value
            with self.assertRaises(ValueError):artifacts(d,ROOT)
        d=defaults(ROOT);d['experiment']['typo']=1
        with self.assertRaises(ValueError):artifacts(d,ROOT)
    def test_import_is_draft_only_and_does_not_start_or_save(self):
        with tempfile.TemporaryDirectory() as tmp:
            import shutil
            root=Path(tmp)
            for name in ['config/targeting.yaml','config/actors.yaml','config/machine.local.json','src/evtol_simulation/config/airsim.navigation_2k.json']:
                target=root/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ROOT/name,target)
            engine=Engine(root)
            try:
                old=copy.deepcopy(engine.config);d=copy.deepcopy(old);d['experiment']['radius_m']=15
                result=engine.request(dict(action='import_config',payload=d));self.assertTrue(result['ok'],result)
                self.assertEqual(engine.config,old);self.assertFalse(engine.processes)
                self.assertFalse((root/'config/console.local.json').exists())
                self.assertEqual(result['data']['experiment']['radius_m'],15)
            finally:engine.close()


class WarningEventContracts(unittest.TestCase):
    def events(self):
        return [dict(id='risk',truth_id=1,start_s=0.,end_s=2.,deadline_s=1.,risk=True),
                dict(id='safe',truth_id=2,start_s=0.,end_s=2.,deadline_s=1.,risk=False)]
    def test_frozen_labels_produce_precision_recall_without_circular_labels(self):
        a=WarningAudit(self.events())
        a.observe(.5,{11:1,22:2},[dict(track_id=11,warning=True),dict(track_id=22,warning=True)],1)
        m=a.observe(3,{},[],2)
        self.assertEqual(m['counts']['tp'],1);self.assertEqual(m['counts']['fp'],1)
        self.assertEqual(m['precision'],.5);self.assertEqual(m['recall'],1.)
    def test_late_warning_is_miss_and_missing_truth_is_unscorable(self):
        a=WarningAudit(self.events());a.observe(1.5,{11:1},[dict(track_id=11,warning=True)],1)
        m=a.observe(3,{},[],2);self.assertEqual(m['counts']['fn'],1);self.assertEqual(m['counts']['late'],1)
        b=WarningAudit(self.events());b.observe(.5,{},[],1,valid=False)
        self.assertEqual(b.observe(3,{},[],2)['counts']['unscorable'],2)


if __name__=='__main__':unittest.main()
