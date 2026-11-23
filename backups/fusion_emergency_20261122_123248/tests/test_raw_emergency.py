"""Raw point-cloud emergency protection without visual detections or truth."""
from pathlib import Path
import copy
import unittest
import numpy as np
import yaml
from evtol_safety.raw_emergency import RawLidarEmergency,options
from evtol_safety.velocity import LocalVelocityAvoidance
from evtol_safety.dynamic import DEFAULTS,candidate_clearance
from evtol_console.configuration import defaults,artifacts
from evtol_console.experiment_config import upgrade

ROOT=Path(__file__).resolve().parents[1]

class RawEmergencyContracts(unittest.TestCase):
    def guard(self,points,armed=True,**kw):
        guard=RawLidarEmergency();p=np.array([0.,0.,3.])
        return guard,guard.update(np.asarray(points)+p,p,p,1_000_000_000,0.,armed,ground_z=0.,**kw)

    def test_default_radius_and_all_directions(self):
        self.assertEqual(options()['radius_m'],2.)
        points=np.vstack((np.eye(3),-np.eye(3)))*1.8
        g,(obstacles,raw)=self.guard(points)
        self.assertEqual(len(obstacles),6);self.assertEqual(len(raw),6)
        self.assertTrue(all(o.track_id<0 and o.radius==.8 for o in obstacles))
        self.assertTrue(g.diagnostics['active'])
        _,(obstacles,_)=self.guard([[2.01,0,0]])
        self.assertFalse(obstacles)

    def test_takeoff_landing_and_disable_clear_motion_history(self):
        for armed,config in ((False,{}),(True,dict(enabled=False))):
            g=RawLidarEmergency(config)
            g.update([[0,1.8,3]],[0,0,3],[0,0,3],1_000_000_000,0.,True)
            obstacles,_=g.update([[0,1.5,3]],[0,0,3],[0,0,3],1_050_000_000,0.,armed)
            self.assertFalse(obstacles);self.assertFalse(g.cells);self.assertFalse(g.diagnostics['armed'])

    def test_ground_and_self_filter_keeps_raised_and_side_objects(self):
        g=RawLidarEmergency();p=np.array([0.,0.,1.5])
        points=[[0,0,0],[0,0,.1],[0,.1,1.5],[0,1.2,1.5],[0,0,2.8],[0,0,.5]]
        obstacles,raw=g.update(points,p,p,1,0.,True,ground_z=0.)
        self.assertEqual(len(raw),3)
        self.assertTrue(np.any(np.all(np.isclose(raw,[0,0,.5]),axis=1)))

    def test_radial_approach_compensates_ego_translation_and_replayed_stamp(self):
        g=RawLidarEmergency();p=np.array([0.,0.,3.])
        g.update([[0,1.9,3]],p,p,1_000_000_000,0.,True)
        moved=np.array([0.,.2,3.])
        obs,_=g.update([[0,1.9,3]],moved,moved,1_100_000_000,0.,True)
        np.testing.assert_allclose(obs[0].velocity,[0,0,0],atol=1e-8)
        obs,_=g.update([[0,1.7,3]],moved,moved,1_200_000_000,.05,True)
        np.testing.assert_allclose(obs[0].velocity,[0,-2,0],atol=1e-8)
        np.testing.assert_allclose(obs[0].position,[0,1.4,0],atol=1e-8)
        repeat,_=g.update([[0,1.7,3]],moved,moved,1_200_000_000,.05,True)
        np.testing.assert_allclose(repeat[0].velocity,obs[0].velocity)
        obs,_=g.update([[0,1.6,3]],moved,moved,500_000_000,0.,True)
        np.testing.assert_allclose(obs[0].velocity,[0,0,0])

    def test_nearest_return_retained_and_constraint_count_bounded(self):
        rng=np.random.default_rng(2);points=rng.normal(size=(20000,3))
        points=points/np.linalg.norm(points,axis=1)[:,None]*1.9
        points=np.vstack((points,[1.,0,0]))
        g,(obstacles,raw)=self.guard(points)
        self.assertLessEqual(len(obstacles),72)
        self.assertAlmostEqual(g.diagnostics['nearest_m'],1.)
        self.assertAlmostEqual(min(np.linalg.norm(o.position) for o in obstacles),1.)

    def test_release_requires_clearance_margin_and_fresh_scan_hold(self):
        g=RawLidarEmergency();p=np.array([0.,0.,3.])
        def sample(distance,t):
            return g.update([[0,distance,3]],p,p,int(t*1e9),0.,True)
        sample(2.1,1.);self.assertFalse(g.active)
        sample(1.9,1.1);self.assertTrue(g.active)
        sample(2.2,1.2);self.assertTrue(g.active)
        sample(3.,1.3);self.assertTrue(g.active)
        sample(3.,1.3);self.assertTrue(g.active)
        sample(3.,1.81);self.assertFalse(g.active)
        sample(2.1,1.9);self.assertFalse(g.active)

    def planner(self):
        c=yaml.safe_load((ROOT/'config/targeting.yaml').read_text(encoding='utf8'))
        return LocalVelocityAvoidance(c['safety']['local_velocity'],.2,.05,0.)

    def test_hover_moves_away_from_unclassified_side_return(self):
        g,(obstacles,points)=self.guard([[0,1.8,0]])
        planner=self.planner();desired=g.preference(np.zeros(3),obstacles,.8,planner.speed_limit)
        decision=planner.command(points-[0,0,3],desired,np.zeros(3),.05,obstacles=obstacles,best_effort_escape=True)
        self.assertLess(decision.velocity_enu_mps[1],0.)
        self.assertFalse(decision.emergency_stop)

    def test_side_intercept_without_vision_keeps_clearance(self):
        # Deterministic moving point versus accelerated ego, 20 Hz. The point
        # continues through the old hover location; protection sees it only
        # once it enters 2 m and never receives visual tracks.
        g=RawLidarEmergency();planner=self.planner();p=np.array([0.,0.,3.]);v=np.zeros(3)
        minimum=10.;triggered=False
        for i in range(160):
            target=np.array([0.,3.-.6*i*.05,3.])
            minimum=min(minimum,float(np.linalg.norm(target-p)))
            obs,points=g.update([target],p,p,1_000_000_000+i*50_000_000,0.,True,ground_z=0.)
            triggered |= bool(obs)
            desired=g.preference(np.zeros(3),obs,.8,planner.speed_limit)
            decision=planner.command(points-p,desired,v,.05,obstacles=obs,best_effort_escape=True,emergency_mode=bool(obs))
            dv=decision.velocity_enu_mps-v
            new=v+dv*min(1.,planner.avoidance_acceleration*.05/max(np.linalg.norm(dv),1e-9))
            p+=(v+new)*.025;v=new
        self.assertTrue(triggered);self.assertGreater(minimum,.8)

    def test_fast_side_approach_improves_predicted_separation_or_brakes(self):
        g=RawLidarEmergency();p=np.array([0.,0.,3.])
        g.update([[0,1.95,3]],p,p,1_000_000_000,0.,True)
        obs,points=g.update([[0,1.75,3]],p,p,1_050_000_000,0.,True)
        planner=self.planner();desired=g.preference([0,0,0],obs,.8,planner.speed_limit)
        d=planner.command(points-p,desired,[0,0,0],.05,obstacles=obs,best_effort_escape=True,emergency_mode=True)
        if np.linalg.norm(d.velocity_enu_mps)>.01:
            gap=candidate_clearance([d.velocity_enu_mps],[0,0,0],obs,3.,DEFAULTS,.2)[0]
            brake=candidate_clearance([[0,0,0]],[0,0,0],obs,3.,DEFAULTS,.2)[0]
            self.assertGreater(gap,brake+.02)
            self.assertTrue(d.best_effort_escape or gap>=0.)
        else:self.assertTrue(d.emergency_stop)

    def test_surrounded_stop_and_descent_constraint(self):
        directions=np.vstack((np.eye(3),-np.eye(3)))
        g,(obs,world)=self.guard(directions*.7)
        planner=self.planner();desired=g.preference([1,0,0],obs,.8,planner.speed_limit)
        d=planner.command(world-[0,0,3],desired,[0,0,0],.05,obstacles=obs,
            minimum_vertical_velocity_mps=0.,best_effort_escape=True,emergency_mode=True)
        np.testing.assert_allclose(d.velocity_enu_mps,[0,0,0]);self.assertTrue(d.emergency_stop)

    def test_config_migration_roundtrip_and_invalid_limits(self):
        d=defaults(ROOT);old=copy.deepcopy(d);old['version']=7;old.pop('raw_emergency')
        migrated=upgrade(old);self.assertEqual(migrated['version'],8)
        self.assertEqual(migrated['raw_emergency']['radius_m'],2.)
        d['raw_emergency']['radius_m']=3.
        out,c,_,_,_=artifacts(d,ROOT)
        self.assertEqual(c['safety']['raw_emergency']['radius_m'],3.)
        self.assertEqual(c['safety']['local_velocity']['detection_radius_m'],2.)
        self.assertEqual(upgrade(out),out)
        for changes in ({'radius_m':0.},{'radius_m':float('nan')},{'enabled':1},{'typo':2}):
            with self.assertRaises(ValueError):options(changes)

if __name__=='__main__':unittest.main()
