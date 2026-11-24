import unittest
import numpy as np
from scipy.spatial import cKDTree
from evtol_safety.recovery import RouteRecovery,options
from evtol_safety.navigation import VoxelNavigator
from evtol_safety.route_following import SampledLeg


class RecoveryTests(unittest.TestCase):
    def make(self):return RouteRecovery(dict(planning_radius_m=6.,clearance_m=.8))
    def stall(self,r,p=(0,0,3),target=(0,30,3)):
        for t in np.arange(0,3.01,.5):r.observe(p,target,'leg',0.,t,blocked=True)
        self.assertTrue(r.active)

    def test_cruise_and_final_approach_do_not_trigger(self):
        r=self.make()
        for t in np.arange(0,10,.1):r.observe([0,t,3],[0,30,3],'leg',t,t)
        self.assertFalse(r.active)
        for t in np.arange(10,20,.1):r.observe([0,29.5,3],[0,30,3],'leg',29.5,t)
        self.assertFalse(r.active)

    def test_side_exit_rejoins_original_leg_without_overwriting_goal(self):
        r=self.make();self.stall(r)
        # A forward wall occupies the usual sample and stops the direct route.
        wall=np.array([[x,2.,z] for x in np.arange(-1,1.01,.2) for z in np.arange(1,5.01,.2)])
        target=np.array([0.,30.,3.]);q=r.reference([0,0,3],target,[0,4,3],wall,3.,.8)
        self.assertIsNotNone(q);self.assertGreater(abs(q[0]),1.)
        planner=VoxelNavigator(dict(planning_radius_m=6.,clearance_m=.8,search_budget_s=2.))
        self.assertIsNotNone(planner.plan([0,0,3],q,wall),planner.reason)
        tree=cKDTree(wall)
        for a,b in zip(planner.path,planner.path[1:]):self.assertGreaterEqual(tree.query(np.linspace(a,b,100))[0].min(),.8)
        r.observe(q,target,'leg',q[1],4.)
        self.assertFalse(r.active);np.testing.assert_array_equal(target,[0,30,3])
        np.testing.assert_array_equal(r.reference(q,target,[0,8,3],wall,4.,.8),[0,8,3])

    def test_backtracking_is_allowed_when_forward_and_sides_are_closed(self):
        r=self.make();self.stall(r)
        # Dense occupied volume leaves only the rear half-space available.
        points=np.array([[x,y,z] for x in np.arange(-6,6.1,.3) for y in np.arange(-.4,6.1,.3)
            for z in np.arange(0,6.1,.4) if abs(x)>=.9 or y>=.9])
        q=r.reference([0,0,3],[0,30,3],[0,4,3],points,3.,.8)
        self.assertIsNotNone(q);self.assertLess(q[1],0.)

    def test_no_free_endpoints_holds_and_expands_with_finite_cap(self):
        r=self.make();self.stall(r)
        points=np.array([[x,y,z] for x in np.arange(-16,16.1,.6) for y in np.arange(-16,16.1,.6) for z in np.arange(0,7.1,.6)])
        self.assertIsNone(r.reference([0,0,3],[0,30,3],[0,4,3],points,3.,.8))
        for t in (10.,17.,24.,31.):
            r.observe([0,0,3],[0,30,3],'leg',0.,t)
            r.reference([0,0,3],[0,30,3],[0,4,3],points,t,.8)
        self.assertEqual(r.radius,15.);self.assertEqual(r.reason,'blocked_no_recovery_goal')

    def test_chosen_goal_persists_and_emergency_resets_progress_window(self):
        r=self.make();self.stall(r)
        q=r.reference([0,0,3],[0,30,3],[0,4,3],[],3.,.8)
        np.testing.assert_array_equal(q,r.reference([.1,0,3],[0,30,3],[0,4,3],[],3.5,.8))
        r.pause();r.observe([0,0,3],[0,30,3],'leg',0.,20.)
        self.assertFalse(r.active)
        r.observe([0,0,3],[0,30,3],'new-leg',0.,30.)
        self.assertFalse(r.active)

    def test_config_rejects_unbounded_or_inconsistent_settings(self):
        for value in ({'max_radius_m':1000},{'stalled_s':float('nan')},{'attempt_s':3.,'switch_hold_s':4.}):
            with self.assertRaises(ValueError):options(value)

    def test_closed_loop_occupied_sample_recovers_and_reaches_original_destination(self):
        settings=dict(planning_radius_m=6.,tracking_lookahead_m=3.,clearance_m=.8,resolution_m=.5,search_budget_s=.3)
        recovery=RouteRecovery(settings);leg=SampledLeg(settings);planner=VoxelNavigator(settings)
        p=np.array([0.,0.,3.]);target=np.array([0.,12.,3.])
        wall=np.array([[x,4.,z] for x in np.arange(-1,1.01,.2) for z in np.arange(1,5.01,.2)])
        tree=cKDTree(wall);entered=False;minimum=np.inf
        for now in np.arange(0,90,.5):
            normal=leg.reference(p,target,'leg',wall)
            recovery.observe(p,target,'leg',float((p-leg.anchor)@leg.direction),now,blocked=planner.reason in ('goal_obstructed','no_local_route'))
            q=recovery.reference(p,target,normal,wall,now,2.6)
            entered|=recovery.active
            if q is None:continue
            planner.set_radius(recovery.radius if recovery.active else 6.)
            aim=planner.plan(p,q,wall)
            if aim is None:continue
            delta=aim-p;p=p+delta*min(1.,.45/max(np.linalg.norm(delta),1e-9))
            minimum=min(minimum,tree.query(p)[0])
            if np.linalg.norm(p-target)<.35:break
        self.assertTrue(entered,'Occupied normal sample should activate recovery')
        self.assertLess(np.linalg.norm(p-target),.35,p)
        self.assertGreaterEqual(minimum,.8)

    def test_slow_unobstructed_vertical_flight_never_creates_side_excursion(self):
        r=self.make()
        for now in np.arange(0,12,.5):
            r.observe([0,0,1.5+now*.02],[0,0,10],'climb',now*.02,now)
        self.assertFalse(r.active)
        self.stall(r,p=(0,0,3),target=(0,0,10))
        q=r.reference([0,0,3],[0,0,10],[0,0,7],[],3.,2.6)
        np.testing.assert_array_equal(q,[0,0,7])
        self.assertEqual(r.reason,'retry_clear_reference')
