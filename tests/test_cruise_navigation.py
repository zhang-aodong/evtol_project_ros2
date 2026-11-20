import queue
import time
import unittest
from unittest.mock import patch
import numpy as np

from evtol_safety.route_following import SampledLeg,cruise_velocity
from evtol_safety.navigation import WaypointMission
from evtol_safety.async_navigation import AsyncNavigator


class SampledCruise(unittest.TestCase):
    def test_arbitrary_start_and_recovery_keep_original_leg(self):
        leg=SampledLeg({})
        goal=np.array([37.,-8.,12.]);start=np.array([7.,-8.,12.])
        target=leg.reference(start,goal,0,[])
        np.testing.assert_allclose(target,[11,-8,12])
        self.assertLessEqual(np.linalg.norm(np.diff(leg.samples,axis=0),axis=1).max(),2.)
        leg.reference([13,-5,12],goal,0,[])
        np.testing.assert_allclose(leg.anchor,start)
        self.assertTrue(np.allclose(np.asarray(leg.samples)[:,1],-8))

    def test_occupied_intermediate_sample_uses_later_sample(self):
        leg=SampledLeg({})
        target=leg.reference(np.zeros(3),[20,0,0],0,[[4,0,0]])
        np.testing.assert_allclose(target,[6,0,0])
        self.assertTrue(np.allclose(leg.samples[-1],[20,0,0]))

    def test_collinear_gate_does_not_require_zero_speed_but_turn_and_final_do(self):
        c=dict(waypoints_enu_m=[[0,0,5],[10,0,5],[20,0,5]],tolerance_m=.35,
               arrival_speed_mps=.3,arrival_hold_s=.3,land_at_last=False)
        mission=WaypointMission(c);mission.index=1
        mission.reference([9.9,0,5],[2,0,0],1.)
        self.assertEqual(mission.index,2)
        mission.reference([20,0,5],[2,0,0],2.)
        self.assertFalse(mission.completed)
        turn=WaypointMission(dict(c,waypoints_enu_m=[[0,0,5],[10,0,5],[10,10,5]]));turn.index=1
        turn.reference([10,0,5],[2,0,0],1.)
        self.assertEqual(turn.index,1)

    def test_cruise_speed_survives_longitudinal_damping_and_sampling(self):
        leg=SampledLeg({});speeds=[]
        for x in np.arange(0,26,.1):
            p=np.array([x,0.,5.]);aim=leg.reference(p,[30,0,5],0,[])
            v=cruise_velocity(p,aim,np.array([2.,0,0]),2.,30-x,False,{},3.)
            speeds.append(np.linalg.norm(v))
        np.testing.assert_allclose(speeds,2.,atol=1e-10)
        v=cruise_velocity(np.array([29.9,0,5]),np.array([30,0,5]),np.array([2.,0,0]),2.,.1,True,{},3.)
        self.assertLess(np.linalg.norm(v),.3)

    def test_sampling_has_bounded_memory_and_local_horizon(self):
        leg=SampledLeg(dict(max_reference_samples=100))
        goal=leg.reference(np.zeros(3),[100000,0,0],0,[])
        self.assertLessEqual(len(leg.samples),100)
        self.assertLessEqual(np.linalg.norm(goal),9.)


class AsyncRouteContinuity(unittest.TestCase):
    def fake_worker(self,nav):
        class Process:
            def is_alive(self):return True
        nav.process=Process();nav.requests=queue.Queue(maxsize=1);nav.results=queue.Queue(maxsize=1)

    def test_clear_corridor_never_waits_for_worker(self):
        nav=AsyncNavigator({})
        with patch.object(nav,'_start',side_effect=AssertionError('No worker needed')):
            self.assertIsNotNone(nav.plan([0,0,0],[6,0,0],[]))
        self.assertEqual(nav.reason,'sampled_corridor_clear')

    def test_failed_refresh_keeps_revalidated_detour_and_obstacle_invalidates_it(self):
        nav=AsyncNavigator({});self.fake_worker(nav)
        nav.goal=np.array([6.,0,0]);nav.version=1
        nav.path=[np.array(p,float) for p in ([0,0,0],[0,3,0],[6,3,0],[6,0,0])]
        nav.results.put((1,nav.goal,[],'planning_budget_exceeded',time.monotonic()))
        self.assertIsNotNone(nav.plan([0,0,0],[6,0,0],[[3,0,0]]))
        self.assertEqual(nav.reason,'tracking_checked_route')
        # An obstacle at the current position invalidates every outgoing
        # segment, including any previously safe asynchronous result.
        self.assertIsNone(nav.plan([0,0,0],[6,0,0],[[0,0,0]]))

    def test_previous_goal_result_cannot_be_applied_after_reference_advances(self):
        nav=AsyncNavigator({});self.fake_worker(nav)
        nav.goal=np.array([4.,0,0]);nav.version=1;nav.busy=True
        nav.results.put((1,nav.goal,[[0,0,0],[4,0,0]],'direct',time.monotonic()))
        nav.plan([0,0,0],[6,0,0],[[6,0,0],[3,0,0]])
        self.assertFalse(nav.path);self.assertEqual(nav.version,2)


if __name__=='__main__':unittest.main()
