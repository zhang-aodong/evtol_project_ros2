"""Waypoint ordering, 3-D detours, unreachable goals and final landing rules."""
from pathlib import Path
import sys
import unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/evtol_safety'))
from evtol_safety.navigation import WaypointMission,VoxelNavigator


class NavigationTests(unittest.TestCase):
    def settings(self):return dict(waypoints_enu_m=[[0,0,3],[0,30,3],[0,30,0]],tolerance_m=.35,arrival_hold_s=.2,arrival_speed_mps=.3,landing_handoff_m=1.2)

    def test_climb_then_translate_then_landing_handoff(self):
        mission=WaypointMission(self.settings())
        target,phase=mission.reference([0,0,1.5],[0,0,0],0.)
        np.testing.assert_allclose(target,[0,0,3]);self.assertEqual(mission.index,0)
        mission.reference([0,0,3],[0,0,1],1.);self.assertEqual(mission.index,0)
        mission.reference([0,0,3],[0,0,0],2.)
        target,phase=mission.reference([0,0,3],[0,0,0],2.3)
        np.testing.assert_allclose(target,[0,30,3]);self.assertEqual(mission.index,1)
        mission.reference([0,30,3],[0,0,0],3.)
        target,_=mission.reference([0,30,3],[0,0,0],3.3)
        np.testing.assert_allclose(target,[0,30,1.2]);self.assertEqual(mission.index,2)
        mission.reference([0,30,1.2],[0,0,0],4.)
        _,phase=mission.reference([0,30,1.2],[0,0,0],4.3)
        self.assertEqual(phase,'landing');self.assertFalse(mission.completed)
        # Gateway status is asynchronous. Descent must not change the
        # request back to approach while its last status still says flying.
        _,phase=mission.reference([0,30,.8],[0,0,-.7],4.4,'flying')
        self.assertEqual(phase,'landing')
        _,phase=mission.reference([0,30,0],[0,0,0],5.,'landed')
        self.assertEqual(phase,'complete');self.assertTrue(mission.completed)

    def test_airborne_endpoint_hovers_and_disabled_landing_is_exact_waypoint(self):
        c=self.settings();c['waypoints_enu_m']=[[3,-2,5]]
        mission=WaypointMission(c);self.assertFalse(mission.land)
        mission.reference([3,-2,5],[0,0,0],0.)
        _,phase=mission.reference([3,-2,5],[0,0,0],1.)
        self.assertEqual(phase,'complete')
        c=self.settings();c['land_at_last']=False
        self.assertFalse(WaypointMission(c).land)

    def test_bootstrap_cannot_advance_waypoints(self):
        mission=WaypointMission(self.settings())
        for phase in ('waiting','arming','takeoff','failed','stopped','shadow'):
            mission.reference([0,0,3],[0,0,0],5.,phase)
            self.assertEqual(mission.index,0)

    def test_full_3d_direct_flight_and_rear_target(self):
        planner=VoxelNavigator(dict(clearance_m=.8,search_budget_s=1.))
        for goal in ([0,0,3],[0,0,-3],[-4,0,0],[2,-3,4]):
            aim=planner.plan([0,0,0],goal,np.empty((0,3)))
            self.assertIsNotNone(aim)
            self.assertGreater(np.asarray(goal)@aim,0)

    def test_wall_requires_collision_checked_3d_detour(self):
        yz=np.array(np.meshgrid(np.arange(-1.,1.01,.2),np.arange(-1.,1.01,.2))).reshape(2,-1).T
        wall=np.column_stack([np.full(len(yz),2.),yz])
        planner=VoxelNavigator(dict(clearance_m=.55,resolution_m=.5,planning_radius_m=7.,search_budget_s=3.))
        aim=planner.plan([0,0,0],[4,0,0],wall)
        self.assertIsNotNone(aim,planner.reason);self.assertEqual(planner.reason,'detour')
        self.assertTrue(any(np.linalg.norm(np.asarray(p)[1:])>1.3 for p in planner.path))
        from scipy.spatial import cKDTree
        tree=cKDTree(wall)
        for a,b in zip(planner.path,planner.path[1:]):
            samples=np.linspace(a,b,100)
            self.assertGreaterEqual(np.min(tree.query(samples)[0]),.55)

    def test_goal_inside_obstacle_is_not_forced(self):
        planner=VoxelNavigator(dict(search_budget_s=1.))
        self.assertIsNone(planner.plan([0,0,0],[3,0,0],[[3,0,0]]))
        self.assertEqual(planner.reason,'goal_obstructed')

    def test_search_budget_fails_closed(self):
        planner=VoxelNavigator(dict(search_budget_s=1e-9))
        self.assertIsNone(planner.plan([0,0,0],[4,0,0],[[2,0,0]]))
        self.assertEqual(planner.reason,'planning_budget_exceeded')

    def test_distant_waypoint_can_detour_around_blocked_local_endpoint(self):
        planner=VoxelNavigator(dict(search_budget_s=3.))
        self.assertIsNotNone(planner.plan([0,0,0],[20,0,0],[[9,0,0]]),planner.reason)
        self.assertEqual(planner.reason,'detour')


if __name__=='__main__':unittest.main()
