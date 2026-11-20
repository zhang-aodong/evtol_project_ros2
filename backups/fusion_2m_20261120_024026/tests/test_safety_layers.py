import unittest
import numpy as np
from evtol_safety.layers import emergency_points,descent_limit,fused_route_points
from evtol_safety.velocity import LocalVelocityAvoidance
from evtol_safety.dynamic import Obstacle
from evtol_safety.navigation import WaypointMission

class SafetyLayers(unittest.TestCase):
    def test_raw_returns_outside_two_metres_do_not_slow_cruise(self):
        p=LocalVelocityAvoidance(dict(detection_radius_m=2,clearance_m=.8,braking_acceleration_mps2=6,max_speed_mps=2),.2,.05,0.)
        p.previous=np.array([2.,0,0])
        result=p.command([[2.01,0,0],[0,5,0]],[2,0,0],[2,0,0],.05)
        np.testing.assert_allclose(result.velocity_enu_mps,[2,0,0]);self.assertFalse(result.avoidance_active)
        self.assertIsNone(result.nearest_obstacle_m)
        result=p.command([[1.2,0,0]],[2,0,0],[2,0,0],.05)
        self.assertTrue(result.avoidance_active)

    def test_raw_returns_never_seed_distant_route(self):
        points=emergency_points([[1.99,0,0],[2.01,0,0],[4,0,0]],np.zeros(3))
        self.assertEqual(len(points),1)

    def test_fused_target_is_represented_before_raw_emergency_range(self):
        o=Obstacle(1,np.array([0,4.,8.]),np.zeros(3),1.55,.1,.1)
        geometry=fused_route_points([o],np.zeros(3),1.)
        self.assertGreater(len(geometry),1);self.assertTrue(np.any(geometry[:,1]<4))

    def test_level_leg_cannot_evade_down_to_ground(self):
        self.assertEqual(descent_limit([0,10,9.6],[0,50,10],[0,0,10],'waypoint',0,.8,6),0.)
        self.assertEqual(descent_limit([0,10,.8],[0,50,0],None,'landing_approach',0,.8,6),0.)
        self.assertLess(descent_limit([0,10,8],[0,50,3],[0,0,10],'waypoint',0,.8,6),0.)

    def test_ground_endpoint_without_landing_holds_above_support(self):
        c=dict(waypoints_enu_m=[[0,0,0]],land_at_last=False,tolerance_m=.35,arrival_hold_s=.2,landing_handoff_m=1.2)
        m=WaypointMission(c);target,phase=m.reference([0,0,3],[0,0,0],0.)
        np.testing.assert_allclose(target,[0,0,1.2]);self.assertNotEqual(phase,'landing')
        m.reference(target,[0,0,0],1.);_,phase=m.reference(target,[0,0,0],2.)
        self.assertEqual(phase,'complete')

if __name__=='__main__':unittest.main()
