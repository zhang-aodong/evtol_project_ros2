import unittest
import numpy as np
from evtol_safety.corridor import near_obstacles,local_points,route_aim
from evtol_safety.dynamic import Obstacle
from evtol_safety.velocity import LocalVelocityAvoidance


class CorridorContracts(unittest.TestCase):
    def test_remote_static_target_does_not_join_near_collision_rollout(self):
        targets=[Obstacle(1,np.array([8.,0,0]),np.zeros(3),1.),
                 Obstacle(2,np.array([3.,0,0]),np.zeros(3),1.)]
        selected=near_obstacles(targets,np.zeros(3),4.,1.)
        self.assertEqual([o.track_id for o in selected],[2])

    def test_fast_side_entry_is_retained_before_it_is_currently_nearby(self):
        target=Obstacle(1,np.array([2.,12.,0]),np.array([0.,-20.,0]),1.)
        self.assertEqual(len(near_obstacles([target],np.zeros(3),4.,1.)),1)

    def test_following_does_not_cut_unchecked_corner(self):
        goal=route_aim(np.array([1.9,0,0]),[[0,0,0],[2,0,0],[2,4,0]],1.5)
        np.testing.assert_allclose(goal,[2.,1.5,0.])
        goal=route_aim(np.array([.5,0,0]),[[0,0,0],[2,0,0],[2,4,0]],4.)
        np.testing.assert_allclose(goal,[2.,0,0])

    def test_clear_corridor_does_not_swerve_for_off_path_returns(self):
        planner=LocalVelocityAvoidance(dict(detection_radius_m=4.,clearance_m=.8,max_speed_mps=2.))
        planner.previous=np.array([2.,0,0])
        points=np.array([[x,2.,0] for x in np.arange(-3.,3.1,.2)])
        decision=planner.command(points,[2,0,0],[2,0,0],.05)
        np.testing.assert_allclose(decision.velocity_enu_mps,[2,0,0]);self.assertFalse(decision.avoidance_active)

    def test_front_obstacle_and_short_range_stop_envelope_still_apply(self):
        planner=LocalVelocityAvoidance(dict(detection_radius_m=4.,clearance_m=.8,max_speed_mps=2.))
        planner.previous=np.array([2.,0,0])
        decision=planner.command([[1.,0,0]],[2,0,0],[2,0,0],.05)
        self.assertFalse(np.allclose(decision.velocity_enu_mps,[2,0,0]))
        self.assertLessEqual(np.linalg.norm(decision.velocity_enu_mps),2.)
        self.assertGreater(planner.speed_limit,1.99)

    def test_far_points_do_not_enter_local_cloud(self):
        points=np.array([[0,0,0],[3,0,0],[10,0,0]])
        self.assertEqual(len(local_points(points,np.zeros(3),6.)),2)


if __name__=='__main__':unittest.main()
