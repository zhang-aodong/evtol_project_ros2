import unittest
from evtol_simulation.backend import AirSimBackend


class NativeKinematics(unittest.TestCase):
    def sample(self,**changes):
        response=dict(message='kinematics_state_v1|1000000000|1000001000',time_stamp=1000000500,
            image_data_float=[1,2,-3,0,0,0,1,4,5,6,.1,.2,.3])
        response.update(changes)
        class RPC:
            def call(self,*args):self.request=args;return [response]
        backend=object.__new__(AirSimBackend);backend.c=dict(native_kinematics=True,max_truth_bracket_ms=50.)
        backend.vehicle='Drone1';backend.rpc=RPC();return backend

    def test_own_pose_velocity_and_orientation_use_native_sample(self):
        backend=self.sample();t,state=backend.truth()
        self.assertEqual(t,1000000500);self.assertEqual(state['position']['z_val'],-3.)
        self.assertEqual(state['orientation']['w_val'],1.);self.assertEqual(state['linear_velocity']['y_val'],5.)
        self.assertEqual(backend.rpc.request[0],'simGetImages')
        self.assertEqual(backend.rpc.request[1][0]['camera_name'],'kinematics_state:Drone1')

    def test_missing_plugin_does_not_masquerade_as_valid_truth(self):
        with self.assertRaises(ValueError):self.sample(message='camera missing').truth()

    def test_wide_bracket_and_nonfinite_state_are_rejected(self):
        with self.assertRaises(ValueError):self.sample(message='kinematics_state_v1|1000000000|1100000000').truth()
        with self.assertRaises(ValueError):self.sample(image_data_float=[float('nan')]*13).truth()


if __name__=='__main__':unittest.main()
