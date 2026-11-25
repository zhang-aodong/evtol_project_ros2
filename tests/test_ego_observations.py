import sys
import time
import unittest
from importlib.util import find_spec
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'evtol_simulation'))
from evtol_simulation.backend import MockBackend
from evtol_simulation.configuration import load
from evtol_simulation.fix_error import StandaloneFixError, displace_fix, fix_fields
from evtol_simulation.gnss_model import enu_to_ecef_matrix, geodetic_to_ecef
from evtol_simulation.radio_model import FiveGAState, FiveGASimulator


class EgoObservationTests(unittest.TestCase):
    def setUp(self):
        self.config = load(ROOT / 'src/evtol_simulation/config/simulation.yaml')
        self.radio = self.config['source']['five_g_a']

    def test_fix_wgs84_displacement_and_covariance(self):
        offset = [3., -5., 8.]
        for origin in [(47.64, -122.14, 122.), (-13., 179.99999, -20.), (89.999999, -2., 7.)]:
            shifted = displace_fix(*origin, offset)
            reconstructed = enu_to_ecef_matrix(*origin[:2]).T @ (
                geodetic_to_ecef(*shifted) - geodetic_to_ecef(*origin))
            np.testing.assert_allclose(reconstructed, offset, atol=1e-7)
        gps = MockBackend(self.config, 'gnss').capture()[1]['gnss']
        original, valid, native_cov = fix_fields(gps, 10)
        observed, degraded_valid, covariance = fix_fields(gps, 10, StandaloneFixError())
        self.assertTrue(valid and degraded_valid)
        self.assertNotEqual(original, observed)
        np.testing.assert_allclose(np.reshape(covariance, (3, 3)) - np.reshape(native_cov, (3, 3)),
                                   np.diag(StandaloneFixError.variance_m2))
        gps['fix_type'] = 1
        no_fix, valid, cov = fix_fields(gps, 10, StandaloneFixError())
        self.assertFalse(valid)
        self.assertEqual(no_fix, original)
        self.assertEqual(cov, native_cov)

    def test_fix_temporal_correlation_and_stationary_variance(self):
        model = StandaloneFixError()
        repeat = StandaloneFixError()
        np.testing.assert_array_equal(model.sample(0), repeat.sample(0))
        errors = np.asarray([model.sample(i * 1_000_000_000) for i in range(1, 24001)])
        np.testing.assert_allclose(errors.var(axis=0), model.variance_m2, rtol=.10)
        empirical = np.mean(errors[1:] * errors[:-1], axis=0)
        prediction = model.bias_std_m**2 * np.exp(-1 / model.correlation_time_s)
        np.testing.assert_allclose(empirical, prediction, rtol=.12)

    def test_mock_fix_matches_map_height_only_for_new_mode(self):
        for enabled, height in [(False, 0.), (True, 10.)]:
            self.config['localization'] = {'enabled': enabled}
            backend = MockBackend(self.config, 'gnss')
            geo = backend.capture()[1]['gnss']['geo_point']
            self.assertEqual(geo['altitude'] - backend.home()['altitude'], height)

    @unittest.skipUnless(find_spec('rclpy') and find_spec('evtol_sim_interfaces'),
                         'requires a built ROS 2 workspace')
    def test_late_subscriber_receives_home_with_raw_gnss_disabled(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import NavSatFix
        from evtol_simulation.sensor_node import SensorNode

        self.config.update(backend='mock', namespace='test_home',
                           localization={'enabled': True, 'fix_profile': 'standalone'})
        self.config['source']['gnss_raw']['enabled'] = False
        rclpy.init()
        sensor = SensorNode(self.config, 'gnss')
        listener = Node('test_late_home_listener')
        try:
            sensor.timer.cancel()
            sensor.tick()
            self.assertEqual(sensor.errors, 0, sensor.last_error)
            self.assertNotIn('gnss/raw_observations', sensor.pubs)
            first_stamp = sensor.last_stamp
            sensor.tick()
            received = []
            datum_received = []
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            listener.create_subscription(NavSatFix, '/test_home/gnss/home', received.append, qos)
            listener.create_subscription(NavSatFix, '/simulation/geodetic_reference', datum_received.append, qos)
            deadline = time.monotonic() + 5
            while (not received or not datum_received) and time.monotonic() < deadline:
                rclpy.spin_once(listener, timeout_sec=.05)
            self.assertEqual(len(received), 1)
            self.assertEqual(len(datum_received), 1)
            home = received[0]
            self.assertEqual(home, datum_received[0])
            self.assertEqual(home.header.stamp.sec * 10**9 + home.header.stamp.nanosec, first_stamp)
            self.assertEqual(home.header.frame_id, self.config['frames']['world'])
            self.assertEqual(home.altitude, sensor.backend.home()['altitude'])
        finally:
            sensor.close()
            sensor.destroy_node()
            listener.destroy_node()
            rclpy.shutdown()

    @staticmethod
    def state(t):
        return FiveGAState(round(t * 10), round(t * 1e9), t, 'test',
                           np.array([0., 0., -10.]), np.zeros(3))

    def test_radio_window_boundaries_original_scene_and_rtt(self):
        forced_nlos = lambda rover, station: False
        native = FiveGASimulator(self.radio, forced_nlos)
        moderate = FiveGASimulator(self.radio, forced_nlos, profile='moderate', added_sigma_m=0.)
        for t in (0, 20, 30, 35, 43, 46, 55, 60, 75, 76, 120):
            for a, b in zip(native.simulate_epoch(self.state(t)), moderate.simulate_epoch(self.state(t))):
                site = a.gnb_id
                outage = ((site == 'gNB6' and 20 <= t < 55) or
                          (site == 'gNB5' and 35 <= t < 43) or
                          (site == 'gNB3' and 55 <= t < 75))
                scheduled = ((site == 'gNB2' and 30 <= t < 46) or
                             (site == 'gNB4' and 60 <= t < 76))
                bias = (3. + np.sin(2 * np.pi * t / 8 + (site == 'gNB4'))) if scheduled else 0.
                self.assertEqual(b.is_los, 0)
                self.assertAlmostEqual(b.nlos_bias_m - a.nlos_bias_m, bias)
                self.assertAlmostEqual(a.prs_rsrp_dbm - b.prs_rsrp_dbm, 5. if scheduled else 0.)
                self.assertEqual(bool(b.measurement_valid), not outage)
                if outage:
                    self.assertTrue(np.isnan(b.measured_range_m) and np.isnan(b.measured_rtt_s))
                else:
                    self.assertAlmostEqual(b.measured_range_m, a.measured_range_m + bias)
                    self.assertAlmostEqual(b.measured_rtt_s * self.radio['speed_of_light_mps'] / 2.,
                                           b.measured_range_m)

    def test_radio_added_noise_independent_of_native_and_matches_weights(self):
        native = FiveGASimulator(self.radio)
        moderate = FiveGASimulator(self.radio, profile='moderate')
        differences = []
        original_noise = []
        # Sample outside the scheduled stress windows, isolating white noise.
        for i in range(1000):
            state = self.state(90 + i / 10)
            for a, b in zip(native.simulate_epoch(state), moderate.simulate_epoch(state)):
                self.assertEqual(b.is_los, 1)
                self.assertAlmostEqual(b.range_sigma_m**2, a.range_sigma_m**2 + 1.2**2)
                original_noise.append(a.measured_range_m - a.true_range_m)
                differences.append(b.measured_range_m - a.measured_range_m)
        self.assertAlmostEqual(np.std(differences), 1.2, delta=.05)
        self.assertLess(abs(np.corrcoef(differences, original_noise)[0, 1]), .05)


if __name__ == '__main__':
    unittest.main()
