"""Migration and evaluation contracts for the single incoming localization implementation."""
import copy
import unittest

from evtol_localization.options import options
from evtol_localization.settings import estimator_module


class LocalizationConfigurationTests(unittest.TestCase):
    def test_old_saved_selector_always_migrates_to_branch(self):
        old = dict(enabled=True, mode='gnss_imu_5g', estimator='legacy',
                   accel_noise=.12, fiveg_sigma_floor_m=.4, max_queue=3000,
                   fiveg_min_rsrp_dbm=-80., absolute={})
        saved = copy.deepcopy(old)
        result = options(old)
        self.assertEqual(result['estimator'], 'absolute')
        self.assertEqual(result['absolute']['accel_noise_density'], .12)
        self.assertEqual(result['absolute']['range_sigma_floor_m'], .4)
        self.assertEqual(result['absolute']['min_rsrp_dbm'], -80.)
        self.assertNotIn('max_queue', result)
        self.assertNotIn('accel_noise', result)
        self.assertEqual(old, saved)
        self.assertEqual(estimator_module(dict(localization=old, state={'source':'truth'})),
                         'evtol_localization.absolute_node')

    def test_explicit_branch_parameters_override_migrated_fields(self):
        o = options(dict(gnss_sigma_floor_m=.8, absolute={'gnss_sigma_floor_m':.2}))
        self.assertEqual(o['absolute']['gnss_sigma_floor_m'], .2)

    def test_console_upgrade_migrates_version_8_without_saving_or_starting(self):
        from pathlib import Path
        from evtol_console.configuration import defaults
        from evtol_console.experiment_config import upgrade
        d = defaults(Path(__file__).resolve().parents[1])
        d['localization'].update(estimator='legacy', fiveg_sigma_floor_m=.5)
        d['localization']['absolute'].pop('range_sigma_floor_m')
        migrated = upgrade(d)
        self.assertEqual(migrated['localization']['estimator'], 'absolute')
        self.assertEqual(migrated['localization']['absolute']['range_sigma_floor_m'], .5)
        self.assertEqual(d['localization']['estimator'], 'legacy')

    def test_single_runtime_class_is_the_branch_class(self):
        from evtol_localization.node import LocalizationNode
        from evtol_localization.absolute_node import AbsoluteNode
        self.assertIs(LocalizationNode, AbsoluteNode)

    def test_vio_cannot_own_enu_control(self):
        with self.assertRaises(ValueError):
            options(dict(mode='camera_imu', state_source='fused'))
        self.assertEqual(options(dict(mode='camera_imu', estimator='legacy'))['estimator'], 'vio')

    def test_branch_defaults_and_invalid_inputs(self):
        o = options()
        self.assertEqual(o['fix_profile'], 'standalone')
        self.assertEqual(o['fiveg_profile'], 'moderate')
        self.assertEqual(o['absolute']['min_rsrp_dbm'], -32.)
        for value in ({'absolute':{'typo':1}}, {'vio':{'max_image_width':0}},
                      {'fiveg_added_sigma_m':float('nan')}, {'unknown':1}):
            with self.assertRaises(ValueError): options(value)
