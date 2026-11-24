from pathlib import Path
import sys
import unittest

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/evtol_localization"))
from evtol_localization.settings import experiment_config, estimator_module
from evtol_localization.metrics import summarize


class WorkflowTest(unittest.TestCase):
    def test_config_is_explicit_and_does_not_edit_source(self):
        base = yaml.safe_load((ROOT / "config/targeting.yaml").read_text(encoding="utf-8-sig"))
        for mode in ("gnss_imu", "gnss_imu_5g", "camera_imu"):
            settings = experiment_config(base, mode, False, False)
            self.assertEqual(settings["state"]["source"], "truth")
            self.assertEqual(settings["localization"]["fiveg_added_sigma_m"], 1.2)
            self.assertTrue(estimator_module(settings).endswith(
                "vio_node" if mode == "camera_imu" else "absolute_node"))
        self.assertEqual(base["state"]["source"], "truth")
        self.assertEqual(estimator_module(base), 'evtol_localization.node')
        self.assertIsNone(estimator_module({'localization': {'enabled': False}}))
        with self.assertRaises(ValueError):
            experiment_config(base, "camera_imu", True, False)
        self.assertEqual(experiment_config(base, "gnss_imu", True, True)
                         ["localization"]["gnss_fault"]["segments"][0]['end_s'], 60.)

    def test_absolute_error_preserves_bias(self):
        t = np.arange(71, dtype=np.int64) * 100_000_000
        xyz = np.column_stack((t * 1e-9, np.zeros((len(t), 2))))
        yaw = np.zeros(len(t))
        report = summarize((t, xyz, yaw), (t, xyz + [1., 2., 2.], yaw), False)
        self.assertAlmostEqual(report["absolute_3d_rmse_m"], 3.)
        self.assertAlmostEqual(report["rpe_5s_rmse_m"], 0.)
        self.assertEqual(report["matched_epochs"], len(t))

    def test_vio_initial_yaw_alignment_and_stream_gaps(self):
        t = np.arange(81, dtype=np.int64) * 100_000_000
        east = np.column_stack((t * 1e-9, np.zeros((len(t), 2))))
        north = east[:, [1, 0, 2]] + [4., 7., 0.]
        keep = (t < 2e9) | (t >= 4e9)
        report = summarize((t, east, np.zeros(len(t))),
                           (t[keep], north[keep], np.full(keep.sum(), np.pi/2)), True)
        self.assertNotIn("absolute_3d_rmse_m", report)
        self.assertAlmostEqual(report["aligned_3d_rmse_m"], 0.)
        self.assertLess(report["coverage"], 1.)
        self.assertIsNone(report["rpe_5s_rmse_m"])
