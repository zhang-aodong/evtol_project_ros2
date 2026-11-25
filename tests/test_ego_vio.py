"""ROS-independent regression tests with calibrated synthetic image pairs."""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for package in ("evtol_simulation", "evtol_localization"):
    sys.path.insert(0, str(ROOT / "src" / package))
from evtol_localization.vio import Calibration, GRAVITY, LocalVio, gray_image, translation_direction


class VioTests(unittest.TestCase):
    def setUp(self):
        self.vio = LocalVio([.3, 0., 0.], [0., 0., 0.])
        self.cal = Calibration.from_info(SimpleNamespace(width=400, height=300,
            k=[250., 0., 200., 0., 250., 150., 0., 0., 1.], d=[0.] * 5, distortion_model="plumb_bob"))

    def rest(self, force=GRAVITY):
        for i in range(51):
            self.vio.add_imu(i * 10_000_000, force, [0., 0., 0.])

    def test_startup_tilt_and_motion_gate(self):
        angle = .2
        force = GRAVITY[2] * np.array([0., np.sin(angle), np.cos(angle)])
        self.rest(force)
        np.testing.assert_allclose(self.vio.rotation @ force, GRAVITY, atol=1.e-10)
        self.vio = LocalVio([0, 0, 0], [0, -10, 0])
        for i in range(80):
            self.vio.add_imu(i * 10_000_000, GRAVITY, [0., .07, 0.])
        self.assertIsNone(self.vio.cursor)

    def test_padding_and_bgr_encoding(self):
        raw = bytes([255, 0, 0, 0, 0, 255, 7, 7] * 2)
        msg = SimpleNamespace(width=2, height=2, step=8, data=raw, encoding="bgr8")
        np.testing.assert_array_equal(gray_image(msg, 640), [[29, 76], [29, 76]])
        with self.assertRaises(ValueError):
            gray_image(SimpleNamespace(**dict(vars(msg), step=5)), 640)

    def test_pure_rotation_has_no_translation(self):
        rng = np.random.default_rng(3)
        b1 = np.column_stack((rng.uniform(-.4, .4, (100, 2)), np.ones(100)))
        r = cv2.Rodrigues(np.array([.03, .01, -.07]))[0]
        b2 = b1 @ r.T
        b2 /= b2[:, 2:3]
        direction, reason = translation_direction(b1, b2, r, 250.)
        self.assertIsNone(direction)
        self.assertEqual(reason, "low_parallax")

    def test_two_view_direction_with_rotation_and_outliers(self):
        rng = np.random.default_rng(20)
        xyz = rng.uniform([-2., -1.5, 4.], [2., 1.5, 12.], (170, 3))
        r = cv2.Rodrigues(np.array([.05, -.08, .03]))[0]
        t = np.array([-.25, .12, .07])
        moved = xyz @ r.T + t
        b1, b2 = xyz / xyz[:, 2:3], moved / moved[:, 2:3]
        b2[:25, :2] = rng.uniform(-.3, .3, (25, 2))
        direction, reason = translation_direction(b1, b2, r, 450.)
        self.assertEqual(reason, "accepted")
        self.assertGreater(direction @ (t / np.linalg.norm(t)), .99)

    def test_metric_track_from_raw_images_and_acceleration(self):
        self.rest()
        rng = np.random.default_rng(26)
        uv = np.array([(u, v) for u in range(35, 380, 21) for v in range(20, 270, 20)])
        z = rng.uniform(4., 10., len(uv))
        points = (uv - [200., 150.]) * z[:, None] / 250.
        patterns = rng.integers(50, 240, size=(len(uv), 5, 5), dtype=np.uint8)

        def render(camera_right):
            img = np.zeros((300, 400), dtype=np.uint8)
            shifted = (points - [camera_right, 0.]) * 250. / z[:, None] + [200., 150.]
            for (u, v), pattern in zip(np.rint(shifted).astype(int), patterns):
                if 3 < u < 396 and 3 < v < 296:
                    img[v-2:v+3, u-2:u+3] = pattern
            return img

        self.assertFalse(self.vio.image(500_000_000, render(0.), self.cal))
        for i in range(51, 151):
            self.vio.add_imu(i * 10_000_000, GRAVITY + [0., -.5, 0.], [0., 0., 0.])
        self.assertTrue(self.vio.image(1_500_000_000, render(.25), self.cal), self.vio.reason)
        np.testing.assert_allclose(self.vio.x[:3], [0., -.25, 0.], atol=.03)
        np.testing.assert_allclose(self.vio.x[3:6], [0., -.5, 0.], atol=.04)
        self.assertTrue(self.vio.add_imu(1_800_000_000, GRAVITY, [0, 0, 0]))
        self.assertEqual(self.vio.frame, "vio_odom_epoch_2")
        self.assertEqual(self.vio.last_reset, "imu_gap")
        self.assertIsNone(self.vio.cursor)
        # A new stationary interval establishes a fresh independent frame.
        for i in range(181, 231):
            self.vio.add_imu(i * 10_000_000, GRAVITY, [0., 0., 0.])
        self.assertIsNotNone(self.vio.cursor)
        np.testing.assert_array_equal(self.vio.x[:6], np.zeros(6))

    def test_timestamp_rewind_requires_confirmation(self):
        self.rest()
        self.assertFalse(self.vio.add_imu(200_000_000, GRAVITY, [0., 0., 0.]))
        self.assertTrue(self.vio.add_imu(510_000_000, GRAVITY, [0., 0., 0.]))
        self.assertEqual(self.vio.resets, 0)
        for i in [0, 1, 2]:
            self.vio.add_imu(i * 10_000_000, GRAVITY, [0., 0., 0.])
        self.assertEqual(self.vio.last_reset, "imu_clock_rewind")
        self.assertIsNone(self.vio.cursor)
        self.assertEqual(self.vio.frame, "vio_odom")  # No output existed before rewind.

    def test_missing_texture_and_imu_timing(self):
        self.rest()
        blank = np.zeros((300, 400), np.uint8)
        self.assertFalse(self.vio.image(500_000_000, blank, self.cal))
        for i in range(51, 401):
            self.vio.add_imu(i * 10_000_000, GRAVITY, [0, 0, 0])
            if i == 76:
                self.assertFalse(self.vio.image(755_000_000, blank, self.cal))
                self.assertEqual(self.vio.reason, "no_features")
                self.assertEqual(self.vio.cursor.stamp, 755_000_000)
        self.assertLessEqual(len(self.vio.samples), 201)
        self.assertFalse(self.vio.image(800_000_000, blank, self.cal))
        self.assertFalse(self.vio.add_imu(4_000_000_000, GRAVITY, [0, 0, 0]))
        np.testing.assert_allclose(self.vio.x[:6], 0., atol=1.e-12)

    def test_camera_lever_arm_and_negative_scale_gate(self):
        self.rest()
        old = self.vio.rotation.copy()
        self.vio.rotation = cv2.Rodrigues(np.array([0., 0., .25]))[0]
        delta_cam = (self.vio.rotation - old) @ self.vio.arm
        direction = delta_cam / np.linalg.norm(delta_cam)
        self.assertEqual(self.vio._correct_direction(-direction, old), "unresolved_positive_scale")
        self.assertEqual(self.vio._correct_direction(direction, old), "accepted")
        np.testing.assert_allclose(self.vio.x[:3], 0., atol=1.e-12)


if __name__ == "__main__":
    unittest.main()
