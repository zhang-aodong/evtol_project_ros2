"""ESKF geometry, delayed observations, quality gates and ROS message contracts."""
from pathlib import Path
from importlib.util import find_spec
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for package in ('evtol_localization', 'evtol_simulation', 'evtol_targeting'):
    sys.path.insert(0, str(ROOT / 'src' / package))

from evtol_localization.absolute import (
    GRAVITY, ImuSample, RangeLink, SessionMap, Settings, Timeline,
)

T = 1_000_000_000
MS = 1_000_000
STILL = ImuSample([0, 0, 0], [0, 0, GRAVITY], [0, 0, 0, 1])
FIX = (np.zeros(3), np.eye(3) * .1)


class AbsoluteTests(unittest.TestCase):
    def test_static_launch_imu_bias_and_covariance(self):
        tl = Timeline(Settings(stationary_start=True))
        sample = ImuSample([.005, -.003, 0], [.03, -.02, GRAVITY + .04], [0, 0, 0, 1])
        for i in range(101):
            t = T + 10 * MS * i
            tl.add('imu', t, sample)
            if i % 10 == 0:
                tl.add('fix', t, FIX)
        s = tl.state
        self.assertTrue(s.fixed)
        np.testing.assert_allclose(s.vel, 0., atol=1e-10)
        np.testing.assert_allclose(s.pos, 0., atol=1e-10)
        np.testing.assert_allclose(s.bg, [.005, -.003, 0], atol=1e-12)
        np.testing.assert_allclose(s.ba, [.03, -.02, .04])
        np.testing.assert_allclose(s.P, s.P.T)
        self.assertGreater(np.linalg.eigvalsh(s.P).min(), 0.)
        self.assertGreater(tl.counts['fix/waiting_static_window'], 0)

    def test_north_forward_body_velocity_and_lever_arms(self):
        # Body +x points north. Body +y points west.
        tl = Timeline(Settings(gnss_lever_flu_m=[1, 0, 0], radio_lever_flu_m=[0, 1, 0]))
        heading = [0., 0., np.sqrt(.5), np.sqrt(.5)]
        forward = ImuSample([0, 0, 0], [2, 0, GRAVITY], heading)
        tl.add('imu', T, forward)
        tl.add('fix', T, (np.array([0, 1, 0]), np.diag([.5, 1., 1.])))
        np.testing.assert_allclose(tl.state.pos, 0, atol=1e-12)
        for i in range(1, 101):
            tl.add('imu', T + 10 * MS * i, forward)
        s = tl.state
        np.testing.assert_allclose(s.pos, [0, 1, 0], atol=1e-10)
        np.testing.assert_allclose(s.vel, [0, 2, 0], atol=1e-10)
        linear, angular, pose, twist = s.output(.02)
        np.testing.assert_allclose(linear, [2, 0, 0], atol=1e-10)
        np.testing.assert_allclose(angular, 0, atol=1e-10)
        self.assertGreater(np.linalg.eigvalsh(pose).min(), 0)
        self.assertGreater(np.linalg.eigvalsh(twist).min(), 0)
        rotated_antenna = np.array([-1., 1., 0.])
        anchors = [np.array([2., 4., 3.]), np.array([0., -2., 5.])]
        links = [RangeLink(str(i), a, np.linalg.norm(rotated_antenna - a), .4, -80, True)
                 for i, a in enumerate(anchors)]
        tl.add('radio', s.t, links)
        np.testing.assert_allclose(tl.state.pos, s.pos, atol=1e-10)
        self.assertEqual(tl.counts['radio/accepted'], 2)

    def test_nonzero_session_origin_and_WGS84_translation(self):
        origin = np.array([105., -36., 14.])
        frame = SessionMap([45, 120, 500], origin)
        at_home, cov = frame.fix([45, 120, 500], np.diag([1., 2., 4.]))
        np.testing.assert_allclose(at_home, -origin)
        np.testing.assert_allclose(cov, np.diag([1, 2, 4]), atol=1e-12)
        above, _ = frame.fix([45, 120, 502], np.eye(3))
        np.testing.assert_allclose(above - at_home, [0, 0, 2], atol=1e-7)
        east, _ = frame.fix([45, 120.00001, 500], np.eye(3))
        self.assertAlmostEqual(east[0] - at_home[0], .7885, places=3)
        anchor = np.array([106., -42., 82.])
        self.assertAlmostEqual(np.linalg.norm(above - frame.station(anchor)),
                               np.linalg.norm([anchor[0], anchor[1], anchor[2] - 2]))

    def test_replay_preserves_all_inputs_and_rejects_old_or_duplicate(self):
        events = []
        for i in range(101):
            t = T + i * 10 * MS
            events.append((t, 'imu', t + 100 * MS, STILL))
            if i % 10 == 0:
                events.append((t, 'fix', t + 200 * MS,
                               (np.array([.003 * i, 0., 0.]), np.eye(3) * .25)))
            if i % 10 == 5:
                link = RangeLink('A', np.array([10., 0., 0.]), 10. - .003 * i, .4, -77., True)
                events.append((t, 'radio', t + 8 * MS, [link]))
        results = []
        for order in (lambda e: (e[0], Timeline.PRIORITY[e[1]]), lambda e: e[2]):
            tl = Timeline(Settings())
            for t, kind, _, sample in sorted(events, key=order):
                tl.add(kind, t, sample)
            results.append(tl)
        a, b = results
        np.testing.assert_allclose(a.state.pos, b.state.pos, atol=1e-12)
        np.testing.assert_allclose(a.state.P, b.state.P, atol=1e-12)
        self.assertEqual(a.counts, b.counts)
        self.assertEqual(a.counts['fix/accepted'], 11)
        self.assertEqual(a.counts['radio/accepted'], 10)
        self.assertFalse(b.add('fix', b.state.t, FIX))
        self.assertFalse(b.add('fix', T, FIX))
        self.assertFalse(b.add('radio', b.state.t + T, []))
        self.assertLess(len(b.keys), 70)

    def test_quality_gates_and_gnss_outage_recovery(self):
        tl = Timeline(Settings(gnss_outage_s=[.1, .3]))
        tl.add('imu', T, STILL)
        tl.add('fix', T, FIX)
        base = np.array([10., 0., 0.])
        links = [RangeLink('ok', base, 10.1, .4, -80, True),
                 RangeLink('weak', base, 10, 1, -140, True),
                 RangeLink('flag', base, 10, 1, -70, False),
                 RangeLink('large', base, 80, .5, -75, True),
                 RangeLink('sigma', base, 10, np.nan, -65, True)]
        for i in range(1, 41):
            t = T + i * 10 * MS
            tl.add('imu', t, STILL)
            if i in (10, 20, 30):
                tl.add('fix', t, FIX)
            if i == 15:
                tl.add('radio', t, links + [links[0]])
            if i == 35:
                tl.add('fix', t, (np.ones(3) * 1e4, np.eye(3)))
            if i == 40:
                tl.add('fix', t, (np.ones(3), np.diag([1., 1., -1.])))
        self.assertEqual(tl.counts['fix/masked'], 2)
        self.assertEqual(tl.state.last_fix, T + 300 * MS)
        self.assertEqual(tl.counts['fix/innovation'], 1)
        self.assertEqual(tl.counts['fix/covariance'], 1)
        self.assertEqual(tl.counts['station/ok/accepted'], 1)
        for id_, why in [('weak', 'weak_signal'), ('flag', 'invalid'), ('large', 'innovation'),
                         ('sigma', 'invalid_geometry_or_sigma'), ('ok', 'duplicate_or_missing_id')]:
            self.assertEqual(tl.counts['station/' + id_ + '/' + why], 1)

    def test_no_extrapolation_over_gap_or_uninitialized_attitude(self):
        tl = Timeline(Settings())
        tl.add('imu', T, ImuSample([0, 0, 0], [0, 0, GRAVITY]))
        tl.add('fix', T, FIX)
        self.assertFalse(tl.state.fixed)
        tl.add('imu', T + 10 * MS, STILL)
        tl.add('fix', T + 10 * MS, FIX)
        self.assertTrue(tl.state.fixed)
        tl.add('imu', T + 350 * MS, STILL)
        self.assertFalse(tl.state.fixed)
        self.assertEqual(tl.counts['imu/gap_reset'], 1)
        tl.add('fix', T + 350 * MS, FIX)
        self.assertTrue(tl.state.fixed)


@unittest.skipUnless(find_spec('rclpy'), 'requires built ROS 2 workspace')
class AbsoluteNodeTests(unittest.TestCase):
    def test_node_session_reset_and_measurement_epoch(self):
        import rclpy
        from sensor_msgs.msg import Imu, NavSatFix
        from rosgraph_msgs.msg import Clock
        from evtol_perception_interfaces.msg import ReferenceOrigin
        from evtol_localization.absolute_node import AbsoluteNode
        from evtol_targeting.common import header, put, ns, stamp

        rclpy.init(domain_id=213)
        config = dict(namespace='drone1', localization={'mode': 'gnss_imu'},
                      state={'timeout_s': .3, 'fused_topic': '/drone1/localization/odometry'})
        node = AbsoluteNode(config)
        published = []
        from unittest.mock import Mock
        node.pub = Mock()
        node.pub.publish.side_effect = published.append
        try:
            origin = ReferenceOrigin(header=header(T), source_frame='map', session_id='session1')
            put(origin.source_origin_enu, [4, 6, 10])
            node.on_origin(origin)
            home = NavSatFix(latitude=45., longitude=120., altitude=560.)
            home.status.status = home.status.STATUS_FIX
            node.on_home(home)
            imu = Imu(header=header(T + 4 * MS, 'drone1/imu_link'))
            imu.orientation.w = 1.
            imu.linear_acceleration.z = GRAVITY
            node.on_imu(imu)
            fix = NavSatFix(header=header(T + 5 * MS, 'drone1/gnss_link'),
                            latitude=45., longitude=120., altitude=570.)
            fix.status.status = fix.status.STATUS_FIX
            node.on_fix(fix)
            imu.header.stamp = stamp(T + 15 * MS)
            node.on_imu(imu)
            self.assertEqual(len(published), 1)
            odom = published[-1]
            self.assertEqual(ns(odom.header.stamp), T + 15 * MS)
            self.assertEqual((odom.header.frame_id, odom.child_frame_id), ('map_enu', 'drone1/base_link'))
            self.assertAlmostEqual(odom.pose.pose.position.x, -4., places=6)
            self.assertAlmostEqual(odom.pose.pose.position.y, -6., places=6)
            self.assertAlmostEqual(odom.pose.pose.position.z, 0., places=6)
            self.assertEqual(len(odom.twist.covariance), 36)
            node.on_clock(Clock(clock=stamp(T + 500 * MS)))
            imu.header.stamp = stamp(T + 25 * MS)
            node.on_imu(imu)
            self.assertEqual(node.timeline.counts['output/stale'], 1)
            self.assertEqual(len(published), 1)
            origin.session_id = 'session2'
            origin.header.stamp = stamp(T + 20 * MS)
            node.on_origin(origin)
            self.assertFalse(node.timeline.state.fixed)
            node.on_fix(fix)
            self.assertEqual(node.timeline.counts['fix/before_session'], 1)
            node.on_clock(Clock(clock=stamp(T - MS)))
            self.assertIsNone(node.frame)
            self.assertEqual(node.last_reset, 'simulation_clock_rewind')
            self.assertEqual(len(published), 1)
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
