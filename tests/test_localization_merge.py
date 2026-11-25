"""Functional contracts shared by the merged launch, console and estimators."""
import copy
import importlib.util
import json
from pathlib import Path
import unittest
from unittest.mock import Mock

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]


class UniqueLoader(yaml.SafeLoader):
    def construct_mapping(self, node, deep=False):
        keys = [self.construct_object(k, deep=deep) for k, _ in node.value]
        if len(keys) != len(set(keys)):
            raise ValueError('duplicate YAML keys: ' + str(keys))
        return super().construct_mapping(node, deep=deep)


class MergeContracts(unittest.TestCase):
    def settings(self):
        return yaml.load((ROOT/'config/targeting.yaml').read_text(encoding='utf-8-sig'), Loader=UniqueLoader)

    def test_baseline_and_opt_in_experiments(self):
        from evtol_localization.options import options
        from evtol_localization.settings import experiment_config, estimator_module
        base = self.settings()
        default = options(base['localization'])
        self.assertEqual(default['estimator'], 'absolute')
        self.assertEqual((default['fix_profile'], default['fiveg_profile']), ('standalone', 'moderate'))
        self.assertEqual(estimator_module(base), 'evtol_localization.absolute_node')
        for mode in ('gnss_imu', 'gnss_imu_5g', 'camera_imu'):
            value = experiment_config(base, mode, False, False)
            self.assertTrue(estimator_module(value).endswith('vio_node' if mode=='camera_imu' else 'absolute_node'))
        with self.assertRaises(ValueError):
            experiment_config(base, 'camera_imu', True, False)
        with self.assertRaises(ValueError):
            options({'absolute': {'misspelled_setting': 1}})

    def test_sensor_config_is_propagated_and_isolated(self):
        from evtol_simulation.targeting_config import configure
        target = self.settings()
        target['surround']['enabled'] = False
        target['localization'].update(estimator='absolute', fix_profile='standalone')
        sim = {'airsim': {}}
        configure(sim, target)
        self.assertEqual(sim['localization'], target['localization'])
        sim['localization']['enabled'] = False
        self.assertTrue(target['localization']['enabled'])

    def test_launch_and_console_choose_one_identical_estimator(self):
        from evtol_console.processes import node_specs
        from evtol_localization.settings import experiment_config
        from launch import LaunchContext
        from launch.utilities import perform_substitutions
        path = ROOT/'src/evtol_bringup/launch/target_localization.launch.py'
        spec = importlib.util.spec_from_file_location('merged_localization_launch', path)
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        base = self.settings()
        variants = [base] + [experiment_config(base, mode, False, False)
                            for mode in ('gnss_imu', 'gnss_imu_5g', 'camera_imu')]
        for settings in variants:
            # Stub only file I/O; inspect the actual process construction path.
            from unittest.mock import patch
            context = LaunchContext()
            context.launch_configurations.update(config='test.yaml', execute='false', actors='false', evaluate='false')
            with patch.object(module.Path, 'read_text', return_value=yaml.safe_dump(settings)):
                actions = module.processes(context)
            cmds = [[perform_substitutions(context, part) for part in a.cmd]
                    for a in actions if hasattr(a, 'cmd')]
            launch_estimators = [cmd for cmd in cmds if 'evtol_localization.' in str(cmd)
                                 and 'evaluation_node' not in str(cmd)]
            console = node_specs(settings, 'test.yaml', 'sim.yaml', dict(execute=False, actors=False, evaluate=False))
            estimators = [m for n, m, _ in console if n == 'ego_localization']
            self.assertEqual(len(estimators), 1)
            self.assertEqual(len(launch_estimators), 1)
            self.assertIn(estimators[0], str(launch_estimators[0]))
            if settings['localization']['mode'] == 'camera_imu':
                self.assertFalse(any(n == 'ego_localization_evaluation' for n, _, _ in console))

    def test_recording_qos_and_dependencies_have_no_duplicate_keys(self):
        import xml.etree.ElementTree as ET
        qos = yaml.load((ROOT/'src/evtol_bringup/config/bag_qos.yaml').read_text(), Loader=UniqueLoader)
        for topic in ('/simulation/geodetic_reference', '/drone1/gnss/home',
                      '/drone1/localization/odometry', '/drone1/localization/vio/odometry',
                      '/drone1/localization/error_enu', '/drone1/localization/error_3d'):
            self.assertIn(topic, qos)
            self.assertIn(topic, (ROOT/'scripts/record.ps1').read_text(encoding='utf-8-sig'))
        deps = [e.text for e in ET.parse(ROOT/'src/evtol_bringup/package.xml').findall('exec_depend')]
        self.assertEqual(len(deps), len(set(deps)))

    def test_localization_recording_covers_all_three_modes(self):
        from evtol_collection.options import validate
        preset = json.loads((ROOT / 'config/collection.localization.json').read_text())
        topics = set(validate(preset)['bag']['topics'])
        self.assertTrue(preset['bag']['use_sim_time'])
        required = {'/clock', '/simulation/reference_origin', '/drone1/gnss/home',
                    '/drone1/imu/data', '/drone1/gnss/fix', '/drone1/five_ga/observations',
                    '/drone1/ground_truth/local_odometry', '/drone1/camera/rgb/image_raw',
                    '/drone1/camera/rgb/camera_info'}
        self.assertFalse(required - topics, required - topics)

    def test_replay_launch_and_old_entry_use_branch_modules(self):
        from launch import LaunchContext
        from launch.utilities import perform_substitutions
        from unittest.mock import patch
        from evtol_localization import node
        spec = importlib.util.spec_from_file_location('ego_replay_launch',
            ROOT/'src/evtol_bringup/launch/ego_localization_replay.launch.py')
        launch = importlib.util.module_from_spec(spec); spec.loader.exec_module(launch)
        for mode in ('gnss_imu', 'gnss_imu_5g', 'camera_imu'):
            c = self.settings(); c['localization'].update(mode=mode, estimator='legacy')
            expected = 'evtol_localization.' + ('vio_node' if mode=='camera_imu' else 'absolute_node')
            context = LaunchContext(); context.launch_configurations['config'] = 'test.yaml'
            with patch.object(launch.Path, 'read_text', return_value=yaml.safe_dump(c)):
                actions = launch.nodes(context)
            cmds = [[perform_substitutions(context, part) for part in a.cmd] for a in actions]
            self.assertIn(expected, str(cmds))
            self.assertNotIn('evtol_localization.node', str(cmds))
            with patch('evtol_targeting.common.args_config', return_value=(c, [])), patch.object(node, 'import_module') as load:
                node.main(['--config', 'test.yaml'])
                load.assert_called_once_with(expected)

    def test_offline_comparison_runs_the_same_branch_node(self):
        from sensor_msgs.msg import Imu, NavSatFix
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String
        from evtol_perception_interfaces.msg import ReferenceOrigin
        from evtol_sim_interfaces.msg import FiveGObservations, FiveGLink
        from evtol_targeting.common import header, put
        from evtol_localization.absolute import GRAVITY
        spec = importlib.util.spec_from_file_location('same_branch_replay', ROOT/'scripts/compare_localization_bag.py')
        replay = importlib.util.module_from_spec(spec); spec.loader.exec_module(replay)
        start = 1_000_000_000
        home = NavSatFix(header=header(start,'map'),latitude=45.,longitude=120.,altitude=560.)
        home.status.status = 0
        events = [('/simulation/reference_origin', ReferenceOrigin(header=header(start),source_frame='map',session_id='offline'),start),
                  ('/drone1/gnss/home',home,start),
                  ('/drone1/control/flight_status',String(data=json.dumps(dict(session_id='offline',phase='takeoff',takeoff_stamp_ns=start))),start)]
        for i in range(601):
            t = start+i*10_000_000
            imu = Imu(header=header(t)); imu.orientation.w=1.; imu.linear_acceleration.z=GRAVITY
            events.append(('/drone1/imu/data',imu,t))
            if i%10==0:
                fix = copy.deepcopy(home); fix.header=header(t,'drone1/gnss_link')
                fix.position_covariance=np.eye(3).ravel().tolist();fix.position_covariance_type=2
                events.append(('/drone1/gnss/fix',fix,t))
                radio=FiveGObservations(header=header(t,'map'))
                link=FiveGLink(gnb_id='test',measured_range_m=10.,range_sigma_m=.2,prs_rsrp_dbm=-20.,measurement_valid=True)
                put(link.base_station_enu,[10.,0.,0.]);radio.observations.append(link)
                events.append(('/drone1/five_ga/observations',radio,t))
            if i%5==0:
                truth=Odometry(header=header(t));truth.pose.pose.orientation.w=1.
                events.append(('/drone1/ground_truth/local_odometry',truth,t))
        for mode in ('gnss_imu','gnss_imu_5g'):
            result=replay.replay(events,self.settings(),mode,True,2.,4.)
            self.assertEqual(result['estimator'],'evtol_localization.absolute_node')
            self.assertGreater(result['state_hz'],90.)
            self.assertLess(result['summary']['rmse_3d_m'],.001)
            self.assertGreater(result['observations']['counts']['fix/injected_dropout'],10)
            self.assertTrue(result['observations']['recovery_events'])
            if mode=='gnss_imu_5g':
                self.assertGreater(result['observations']['counts']['radio/accepted'],20)

    def test_ui_edits_effective_branch_parameters(self):
        text=(ROOT/'src/evtol_console/evtol_console/web/app.js').read_text(encoding='utf8')
        self.assertNotIn('data-bind="localization.estimator"',text)
        self.assertIn('data-bind="localization.absolute.min_rsrp_dbm"',text)
        self.assertIn('<option value="camera_imu">',text)

    def test_absolute_shared_fault_schedule_path_and_status(self):
        import rclpy
        from sensor_msgs.msg import Imu, NavSatFix
        from std_msgs.msg import String
        from evtol_localization.absolute_node import AbsoluteNode
        from evtol_localization.absolute import GRAVITY
        from evtol_perception_interfaces.msg import ReferenceOrigin
        from evtol_targeting.common import header, stamp
        config = self.settings()
        fault = dict(enabled=True, time_base='flight', default_mode='normal', segments=[
            dict(start_s=.2, end_s=.5, mode='dropout', bias_enu_m=[0., 0., 0.])])
        config['localization'].update(mode='gnss_imu', estimator='absolute', gnss_fault=fault,
                                      absolute={'gnss_outage_s': [0., 100.]})
        rclpy.init(domain_id=214)
        node = AbsoluteNode(config)
        node.pub = Mock(); node.path_pub = Mock(); node.status_pub = Mock()
        try:
            start = 1_000_000_000
            node.on_origin(ReferenceOrigin(header=header(start), source_frame='map', session_id='merged'))
            home = NavSatFix(latitude=45., longitude=120., altitude=560.)
            home.status.status = 0
            node.on_home(home)
            node.on_flight(String(data=json.dumps(dict(session_id='merged', phase='takeoff', takeoff_stamp_ns=start))))
            for i in range(81):
                t = start + i*10_000_000
                node.sim_now = t
                imu = Imu(header=header(t)); imu.orientation.w = 1.; imu.linear_acceleration.z = GRAVITY
                node.on_imu(imu)
                if i % 10 == 0:
                    fix = copy.deepcopy(home); fix.header = header(t)
                    fix.position_covariance = np.eye(3).ravel().tolist(); fix.position_covariance_type = 2
                    node.on_fix(fix)
            node.report()
            self.assertGreater(node.timeline.counts['fix/injected_dropout'], 0)
            self.assertTrue(node.timeline.state.fixed)
            self.assertEqual(len(node.recoveries), 1)
            self.assertGreater(node.pub.publish.call_count, 20)
            self.assertGreater(node.path_pub.publish.call_count, 1)
            status = json.loads(node.status_pub.publish.call_args[0][0].data)
            self.assertEqual(status['session_id'], 'merged')
            self.assertEqual(status['gnss_fault_time_base_ns'], start)
            self.assertEqual(status['gnss_fault_mode'], 'normal')
            self.assertTrue(status['recovery_events'])
        finally:
            node.destroy_node(); rclpy.shutdown()
