"""The direct launch entry must start the shared-memory camera producers."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
import yaml
from evtol_simulation.configuration import load

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('target_experiment', ROOT/'src/evtol_bringup/launch/target_experiment.launch.py')
launch_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch_module)


class DetectionLaunchTests(unittest.TestCase):
    def test_localization_launch_builds_real_processes(self):
        from launch import LaunchContext
        from launch.actions import ExecuteProcess
        spec=importlib.util.spec_from_file_location('ego_launch_integration',ROOT/'src/evtol_bringup/launch/target_localization.launch.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        context=LaunchContext();context.launch_configurations.update(config=str(ROOT/'config/targeting.yaml'),execute='false',actors='false',evaluate='false')
        actions=module.processes(context)
        self.assertGreaterEqual(sum(isinstance(a,ExecuteProcess) for a in actions),10)
    def generated(self, enabled):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        folder = root/'src/evtol_simulation/config'
        folder.mkdir(parents=True)
        for name in ('simulation.perception_2k.yaml', 'source_parameters.yaml'):
            (folder/name).write_bytes((ROOT/'src/evtol_simulation/config'/name).read_bytes())
        c = yaml.safe_load((ROOT/'config/targeting.yaml').read_text(encoding='utf-8-sig'))
        c['namespace'] = 'custom_drone'
        c['airsim'].update(host='192.0.2.1', port=41452, vehicle='TestDrone')
        c['surround']['enabled'] = enabled
        c['surround']['record_truth'] = True
        c['surround']['state_sample_hz'] = 60.
        path = root/'targeting.yaml'
        path.write_text(yaml.safe_dump(c), encoding='utf8')
        before = (folder/'simulation.perception_2k.yaml').read_bytes()
        result = launch_module.sensor_config(root, path)
        self.assertEqual(before, (folder/'simulation.perception_2k.yaml').read_bytes())
        loaded = load(result)
        self.assertEqual(loaded['namespace'], 'custom_drone')
        self.assertEqual(loaded['airsim']['vehicle'], 'TestDrone')
        self.assertEqual(loaded['airsim']['port'], 41452)
        self.assertEqual(loaded['airsim']['host'], '192.0.2.1')
        self.assertTrue(loaded['source'])
        return loaded, c

    def test_detection_launch_starts_all_configured_cameras_and_native_pose(self):
        result, c = self.generated(True)
        self.assertEqual(result['camera']['surround_cameras'], [v['name'] for v in c['surround']['cameras']])
        self.assertTrue(result['native_kinematics'])
        self.assertEqual(result['rates']['truth'], 60.)
        self.assertTrue(result['camera']['surround_record_truth'])

    def test_disabled_surround_does_not_start_ring_cameras(self):
        result, _ = self.generated(False)
        self.assertFalse(result['camera'].get('surround_cameras'))
        self.assertFalse(result.get('native_kinematics', False))


if __name__ == '__main__':
    unittest.main()
