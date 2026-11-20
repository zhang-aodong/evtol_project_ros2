"""Start acquisition and algorithms using the same targeting configuration."""
from pathlib import Path
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def sensor_config(root, targeting_path):
    root = Path(root).resolve()
    settings = yaml.safe_load(Path(targeting_path).read_text(encoding='utf-8-sig'))
    source = root / 'src/evtol_simulation/config/simulation.perception_2k.yaml'
    simulation = yaml.safe_load(source.read_text(encoding='utf-8-sig'))
    simulation['source_parameters'] = str((source.parent / simulation['source_parameters']).resolve())
    simulation['namespace'] = settings['namespace']
    simulation['airsim'].update(settings['airsim'])
    surround = settings.get('surround', {})
    if surround.get('enabled', False):
        simulation['native_kinematics'] = True
        simulation['rates']['truth'] = float(surround.get('state_sample_hz', 50.))
        simulation['camera']['surround_cameras'] = [v['name'] for v in surround['cameras']]
        simulation['camera']['surround_record_truth'] = bool(surround.get('record_truth', False))
    parent = root / 'data/launch_runs'
    from evtol_simulation.targeting_config import configure
    configure(simulation,settings)
    parent.mkdir(parents=True, exist_ok=True)
    folder = Path(tempfile.mkdtemp(prefix='experiment_', dir=str(parent)))
    path = folder / 'simulation.yaml'
    path.write_text(yaml.safe_dump(simulation, allow_unicode=True, sort_keys=False), encoding='utf8')
    return path


def experiment(context):
    share = Path(get_package_share_directory('evtol_bringup'))
    config = str(Path(LaunchConfiguration('config').perform(context)).resolve())
    simulation = sensor_config(share.parents[2], config)
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/simulation.launch.py')),
            launch_arguments={'config':str(simulation), 'backend':'airsim',
                              'scenario':'false', 'rviz':'false'}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/target_localization.launch.py')),
            launch_arguments={'config':config, 'execute':LaunchConfiguration('execute'),
                              'actors':LaunchConfiguration('actors')}.items())]


def generate_launch_description():
    root = Path(get_package_share_directory('evtol_bringup')).parents[2]
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(root/'config/targeting.yaml')),
        DeclareLaunchArgument('execute', default_value='false', choices=['true','false']),
        DeclareLaunchArgument('actors', default_value='false', choices=['true','false']),
        OpaqueFunction(function=experiment)])
