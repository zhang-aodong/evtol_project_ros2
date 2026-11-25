"""Only the ego estimator and independent evaluator; bag owns inputs/clock."""
from pathlib import Path
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess,OpaqueFunction
from launch.substitutions import LaunchConfiguration


def nodes(context):
    config=LaunchConfiguration('config').perform(context)
    import yaml
    from evtol_localization.settings import estimator_module
    settings=yaml.safe_load(Path(config).read_text(encoding='utf-8-sig'))
    module=estimator_module(settings)
    specs=[] if module is None else [('ego_localization',module)]
    if module and settings['localization']['mode']!='camera_imu' and settings['localization'].get('evaluate',True):
        specs.append(('ego_localization_evaluation','evtol_localization.evaluation_node'))
    return [ExecuteProcess(name=name,output='both',cmd=[sys.executable,'-u','-m',module,
        '--config',config,'--ros-args','-p','use_sim_time:=true']) for name,module in
        specs]


def generate_launch_description():
    root=Path(get_package_share_directory('evtol_bringup')).parents[2]
    return LaunchDescription([DeclareLaunchArgument('config',default_value=str(root/'config/targeting.yaml')),
        OpaqueFunction(function=nodes)])
