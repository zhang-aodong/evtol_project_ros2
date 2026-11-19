import sys
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    root=Path(get_package_share_directory('evtol_bringup')).parents[2]
    return LaunchDescription([DeclareLaunchArgument('config',default_value=str(root/'config/targeting.yaml')),
        ExecuteProcess(name='actors',output='screen',cmd=[sys.executable,'-u','-c',
            'from evtol_scenario.actors import main; main()','--config',LaunchConfiguration('config'),
            '--ros-args','-p','use_sim_time:=true'])])
