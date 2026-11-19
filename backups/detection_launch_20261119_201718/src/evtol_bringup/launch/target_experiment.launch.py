from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share=Path(get_package_share_directory('evtol_bringup'));root=share.parents[2]
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value=str(root/'config/targeting.yaml')),
        DeclareLaunchArgument('execute',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('actors',default_value='false',choices=['true','false']),
        GroupAction([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/simulation.launch.py')),
            launch_arguments={'config':str(root/'src/evtol_simulation/config/simulation.perception_2k.yaml'),
                'backend':'airsim','scenario':'false','rviz':'false'}.items())]),
        GroupAction([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/target_localization.launch.py')),
            launch_arguments={'config':LaunchConfiguration('config'),'execute':LaunchConfiguration('execute'),
                'actors':LaunchConfiguration('actors')}.items())])])
