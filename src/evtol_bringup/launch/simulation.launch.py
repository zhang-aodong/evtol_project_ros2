from pathlib import Path
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent, ExecuteProcess
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    config=LaunchConfiguration('config').perform(context)
    backend=LaunchConfiguration('backend').perform(context)
    processes=[]
    for kind in ('truth','camera','lidar','imu','gnss','radio'):
        # Windows console-script EXEs spawn another Python process. Terminating
        # the EXE can orphan the actual publisher and its sensor mutex. Launch
        # the active ROS interpreter directly so launch owns the real process.
        node=ExecuteProcess(name=kind+'_sensor',output='both',
            cmd=[sys.executable,'-u','-X','faulthandler','-c',
                 'from evtol_simulation.sensor_node import main; main()',
                 '--config',config,'--kind',kind,'--backend',backend,
                 '--ros-args','-r','__node:='+kind+'_sensor'])
        processes.append(RegisterEventHandler(OnProcessExit(target_action=node,on_exit=[
            EmitEvent(event=Shutdown(reason=kind+' sensor process exited'))])))
        processes.append(node)
    if backend=='airsim':
        processes.append(Node(package='evtol_scenario',executable='scenario_node',output='screen',
            condition=IfCondition(LaunchConfiguration('scenario')),arguments=['--config',config]))
    return processes


def generate_launch_description():
    sim=Path(get_package_share_directory('evtol_simulation'))
    bringup=Path(get_package_share_directory('evtol_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value=str(sim/'config'/'simulation.yaml')),
        DeclareLaunchArgument('backend',default_value='airsim',choices=['airsim','mock']),
        DeclareLaunchArgument('scenario',default_value='false'),
        DeclareLaunchArgument('rviz',default_value='false'),
        OpaqueFunction(function=nodes),
        Node(package='rviz2',executable='rviz2',output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             parameters=[{'use_sim_time':True}],arguments=['-d',str(bringup/'rviz'/'simulation.rviz')])])
