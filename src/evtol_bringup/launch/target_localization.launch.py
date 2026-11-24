"""Algorithm stack only: an existing simulation owns all raw sensor streams."""
from pathlib import Path
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess,RegisterEventHandler,EmitEvent,OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


def processes(context):
    config=LaunchConfiguration('config').perform(context)
    execute=LaunchConfiguration('execute').perform(context).lower()=='true'
    actors=LaunchConfiguration('actors').perform(context).lower()=='true'
    evaluate=LaunchConfiguration('evaluate').perform(context).lower()=='true'
    specs=[('state','evtol_targeting.state_node',[]),('trajectory','evtol_targeting.trajectory_node',[]),
        ('vision','evtol_targeting.vision_node',[]),
        ('localizer','evtol_targeting.localizer_node',[]),('safety','evtol_safety.node',[]),
        ('gateway','evtol_safety.gateway',['--execute'] if execute else [])]
    import yaml
    settings=yaml.safe_load(Path(config).read_text(encoding='utf-8-sig'))
    if settings.get('localization', {}).get('enabled'):
        from evtol_localization.settings import estimator_module
        specs.append(('ego_localization', estimator_module(settings), []))
    if settings.get('surround',{}).get('enabled',False):
        detect=settings['model'].get('task')=='detect'
        if detect or not settings['surround'].get('front_diagnostic_inference',False):specs=[s for s in specs if s[0]!='vision']
        fusion='evtol_targeting.detection_fusion' if detect else 'evtol_targeting.surround_fusion'
        specs=[(n,fusion if n=='localizer' else m,x) for n,m,x in specs]
        specs.insert(0,('perception_lidar','evtol_simulation.perception_lidar',[]))
        if settings['surround'].get('roi_lidar',False):specs.insert(0,('roi_lidar','evtol_simulation.roi_lidar',[]))
        specs.insert(next(i for i,s in enumerate(specs) if s[0]=='localizer'),('surround_vision','evtol_targeting.surround_vision',[]))
    if settings.get('ue_detection_boxes',{}).get('enabled',True):
        specs.insert(2,('ue_boxes','evtol_targeting.ue_detection_boxes',[]))
    if settings.get('navigation_lidar',{}).get('enabled',False):
        specs.insert(0,('navigation_lidar','evtol_simulation.safety_lidar',[]))
        if settings.get('lidar_obstacles',{}).get('enabled',True):
            specs.insert(next(i for i,s in enumerate(specs) if s[0]=='safety'),('lidar_obstacles','evtol_targeting.lidar_obstacles',[]))
    if actors:specs.append(('actors','evtol_scenario.actors',[]))
    if evaluate:specs.extend([('target_truth','evtol_targeting.evaluation',['--truth']),('evaluation','evtol_targeting.evaluation',[]),('range_evaluation','evtol_targeting.experiment_node',[])])
    actions=[]
    for name,module,extra in specs:
        node=ExecuteProcess(name=name,output='both',cmd=[sys.executable,'-u','-X','faulthandler','-c',
            f'from {module} import main; main()','--config',config]+extra+['--ros-args','-p','use_sim_time:=true'])
        actions.append(RegisterEventHandler(OnProcessExit(target_action=node,on_exit=[EmitEvent(event=Shutdown(reason=name+' exited'))])))
        actions.append(node)
    return actions


def generate_launch_description():
    root=Path(get_package_share_directory('evtol_bringup')).parents[2]
    return LaunchDescription([DeclareLaunchArgument('config',default_value=str(root/'config/targeting.yaml')),
        DeclareLaunchArgument('execute',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('actors',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('evaluate',default_value='true',choices=['true','false']),OpaqueFunction(function=processes)])
