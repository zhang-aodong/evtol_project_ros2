"""Validate console drafts and generate isolated, reproducible run settings."""
import copy
import json
from pathlib import Path
import math
import yaml
from evtol_targeting.configuration import validate_config
from evtol_targeting.flight_profile import compile_profile,finite
from evtol_scenario.paths import validate as validate_actors


def read_yaml(path):return yaml.safe_load(Path(path).read_text(encoding='utf-8-sig'))


def atomic(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf8')
    temporary.replace(path)


def defaults(root):
    root=Path(root);base=read_yaml(root/'config/targeting.yaml')
    actor=read_yaml(root/base['actors_config'])
    machine=json.loads((root/'config/machine.local.json').read_text(encoding='utf-8-sig')) if (root/'config/machine.local.json').exists() else {}
    agents=[]
    for index,a in enumerate(actor['interference_drones']['agents']):
        spawn=a['spawn_offset_ned_m']
        # Existing legacy routes are FRD/NED; supply explicit ENU defaults rather
        # than silently presenting them as ENU values.
        agents.append(dict(name=a['vehicle_name'],speed_mps=a['speed_mps'],spawn_ned_m=spawn,
            waypoints_enu_m=[[spawn[1],spawn[0],10.],[spawn[1]+10.,spawn[0],10.]],repeat=1))
    return dict(version=1,ue=dict(editor=machine.get('UnrealEditor',''),project=machine.get('UnrealProject',''),
        settings='src/evtol_simulation/config/airsim.navigation_2k.json',visible=True,width=1280,height=720),
        actors=dict(count=len(agents),agents=agents,pedestrians_enabled=bool(actor['pedestrians']['enabled'])),
        mission=dict(mode='waypoints',speed_mps=base['mission']['speed_mps'],waypoints_enu_m=base['mission']['waypoints_enu_m'],
            land_at_last=base['mission'].get('land_at_last',True),profile=dict(heading_deg=0.,points=[
                dict(speed_mps=0.,height_m=0.,distance_m=100.),dict(speed_mps=2.,height_m=10.,distance_m=100.),
                dict(speed_mps=2.,height_m=10.,distance_m=30.),dict(speed_mps=0.,height_m=0.,distance_m=100.)]),
            max_speed_mps=base['control']['max_speed_mps'],max_acceleration_mps2=base['safety']['local_velocity']['max_acceleration_mps2']),
        run=dict(execute=False,actors=True,evaluate=True,new_session=True))


def points(value,name,maximum=1000):
    import numpy as np
    try:a=np.asarray(value,dtype=float)
    except (TypeError,ValueError):raise ValueError(name+' must be a list of ENU triples')
    if a.ndim!=2 or a.shape[1]!=3 or not 1<=len(a)<=maximum or not np.isfinite(a).all() or np.any(abs(a)>100000):
        raise ValueError(name+' must contain finite [east,north,up] triples')
    return a.tolist()


def validate(draft,root):
    d=copy.deepcopy(draft)
    if d.get('version')!=1:raise ValueError('Unsupported console configuration version')
    ue=d['ue'];mission=d['mission'];actors=d['actors']
    for key in ('editor','project','settings'):
        if not isinstance(ue[key],str) or not ue[key].strip():raise ValueError('UE '+key+' is required')
        if any(x in ue[key] for x in ('\0','\r','\n')):raise ValueError('Invalid UE path')
    for key,lo,hi in [('width',320,7680),('height',240,4320)]:
        if type(ue[key]) is not int or not lo<=ue[key]<=hi:raise ValueError('Invalid window '+key)
    if not isinstance(ue['visible'],bool):raise ValueError('visible must be boolean')
    for flag in ('execute','actors','evaluate','new_session'):
        if not isinstance(d['run'][flag],bool):raise ValueError('Invalid run flag '+flag)
    if not isinstance(actors['pedestrians_enabled'],bool):raise ValueError('Invalid pedestrians_enabled')
    count=actors['count']
    if type(count) is not int or not 0<=count<=32:raise ValueError('Obstacle count must be 0..32')
    if not isinstance(actors['agents'],list) or len(actors['agents'])!=count:raise ValueError('Obstacle count does not match agents')
    base=read_yaml(Path(root)/'config/targeting.yaml');ego=base['airsim']['vehicle'];names=[]
    for a in actors['agents']:
        name=a['name'];names.append(name)
        if not isinstance(name,str) or not name or name==ego or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in name):
            raise ValueError('Invalid obstacle vehicle name')
        finite(a['speed_mps'],'obstacle speed',.05,30.)
        a['waypoints_enu_m']=points(a['waypoints_enu_m'],'obstacle waypoints')
        if any(p[2]<0 for p in a['waypoints_enu_m']):raise ValueError('Obstacle heights must be nonnegative ENU')
        a['spawn_ned_m']=points([a['spawn_ned_m']],'spawn')[0]
        if type(a['repeat']) is not int or not 0<=a['repeat']<=10000:raise ValueError('Invalid obstacle repeat count')
    if len(names)!=len(set(names)):raise ValueError('Duplicate obstacle vehicle names')
    cap=finite(mission['max_speed_mps'],'max speed',.1,30.)
    acceleration=finite(mission['max_acceleration_mps2'],'max acceleration',.05,5.)
    speed=finite(mission['speed_mps'],'mission speed',.05,cap)
    if not isinstance(mission['land_at_last'],bool):raise ValueError('land_at_last must be boolean')
    if mission['mode']=='waypoints':
        route=points(mission['waypoints_enu_m'],'ego waypoints')
        if any(p[2]<0 for p in route):raise ValueError('Ego heights must be nonnegative ENU')
        compiled=None
    elif mission['mode']=='profile':
        compiled=compile_profile(mission['profile'],cap,acceleration);route=compiled['waypoints_enu_m']
        if max(max(x['start_speed_mps'],x['end_speed_mps']) for x in compiled['legs'])>cap:raise ValueError('Profile exceeds speed cap')
    else:raise ValueError('Unknown flight mode')
    return d,route,compiled


def artifacts(draft,root):
    root=Path(root);d,route,compiled=validate(draft,root)
    base=read_yaml(root/'config/targeting.yaml');scenario=read_yaml(root/base['actors_config'])
    settings_path=Path(d['ue']['settings']);settings_path=settings_path if settings_path.is_absolute() else root/settings_path
    ue=json.loads(settings_path.read_text(encoding='utf-8-sig'))
    ego=base['airsim']['vehicle'];vehicles=ue['Vehicles']
    if ego not in vehicles:raise ValueError('UE settings do not define ego '+ego)
    # Explicit templates only: never give new obstacle drones ego sensor rings.
    for name in list(vehicles):
        if name!=ego:del vehicles[name]
    for a in d['actors']['agents']:
        n,e,z=a['spawn_ned_m']
        vehicles[a['name']]=dict(VehicleType='SimpleFlight',X=n,Y=e,Z=z)
    if d['ue']['visible'] and ue.get('ViewMode')=='NoDisplay':ue['ViewMode']='SpringArmChase'
    scenario['interference_drones']['enabled']=d['actors']['count']>0
    scenario['interference_drones']['agents']=[dict(vehicle_name=a['name'],spawn_offset_ned_m=a['spawn_ned_m'],
        coordinate_mode='initial_enu',speed_mps=a['speed_mps'],repeat=a['repeat'],waypoints=a['waypoints_enu_m']) for a in d['actors']['agents']]
    scenario['pedestrians']['enabled']=d['actors']['pedestrians_enabled']
    validate_actors(scenario,ego)
    base['mission'].update(mode=d['mission']['mode'],speed_mps=d['mission']['speed_mps'],waypoints_enu_m=route,
        land_at_last=d['mission']['land_at_last'],profile=d['mission']['profile'],
        profile_speed_cap_mps=d['mission']['max_speed_mps'],profile_acceleration_cap_mps2=d['mission']['max_acceleration_mps2'])
    base['control']['max_speed_mps']=d['mission']['max_speed_mps']
    local=base['safety']['local_velocity'];local['max_speed_mps']=d['mission']['max_speed_mps'];local['max_acceleration_mps2']=d['mission']['max_acceleration_mps2']
    # Preserve the existing sensing/braking envelope. Reject unachievable
    # speed requests rather than quietly increasing the collision horizon.
    from evtol_safety.velocity import LocalVelocityAvoidance
    envelope=LocalVelocityAvoidance(local,base['safety']['cloud_timeout_s'],base['control']['duration_s'],0.).speed_limit
    requested=max([d['mission']['speed_mps']] if compiled is None else [p['speed_mps'] for p in compiled['points']])
    if requested>envelope+.001:raise ValueError(f'Requested {requested:g} m/s exceeds the current sensor/braking envelope {envelope:.2f} m/s')
    validate_config(base)
    return d,base,scenario,ue,compiled
