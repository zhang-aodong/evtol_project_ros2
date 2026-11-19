"""Configurable actor paths with explicit frame semantics; no RPC dependencies."""
import math
import numpy as np


NED_ENU=np.array([[0.,1.,0.],[1.,0.,0.],[0.,0.,-1.]])


def validate(settings,ego):
    flight=settings['interference_drones']
    if flight.get('initialization','ground') not in ('ground','first_waypoint'):
        raise ValueError('Unknown obstacle initialization mode')
    if not isinstance(flight.get('wait_for_ego_flying',False),bool):
        raise ValueError('wait_for_ego_flying must be boolean')
    names=[]
    for a in settings['interference_drones']['agents']:
        if a['vehicle_name']==ego:raise ValueError('Actors package cannot command the ego vehicle')
        names.append(a['vehicle_name'])
        points=a.get('waypoints_frd_m',a.get('waypoints',[]))
        if not points or np.asarray(points).ndim!=2 or np.asarray(points).shape[1]!=3 or not np.isfinite(points).all():
            raise ValueError('Obstacle vehicle requires finite Nx3 waypoints')
        if not math.isfinite(a['speed_mps']) or a['speed_mps']<=0:raise ValueError('Invalid vehicle speed')
        if a.get('coordinate_mode','relative_heading') not in ('relative_heading','relative_start','initial_enu','absolute_ned','relative_initial'):
            raise ValueError('Unknown vehicle coordinate_mode')
        spawn=np.asarray(a['spawn_offset_ned_m'],dtype=float)
        if spawn.shape!=(3,) or not np.isfinite(spawn).all():raise ValueError('Invalid spawn_offset_ned_m')
        if int(a.get('repeat',0))!=a.get('repeat',0) or a.get('repeat',0)<0:raise ValueError('repeat must be a nonnegative integer')
    if len(set(names))!=len(names):raise ValueError('Duplicate obstacle vehicle names')
    for a in settings['pedestrians']['agents']:
        points=np.asarray(a['waypoints'],dtype=float)
        if points.ndim!=2 or points.shape[1]!=3 or not len(points) or not np.isfinite(points).all():raise ValueError('Invalid pedestrian path')
        if a['speed_mps']<=0 or not math.isfinite(a['speed_mps']):raise ValueError('Invalid pedestrian speed')
        if a['coordinate_mode'] not in ('relative_initial','absolute_ned','initial_enu'):raise ValueError('Unknown pedestrian coordinate mode')
    for a in settings['interference_drones']['agents']+settings['pedestrians']['agents']:
        for name in ('hold_s','start_delay_s'):
            if not math.isfinite(a.get(name,0.)) or a.get(name,0.)<0:raise ValueError('Invalid '+name)
    for a in [settings['interference_drones']]+settings['interference_drones']['agents']:
        value=a.get('waypoint_tolerance_m',.5)
        if not math.isfinite(value) or value<=0:raise ValueError('Invalid waypoint_tolerance_m')
    for name in ('rpc_timeout_s','takeoff_timeout_s','move_timeout_s'):
        value=settings['interference_drones'].get(name,1.)
        if not math.isfinite(value) or value<=0:raise ValueError('Invalid '+name)
    hz=settings['pedestrians']['update_rate_hz']
    if not math.isfinite(hz) or hz<=0:raise ValueError('Invalid actor update rate')


def vehicle_path(agent,ego_origin_ned,ego_rotation_ned,actor_initial_ned):
    mode=agent.get('coordinate_mode','relative_heading')
    points=np.asarray(agent.get('waypoints_frd_m',agent.get('waypoints')),dtype=float)
    spawn=np.asarray(agent['spawn_offset_ned_m'],dtype=float)
    if mode=='relative_heading':return points@ego_rotation_ned.T+ego_origin_ned-spawn
    if mode=='relative_start':return points+ego_origin_ned-spawn
    if mode=='initial_enu':return points@NED_ENU.T+ego_origin_ned-spawn
    if mode=='relative_initial':return points+actor_initial_ned
    if mode=='absolute_ned':return points
    raise ValueError('Unknown vehicle coordinate mode')


def pedestrian_path(agent,initial_ned,ego_origin_ned):
    points=np.asarray(agent['waypoints'],dtype=float)
    if agent['coordinate_mode']=='relative_initial':return points+initial_ned
    if agent['coordinate_mode']=='initial_enu':return points@NED_ENU.T+ego_origin_ned
    if agent['coordinate_mode']=='absolute_ned':return points
    raise ValueError('Unknown pedestrian coordinate mode')


def advance(current,target,distance):
    delta=target-current;length=float(np.linalg.norm(delta))
    return (target.copy(),True) if length<=distance else (current+delta*(distance/length),False)
