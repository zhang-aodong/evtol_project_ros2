"""Reject invalid timing, geometry and control budgets before node startup."""
import math
import numpy as np


def validate_config(c):
    from .ue_box_geometry import options as ue_box_options
    ue_box_options(c)
    positive={
        'state':['timeout_s'],
        'model':['imgsz','max_hz','timeout_s','startup_timeout_s'],
        'fusion':['max_sync_delta_ms','wait_timeout_s','pose_max_gap_s'],
        'detection':['min_lidar_points','min_mask_pixels','cluster_radius_m'],
        'tracking':['max_age_s','association_distance_m','measurement_std_m','acceleration_std_mps2'],
        'mission':['speed_mps','tolerance_m'],
        'safety':['rate_hz','cloud_timeout_s','track_timeout_s','max_points','horizon_s','dynamic_clearance_m'],
        'control':['rate_hz','ttl_s','duration_s','rpc_timeout_s','max_speed_mps'],
        'evaluation':['truth_hz','association_gate_m']}
    for section,fields in positive.items():
        for name in fields:
            value=c[section][name]
            if not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
                raise ValueError(f'{section}.{name} must be finite and positive')
    for name,value in c['safety']['local_velocity'].items():
        if not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
            raise ValueError('Invalid safety.local_velocity.'+name)
    for name in ('camera','lidar'):
        for field in ('xyz_frd_m','rpy_deg'):
            vector=np.asarray(c['extrinsics'][name][field],dtype=float)
            if vector.shape!=(3,) or not np.isfinite(vector).all():raise ValueError('Invalid extrinsics.'+name+'.'+field)
    mode=c['mission'].get('mode','waypoints')
    if mode not in ('waypoints','profile'):raise ValueError('Unknown mission.mode')
    if mode=='profile':
        from .flight_profile import compile_profile
        result=compile_profile(c['mission']['profile'],c['control']['max_speed_mps'],c['safety']['local_velocity']['max_acceleration_mps2'])
        if not np.allclose(np.asarray(result['waypoints_enu_m']),np.asarray(c['mission']['waypoints_enu_m'])):
            raise ValueError('Profile waypoints must match the compiled speed-height profile')
        first=result['points'][0]['height_m']
        if .05<first<c['mission'].get('takeoff_height_m',1.5):raise ValueError('First profile height must be ground or at least the takeoff height')
        if first<.05 and (result['legs'][0]['kind']!='vertical' or result['points'][1]['height_m']<c['mission'].get('takeoff_height_m',1.5)):
            raise ValueError('A ground-start profile must first climb vertically to the takeoff height')
    goals=np.asarray(c['mission']['waypoints_enu_m'],dtype=float)
    if goals.ndim!=2 or goals.shape[1]!=3 or not len(goals) or not np.isfinite(goals).all():
        raise ValueError('mission.waypoints_enu_m must be finite Nx3')
    trajectory=c.get('trajectory',{})
    for field,default in (('publish_hz',2.),('min_distance_m',.05)):
        value=trajectory.get(field,default)
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
            raise ValueError('Invalid trajectory.'+field)
    count=trajectory.get('max_points',20000)
    if isinstance(count,bool) or not isinstance(count,int) or count<4:
        raise ValueError('trajectory.max_points must be an integer >= 4')
    if c['state']['source'] not in ('truth','fused'):raise ValueError('Unknown state.source')
    if c['evaluation']['vehicle_truth_frame'] not in ('vehicle_local','world_ned'):raise ValueError('Unknown evaluation.vehicle_truth_frame')
    if any(v not in range(7) for v in c['detection']['target_class_ids']):raise ValueError('Unknown semantic target class')
    for name,maximum in [('observed_half_yaw_deg',90),('observed_half_pitch_deg',15)]:
        value=c['safety'][name]
        if not math.isfinite(value) or not 0<value<=maximum:raise ValueError('Coverage exceeds configured source lidar: '+name)
    if c['control']['duration_s']>c['control']['ttl_s']:raise ValueError('Command duration exceeds TTL')
    cap=c['control']['max_speed_mps']
    if max(c['mission']['speed_mps'],c['safety']['local_velocity']['max_speed_mps'])>cap:
        raise ValueError('Mission/planner speed exceeds gateway limit')
    gains=c['control'].get('velocity_gains')
    if gains:
        for name in ('kp','ki','kd'):
            value=np.asarray(gains[name],dtype=float)
            if value.shape!=(3,) or not np.isfinite(value).all() or np.any(value<0):raise ValueError('Invalid control.velocity_gains.'+name)
    if c.get('navigation_lidar',{}).get('enabled',False):
        for section,fields in {'navigation_lidar':['rate_hz','range_m','points_per_second','timeout_s'],
            'navigation':['resolution_m','planning_radius_m','clearance_m','max_nodes','search_budget_s','lookahead_m','replan_period_s','position_gain'],
            'mission':['arrival_speed_mps','arrival_hold_s','takeoff_height_m','landing_handoff_m','landing_surface_tolerance_m']}.items():
            for name in fields:
                value=c[section][name]
                if not isinstance(value,(float,int)) or not math.isfinite(value) or value<=0:raise ValueError('Invalid '+section+'.'+name)
        if c['navigation']['planning_radius_m']>=c['navigation_lidar']['range_m']*.8:
            raise ValueError('Planning radius must leave observation range margin')
        if c['navigation']['clearance_m']<c['safety']['local_velocity']['clearance_m']:
            raise ValueError('Navigation clearance cannot be smaller than braking clearance')
        if c['mission']['landing_handoff_m']<=c['mission']['tolerance_m']+c['navigation']['clearance_m']*.5:
            raise ValueError('Landing handoff must remain above the support clearance')
        if c['mission']['takeoff_height_m']<c['navigation']['clearance_m']+.3:
            raise ValueError('Takeoff height must leave ground clearance')
        if not math.isfinite(c['navigation'].get('velocity_damping',.4)) or c['navigation'].get('velocity_damping',.4)<0:
            raise ValueError('Invalid navigation.velocity_damping')
        for key in ('landing_ground_z_m','landing_target_tolerance_m'):
            if not math.isfinite(c['mission'][key]):raise ValueError('Invalid mission.'+key)
        if c['mission']['landing_target_tolerance_m']<0:raise ValueError('Landing target tolerance cannot be negative')
        window=c['navigation_lidar'].get('scan_window_s',.3);sweep=c['navigation_lidar'].get('minimum_sweep_s',.04)
        if not math.isfinite(window) or not 0<window<=.35 or not math.isfinite(sweep) or not 0<=sweep<=window:
            raise ValueError('Invalid navigation LiDAR sweep window')
    for field,default in (('clock_tolerance_s',.075),('takeoff_timeout_s',60.)):
        value=c['control'].get(field,default)
        if not math.isfinite(value) or value<=0:raise ValueError('Invalid control.'+field)
    if c['control'].get('clock_tolerance_s',.075)>c['control']['ttl_s']:
        raise ValueError('Clock tolerance cannot exceed command TTL')
