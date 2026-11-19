"""Reject invalid timing, geometry and control budgets before node startup."""
import math
import numpy as np


def validate_config(c):
    model=c['model']
    if model.get('task','semantic') not in ('semantic','detect'):raise ValueError('Invalid model.task')
    if model.get('task')=='detect':
        if not c.get('surround',{}).get('enabled',False):raise ValueError('Detection mode requires the surround camera stack')
        for key in ('confidence','iou'):
            value=model.get(key)
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not 0<value<=1:
                raise ValueError('Invalid model.'+key)
        if type(model.get('max_det')) is not int or not 1<=model['max_det']<=128:raise ValueError('Invalid model.max_det')
        mapping=model.get('class_map')
        if not isinstance(mapping,dict) or not mapping or any(type(k) is not int or k<0 or type(v) is not int or v!=6 for k,v in mapping.items()):
            raise ValueError('Detection class_map must explicitly map checkpoint drone IDs to canonical class 6')
        if type(c['detection'].get('box_min_lidar_points')) is not int or not 2<=c['detection']['box_min_lidar_points']<=50:
            raise ValueError('Invalid detection.box_min_lidar_points')
        padding=c['detection'].get('box_roi_padding_px')
        if type(padding) is not int or not 0<=padding<=16:raise ValueError('Invalid detection.box_roi_padding_px')
    s=c.get('surround',{})
    if s.get('enabled',False):
        if len(s['cameras'])!=6 or len(set(v['name'] for v in s['cameras']))!=6:
            raise ValueError('Surround requires six uniquely named cube views')
        for k in ('width','height','imgsz'):
            if type(s[k]) is not int or s[k]<128 or s[k]>2048:raise ValueError('Invalid surround.'+k)
        if s['width']!=s['height'] or not 90<s['horizontal_fov_deg']<=120:
            raise ValueError('Cube views require square images and overlapping FOV')
        if not 1<=s['max_hz']<=20 or not .1<=s['max_age_s']<=.4:
            raise ValueError('Invalid surround timing budget')
        for k in ('min_mask_pixels','min_lidar_points','merge_distance_m'):
            if not math.isfinite(s[k]) or s[k]<=0:raise ValueError('Invalid surround.'+k)
        for k,default,limit in [('max_target_speed_mps',4.,20.),('max_motion_pixels',12.,24.),('roi_search_pixels',8,16)]:
            value=s.get(k,default)
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not 0<value<=limit:
                raise ValueError('Invalid surround.'+k)
        if type(s.get('roi_search_pixels',8)) is not int:raise ValueError('ROI search radius must be integer pixels')
        rate=s.get('state_sample_hz',50.)
        if isinstance(rate,bool) or not isinstance(rate,(int,float)) or not math.isfinite(rate) or not 20<=rate<=100:
            raise ValueError('Invalid surround state sample rate')
        for v in s['cameras']:
            if not v['name'].replace('_','').isalnum() or np.asarray(v['rpy_deg']).shape!=(3,) or not np.isfinite(v['rpy_deg']).all():
                raise ValueError('Invalid surround camera')
        lc=s['lidar']
        if not 1<=lc['rate_hz']<=20 or not 1<=lc['channels']<=512 or not 90<=lc['vertical_fov_deg']<=180:
            raise ValueError('Invalid surround LiDAR geometry/rate')
        if not 1<=lc['points_per_second']/lc['rate_hz']<=100000 or not 20<=lc['range_m']<=100:
            raise ValueError('Surround LiDAR exceeds scan budget')
    from .lidar_obstacles import DEFAULTS as LIDAR_DEFAULTS
    lidar={**LIDAR_DEFAULTS,**c.get('lidar_obstacles',{})}
    if set(lidar)!=set(LIDAR_DEFAULTS):raise ValueError('Unknown lidar_obstacles option')
    for key,value in lidar.items():
        if key=='enabled':
            if type(value) is not bool:raise ValueError('Invalid lidar_obstacles.enabled')
        elif key=='min_height_enu_m':
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):
                raise ValueError('Invalid LiDAR ground filter height')
        elif isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
            raise ValueError('Invalid lidar_obstacles.'+key)
    if not 1<=lidar['rate_hz']<=20 or not .05<=lidar['scan_window_s']<=.2 or not .1<=lidar['max_age_s']<=.5:
        raise ValueError('LiDAR motion rate/age exceeds supported budget')
    if type(lidar['min_points']) is not int or not 2<=lidar['min_points']<=50:raise ValueError('Invalid LiDAR min_points')
    if not .3<=lidar['motion_window_s']<=2 or not .2<=lidar['motion_min_span_s']<=lidar['motion_window_s']:
        raise ValueError('Invalid LiDAR motion confirmation window')
    from .ue_box_geometry import options as ue_box_options
    # Reuse the console schema budgets for direct launch configurations too.
    if 'experiment' in c or 'dynamic' in c.get('safety',{}):
        from .experiment import DEFAULTS as EXPERIMENT
        from .experiment_options import DYNAMIC_DEFAULTS as DYNAMIC
        for section,defaults in [(c.get('experiment',{}),EXPERIMENT),(c['safety'].get('dynamic',{}),DYNAMIC)]:
            for key,value in section.items():
                if key not in defaults:raise ValueError('Unknown experiment/dynamic option '+key)
                if key=='warning_events':
                    from .warning_audit import validate_events
                    validate_events(value);continue
                if isinstance(defaults[key],bool):
                    if type(value) is not bool:raise ValueError('Invalid '+key)
                elif isinstance(defaults[key],list):
                    a=np.asarray(value,float)
                    if a.shape!=(len(defaults[key]),) or not np.isfinite(a).all() or np.any(a<0):raise ValueError('Invalid '+key)
                elif isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:raise ValueError('Invalid '+key)
        dy={**DYNAMIC,**c['safety'].get('dynamic',{})}
        if not .05<=dy['step_s']<=.25 or not dy['step_s']<=dy['horizon_s']<=5 or not 1<=dy['max_targets']<=128:raise ValueError('Dynamic prediction budget exceeded')
    ue_box_options(c)
    for key,default in [('max_drone_extent_m',4.),('max_pedestrian_extent_m',3.)]:
        extent=c['detection'].get(key,default)
        if isinstance(extent,bool) or not isinstance(extent,(int,float)) or not math.isfinite(extent) or extent<=0:
            raise ValueError('Invalid detection.'+key)
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
    if 'truth_sample_hz' in c['evaluation']:
        rate=c['evaluation']['truth_sample_hz']
        if isinstance(rate,bool) or not isinstance(rate,(int,float)) or not math.isfinite(rate) or not 20<=rate<=100:
            raise ValueError('Invalid evaluation truth sample rate')
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
