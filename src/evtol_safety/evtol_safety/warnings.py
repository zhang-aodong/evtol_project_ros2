"""Warning geometry is independent of truth, model visibility and path planning."""
import numpy as np

DEFAULTS=dict(lidar_radius_m=5.,lidar_enable_agl_m=5.,self_exclusion_m=.3,
              ground_column_radius_m=1.,ground_min_points=3,ground_max_age_s=2.,
              fusion_radius_m=10.,prediction_horizon_s=3.,prediction_step_s=.5,
              max_measurement_age_s=.3,target_success_rate=.9)


def validate_options(options):
    import math
    if set(options)!=set(DEFAULTS):raise ValueError('Unknown or missing warning options')
    for key,value in options.items():
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
            raise ValueError('Invalid warnings.'+key)
    if options['self_exclusion_m']>=options['lidar_radius_m']:raise ValueError('Self exclusion must be smaller than warning radius')
    if options['prediction_horizon_s']>5 or not .1<=options['prediction_step_s']<=options['prediction_horizon_s']:
        raise ValueError('Invalid warning prediction budget')
    if options['prediction_horizon_s']/options['prediction_step_s']>50:raise ValueError('Too many warning prediction samples')
    if not 0<options['target_success_rate']<=1 or not .1<=options['max_measurement_age_s']<=.5:raise ValueError('Invalid warning timing/target')
    if type(options['ground_min_points']) is not int or not 1<=options['ground_min_points']<=100:raise ValueError('Invalid ground minimum points')


def in_hemisphere(relative,forward,radius):
    p=np.asarray(relative,float);f=np.asarray(forward,float)
    return bool(np.isfinite(p).all() and np.linalg.norm(p)<=radius+1e-9 and p@f>=-1e-9)


def proximity(points,agl,options=None):
    o={**DEFAULTS,**(options or {})};p=np.asarray(points,float).reshape(-1,3)
    p=p[np.isfinite(p).all(axis=1)];d=np.linalg.norm(p,axis=1)
    d=d[d>o['self_exclusion_m']];near=d[d<=o['lidar_radius_m']]
    enabled=agl is not None and agl>o['lidar_enable_agl_m']
    return dict(valid=agl is not None,enabled=enabled,warning=bool(enabled and len(near)),
        agl_m=agl,radius_m=o['lidar_radius_m'],point_count=int(len(near)),
        nearest_m=float(d.min()) if len(d) else None,
        reason='agl_unknown' if agl is None else 'below_enable_height' if not enabled else 'near_points' if len(near) else 'clear')


def predict_target(position,velocity,ego_position,ego_velocity,forward,options=None):
    o={**DEFAULTS,**(options or {})};r=np.asarray(position)-ego_position;v=np.asarray(velocity)-ego_velocity
    times=np.arange(0.,o['prediction_horizon_s']+1e-9,o['prediction_step_s'])
    future=r[None,:]+times[:,None]*v
    speed=float(np.linalg.norm(v));distance=float(np.linalg.norm(r))
    tcpa=float(np.clip(-r@v/max(v@v,1e-12),0,o['prediction_horizon_s']))
    inside=[float(t) for t,p in zip(times,future) if in_hemisphere(p,forward,o['fusion_radius_m'])]
    return dict(warning=in_hemisphere(r,forward,o['fusion_radius_m']),distance_m=distance,
        relative_position_enu_m=r.tolist(),relative_velocity_enu_mps=v.tolist(),relative_speed_mps=speed,
        closing_speed_mps=float(-r@v/max(distance,1e-9)),cpa_time_s=tcpa,cpa_distance_m=float(np.linalg.norm(r+tcpa*v)),
        predicted_entry_s=inside[0] if inside else None,
        prediction=[dict(dt_s=float(t),distance_m=float(np.linalg.norm(p)),relative_position_enu_m=p.tolist()) for t,p in zip(times,future)])
