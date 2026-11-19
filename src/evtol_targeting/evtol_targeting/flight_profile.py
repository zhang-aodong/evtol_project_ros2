"""Compile ordered speed/height points to ENU legs in one vertical plane.

Same height -> forward by the supplied horizontal distance. Different height
-> vertical only. Speed means ground-speed magnitude, not horizontal airspeed.
This module is pure: it never starts ROS or commands a vehicle.
"""
import math
import numpy as np


def finite(value, name, minimum=0., maximum=1e6):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not minimum<=value<=maximum:
        raise ValueError(name+' is outside the supported range')
    return float(value)


def compile_profile(profile, speed_cap=30., acceleration_cap=5.):
    heading=finite(profile.get('heading_deg',0.),'heading_deg',-3600.,3600.)
    points=profile.get('points',[])
    if not isinstance(points,list) or not 2<=len(points)<=256:
        raise ValueError('Profile requires 2..256 ordered points')
    direction=np.array([math.sin(math.radians(heading)),math.cos(math.radians(heading)),0.])
    route=[];legs=[];normalized=[]
    for i,p in enumerate(points):
        v=finite(p.get('speed_mps'),'profile speed',0.,speed_cap)
        h=finite(p.get('height_m'),'profile height',0.,3000.)
        distance=finite(p.get('distance_m',100.),'horizontal distance',.05,10000.)
        normalized.append(dict(speed_mps=v,height_m=h,distance_m=distance))
        if not i:
            route.append(np.array([0.,0.,h]));continue
        previous=normalized[-2];start=route[-1];end=start.copy()
        vertical=abs(h-previous['height_m'])>1e-6
        if vertical:end[2]=h
        else:end+=direction*distance
        length=float(np.linalg.norm(end-start));v0=previous['speed_mps'];v1=v
        if v0+v1<=1e-9:raise ValueError('A nonzero profile leg cannot have zero speed at both ends')
        acceleration=(v1*v1-v0*v0)/(2*length)
        if abs(acceleration)>acceleration_cap+1e-9:
            raise ValueError(f'Profile leg {i}: requested acceleration exceeds {acceleration_cap:g} m/s^2; increase distance or reduce speed')
        duration=2*length/(v0+v1)
        legs.append(dict(start_enu_m=start.tolist(),end_enu_m=end.tolist(),length_m=length,
            start_speed_mps=v0,end_speed_mps=v1,acceleration_mps2=acceleration,
            nominal_duration_s=duration,kind='vertical' if vertical else 'horizontal'))
        route.append(end)
    if normalized[0]['speed_mps']>1e-6:raise ValueError('Profile must start at zero speed')
    if normalized[-1]['speed_mps']>1e-6:raise ValueError('Profile must end at zero speed to settle/land')
    return dict(points=normalized,heading_deg=heading,waypoints_enu_m=[p.tolist() for p in route],
        legs=legs,nominal_duration_s=sum(x['nominal_duration_s'] for x in legs))


def leg_speed(leg, position, acceleration_cap):
    """Distance-based speed envelope; deviations do not advance a wall timer."""
    start=np.asarray(leg['start_enu_m']);end=np.asarray(leg['end_enu_m']);length=leg['length_m']
    progress=float(np.clip((np.asarray(position)-start)@(end-start)/length,0.,length))
    v0=leg['start_speed_mps'];v1=leg['end_speed_mps']
    # A small lookahead starts an accelerating zero-speed leg without a
    # zero-command equilibrium. The existing acceleration limiter still applies.
    lookahead=min(length,progress+.05) if v1>v0 else progress
    value=math.sqrt(max(0.,v0*v0+(v1*v1-v0*v0)*lookahead/length))
    return min(max(v0,v1),max(.1,value))
