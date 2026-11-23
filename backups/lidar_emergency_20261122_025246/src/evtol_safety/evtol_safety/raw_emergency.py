"""Near-field measured-ray fallback, independent of visual classes and tracks.

Angular cells retain their nearest return. Range change is only a radial
approach hint, not an identified obstacle's full velocity. Ego translation is
removed before taking that difference. No simulator state or truth is used.
"""
import math
import numpy as np
from .dynamic import Obstacle

DEFAULTS=dict(enabled=True,radius_m=2.,escape_speed_mps=1.5,self_exclusion_m=.25,
              max_range_rate_mps=12.,range_history_s=.25)


def options(value=None):
    if value is not None and not isinstance(value,dict):
        raise ValueError('raw_emergency must be a configuration object')
    d={**DEFAULTS,**(value or {})}
    if set(d)!=set(DEFAULTS):raise ValueError('Unknown raw_emergency options')
    if type(d['enabled']) is not bool:raise ValueError('raw_emergency.enabled must be boolean')
    for k,v in d.items():
        if k=='enabled':continue
        if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or v<=0:
            raise ValueError('Invalid raw_emergency.'+k)
    if not 1.<=d['radius_m']<=10.:raise ValueError('raw_emergency.radius_m must be 1..10 m')
    if not .05<=d['self_exclusion_m']<=.4 or d['self_exclusion_m']>=d['radius_m']:
        raise ValueError('Invalid raw emergency self exclusion')
    if not .1<=d['escape_speed_mps']<=5. or not 1<=d['max_range_rate_mps']<=30. or not .05<=d['range_history_s']<=.5:
        raise ValueError('Raw emergency speed/history exceeds supported bounds')
    return d


class RawLidarEmergency:
    def __init__(self,value=None):self.o=options(value);self.reset()

    def reset(self):
        self.stamp=0;self.cells={};self.rates={};self.diagnostics=dict(armed=False,active=False,reason='waiting_for_flight')

    def update(self,world,position,capture_position,stamp_ns,age_s,armed,ground_z=None,ground_tolerance=.15,clearance=.8,*,estimate_motion=True):
        if not self.o['enabled'] or not armed:
            self.reset();self.diagnostics['reason']='disabled' if not self.o['enabled'] else 'waiting_for_flight'
            return [],np.empty((0,3))
        p=np.asarray(position,float);capture=np.asarray(capture_position,float)
        points=np.asarray(world,float).reshape(-1,3);points=points[np.isfinite(points).all(axis=1)]
        relative=points-p;distance=np.linalg.norm(relative,axis=1)
        valid=(distance<=self.o['radius_m'])&(distance>=self.o['self_exclusion_m'])
        # Exclude only the configured touchdown plane, never a vertical column
        # or an entire lower hemisphere. Raised/side/overhead objects remain.
        if ground_z is not None:
            valid &= ~((points[:,2]<=ground_z+ground_tolerance)&(points[:,2]<p[2]-.3))
        points=points[valid];relative=relative[valid];distance=distance[valid]
        if stamp_ns<self.stamp:self.reset()
        if not len(points):
            if stamp_ns>self.stamp:self.cells={};self.rates={};self.stamp=int(stamp_ns)
            self.diagnostics=dict(armed=True,active=False,reason='clear',nearest_m=None,points=0,cells=0,max_closing_mps=0.)
            return [],points
        # 18 yaw x 4 elevation bins: <=72 constraints regardless of cloud size.
        yaw=np.floor((np.arctan2(relative[:,1],relative[:,0])+np.pi)/(2*np.pi)*18).astype(int)%18
        pitch=np.minimum(3,np.floor((np.arcsin(np.clip(relative[:,2]/distance,-1,1))+np.pi/2)/np.pi*4).astype(int))
        keys=yaw+18*pitch;order=np.argsort(distance);_,indices=np.unique(keys[order],return_index=True);chosen=order[indices]
        current={int(keys[i]):points[i] for i in chosen};rates={}
        if stamp_ns>self.stamp:
            dt=(stamp_ns-self.stamp)*1e-9
            for key,point in current.items():
                previous=self.cells.get(key);radial=point-capture;length=np.linalg.norm(radial)
                if estimate_motion and previous is not None and .005<=dt<=self.o['range_history_s'] and length>1e-6:
                    before=previous-capture;before_length=np.linalg.norm(before)
                    if before_length>1e-6 and radial@before/(length*before_length)>.94:
                        # A stationary world return has exactly zero approach
                        # even when ego is translating or rotating.
                        rates[key]=float(np.clip((length-before_length)/dt,-self.o['max_range_rate_mps'],0.))
            self.cells=current;self.rates=rates;self.stamp=int(stamp_ns)
        obstacles=[]
        for key,point in current.items():
            delta=point-p;unit=delta/max(np.linalg.norm(delta),1e-9)
            speed=self.rates.get(key,0.);v=unit*speed
            # Compensate only measured inward radial motion; no assumed full
            # target velocity or ground-truth association is introduced.
            obstacles.append(Obstacle(-key-1,delta+v*max(0.,age_s),v,float(clearance)))
        self.diagnostics=dict(armed=True,active=True,reason='nearby_lidar_returns',nearest_m=float(distance.min()),
            points=len(points),cells=len(obstacles),max_closing_mps=max((-self.rates.get(k,0.) for k in current),default=0.))
        return obstacles,points

    def preference(self,desired,obstacles,clearance,speed_limit):
        if not obstacles:return np.asarray(desired,float)
        vectors=np.asarray([o.position for o in obstacles]);dist=np.linalg.norm(vectors,axis=1)
        unit=vectors/np.maximum(dist[:,None],1e-9)
        closing=np.array([max(0.,-o.velocity@n) for o,n in zip(obstacles,unit)])
        # Highest threat first; equal/opposed pressures must not cancel and
        # manufacture a "clear" hover. The velocity selector checks all rays.
        urgency=(self.o['radius_m']-dist)/max(self.o['radius_m']-clearance,.1)+closing
        worst=int(np.argmax(urgency));away=-unit[worst]
        escape=min(speed_limit,self.o['escape_speed_mps'])
        reference=np.asarray(desired,float).copy()
        outward=min(escape,max(.15,escape*np.clip(urgency[worst],0,1)))
        reference+=away*max(0.,outward-reference@away)
        return reference*min(1.,speed_limit/max(np.linalg.norm(reference),1e-9))
