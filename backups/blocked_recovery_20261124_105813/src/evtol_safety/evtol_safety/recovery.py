"""Bounded temporary-goal recovery; never changes or completes mission waypoints."""
from collections import deque
import math
import numpy as np
from scipy.spatial import cKDTree


DEFAULTS = dict(enabled=True, stalled_s=3., min_progress_m=.5, arrival_exclusion_m=1.,
                attempt_s=6., max_radius_m=15., radius_step_m=4., goal_tolerance_m=.6,
                max_vertical_offset_m=2., switch_hold_s=2.)


def options(value=None):
    result={**DEFAULTS,**(value or {})}
    if type(result['enabled']) is not bool:raise ValueError('Invalid navigation.recovery.enabled')
    bounds=dict(stalled_s=(1.,30.),min_progress_m=(.05,5.),arrival_exclusion_m=(.5,5.),
                attempt_s=(3.,30.),max_radius_m=(6.,30.),radius_step_m=(1.,10.),
                goal_tolerance_m=(.2,1.5),max_vertical_offset_m=(0.,5.),switch_hold_s=(.5,10.))
    for key,(lo,hi) in bounds.items():
        if isinstance(result[key],bool) or not isinstance(result[key],(int,float)) or not math.isfinite(result[key]) or not lo<=result[key]<=hi:
            raise ValueError('Invalid navigation.recovery.'+key)
    if result['switch_hold_s']>=result['attempt_s']:raise ValueError('Recovery switch hold must be shorter than attempt')
    return result


class RouteRecovery:
    def __init__(self,settings):
        self.o=options(settings.get('recovery'))
        self.base_radius=float(settings.get('planning_radius_m',6.))
        self.margin=float(settings.get('clearance_m',.8))+float(settings.get('resolution_m',.5))*.15
        self.reset()

    def reset(self):
        self.key=None;self.history=deque(maxlen=max(200,int(self.o['stalled_s']*100)+2));self.active=False;self.goal=None
        self.stage=0;self.started=0.;self.chosen=0.;self.attempt=0;self.visited=[]
        self.reason='normal';self.direction=None;self.radius=self.base_radius;self.progress_m=0.

    def pause(self):
        # Emergency/data/fault time is not evidence of a navigation deadlock.
        self.history.clear();self.active=False;self.goal=None;self.key=None;self.reason='paused'

    def observe(self,p,target,key,along,now,blocked=False):
        p=np.asarray(p,float);target=np.asarray(target,float)
        if key!=self.key:
            self.reset();self.key=key
        if not self.o['enabled'] or np.linalg.norm(target-p)<=self.o['arrival_exclusion_m']:
            self.active=False;self.goal=None;self.history.clear();self.reason='arrival_or_disabled';return
        self.history.append((now,float(along),bool(blocked)))
        while len(self.history)>1 and now-self.history[1][0]>=self.o['stalled_s']:self.history.popleft()
        self.progress_m=float(along)-self.history[0][1]
        blockage=any(sample[2] for sample in self.history if len(sample)>2)
        stalled=blockage and now-self.history[0][0]>=self.o['stalled_s'] and self.progress_m<self.o['min_progress_m']
        if self.active:
            if self.goal is not None and np.linalg.norm(p-self.goal)<=self.o['goal_tolerance_m']:
                self.active=False;self.goal=None;self.history.clear();self.reason='rejoin_original_leg';return
            if now-self.chosen>=self.o['attempt_s']:
                if self.goal is not None:self.visited.append(self.goal.copy())
                self.visited=self.visited[-8:];self.goal=None
                maximum=max(0,int(math.ceil((self.o['max_radius_m']-self.base_radius)/self.o['radius_step_m'])))
                self.stage=min(maximum,self.stage+1);self.chosen=now;self.reason='expand_search'
        elif stalled:
            self.active=True;self.started=now;self.chosen=now;self.goal=None;self.reason='stalled_recovery'
        self.radius=min(self.o['max_radius_m'],self.base_radius+self.stage*self.o['radius_step_m']) if self.active else self.base_radius

    def reference(self,p,target,normal,points,now,floor):
        if not self.active:return np.asarray(normal).copy()
        p=np.asarray(p,float);target=np.asarray(target,float);points=np.asarray(points,float).reshape(-1,3)
        tree=cKDTree(points) if len(points) else None
        def free(q):return tree is None or tree.query(q)[0]>=self.margin
        def clear(q):
            line=np.linspace(p,q,max(2,int(np.ceil(np.linalg.norm(q-p)/.15))+1))
            return tree is None or np.all(tree.query(line)[0]>=self.margin)
        if self.goal is None and free(normal) and clear(normal):
            # Slow unobstructed climbs must not generate lateral excursions.
            self.goal=np.asarray(normal,float).copy();self.chosen=now;self.attempt+=1
            self.reason='retry_clear_reference';return self.goal.copy()
        if self.goal is not None:
            # Persist a chosen side unless its endpoint is occupied. The A*
            # and the 20 Hz dynamic layer recheck the complete motion separately.
            if free(self.goal) or now-self.chosen<self.o['switch_hold_s']:return self.goal.copy()
            self.visited.append(self.goal.copy());self.goal=None
        heading=target-p;heading[2]=0.
        if np.linalg.norm(heading)<1e-6:heading=np.array([0.,1.,0.])
        heading/=np.linalg.norm(heading);side=np.array([-heading[1],heading[0],0.])
        reach=self.radius-1.
        heights=(0.,self.o['max_vertical_offset_m'],-self.o['max_vertical_offset_m'])
        candidates=[]
        for length in sorted(set((min(3.,reach),min(5.,reach),reach))):
            for angle in (0,45,-45,90,-90,135,-135,180):
                a=math.radians(angle);direction=heading*math.cos(a)+side*math.sin(a)
                for dz in heights:
                    q=p+length*direction+np.array([0.,0.,dz])
                    if np.linalg.norm(q-p)>reach or q[2]<floor or not free(q):continue
                    direct=bool(clear(q))
                    gain=np.linalg.norm(target-p)-np.linalg.norm(target-q)
                    # A reachable side-step is preferable to an occupied direct
                    # goal. Backtracking is permitted when forward exits fail.
                    score=gain-.12*length-.6*abs(dz)+(4. if direct else 0.)
                    if self.direction is not None:score+=.6*float(direction@self.direction)
                    if any(np.linalg.norm(q-v)<1.5 for v in self.visited):score-=6.
                    candidates.append((score,q,direction,direct))
        if not candidates:
            self.reason='blocked_no_recovery_goal';return None
        _,self.goal,direction,direct=max(candidates,key=lambda item:(item[3],item[0]))
        self.direction=direction.copy();self.chosen=now;self.attempt+=1
        self.reason='temporary_goal' if direct else 'search_detour_to_temporary_goal'
        return self.goal.copy()

    def status(self):
        return dict(active=self.active,reason=self.reason,attempt=self.attempt,radius_m=self.radius,
                    progress_window_m=self.progress_m,temporary_goal_enu_m=self.goal.tolist() if self.goal is not None else None)
