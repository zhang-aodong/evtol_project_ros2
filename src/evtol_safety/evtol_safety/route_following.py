"""Sample each mission leg without turning the samples into stop waypoints."""
import math
import numpy as np
from scipy.spatial import cKDTree


class SampledLeg:
    def __init__(self, settings):
        self.spacing=float(settings.get('sample_spacing_m',2.))
        self.lookahead=float(settings.get('tracking_lookahead_m',4.))
        self.radius=float(settings.get('planning_radius_m',10.))
        self.clearance=float(settings.get('clearance_m',1.))+float(settings.get('resolution_m',.5))*.15
        self.max_samples=int(settings.get('max_reference_samples',10000))
        self.reset()

    def reset(self):
        self.key=None;self.samples=[];self.progress=0.;self.anchor=None

    def reference(self,position,target,key,points):
        p=np.asarray(position,float);goal=np.asarray(target,float)
        if self.key!=key:
            self.key=key;self.anchor=p.copy();self.progress=0.
            self.delta=goal-p;self.length=float(np.linalg.norm(self.delta))
            self.direction=self.delta/max(self.length,1e-9)
            count=max(1,int(math.ceil(self.length/self.spacing)))
            # Bound memory even for an unusually long user leg.
            count=min(count,self.max_samples-1)
            self.distances=np.linspace(0.,self.length,count+1)
            self.samples=self.anchor+self.distances[:,None]*self.direction
        if self.length<1e-9:return goal.copy()
        # Keep the leg anchor through an evasive manoeuvre. Rebuilding the
        # reference from the current pose every cycle creates a moving goal.
        along=float((p-self.anchor)@self.direction)
        self.progress=max(self.progress,min(self.length,max(0.,along)))
        desired=min(self.length,self.progress+self.lookahead)
        first=min(int(np.searchsorted(self.distances,desired)),len(self.samples)-1)
        candidates=list(range(first,len(self.samples)))
        candidates=[i for i in candidates if np.linalg.norm(self.samples[i]-p)<=self.radius-1.]
        if not candidates:
            delta=self.samples[first]-p
            return p+delta*min(1.,(self.radius-1.)/max(np.linalg.norm(delta),1e-9))
        pts=np.asarray(points,float).reshape(-1,3)
        tree=cKDTree(pts) if len(pts) else None
        # Do not turn a temporarily occupied intermediate sample into a final
        # goal_obstructed stop: a later sample can permit a checked detour.
        for i in candidates:
            if tree is None or tree.query(self.samples[i])[0]>=self.clearance:
                return self.samples[i].copy()
        return self.samples[candidates[0]].copy()


def cruise_velocity(position,aim,velocity,speed,distance,stop_required,settings,braking):
    delta=np.asarray(aim)-position;length=float(np.linalg.norm(delta))
    if length<1e-9:return np.zeros(3)
    unit=delta/length
    approach=float(speed)
    if stop_required:
        approach=min(approach,float(settings.get('position_gain',.5))*distance,
                     math.sqrt(2*braking*max(0.,distance)))
    # Dampen cross-track motion only. Longitudinal damping used to reduce a
    # nominal 2 m/s command to ~1.43 m/s in an otherwise clear corridor.
    desired=unit*approach-float(settings.get('velocity_damping',.4))*(velocity-unit*(velocity@unit))
    return desired*min(1.,approach/max(np.linalg.norm(desired),1e-9))
