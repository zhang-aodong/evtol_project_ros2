"""Three-dimensional waypoint mission and bounded, collision-checked A* detours."""
import heapq
import itertools
import math
import time
import numpy as np
from scipy.spatial import cKDTree


class WaypointMission:
    def __init__(self, settings):
        self.c=settings;self.points=np.asarray(settings['waypoints_enu_m'],float)
        self.index=0;self.settled_since=None;self.phase='waiting';self.completed=False;self.takeoff_target=None;self.landing_started=False
        self.land=bool(settings.get('land_at_last',True) and
            abs(self.points[-1,2]-settings.get('landing_ground_z_m',0.))<=settings.get('landing_target_tolerance_m',.25))
        # A ground endpoint with landing disabled means hold at handoff height.
        # Trying to fly an ordinary waypoint into the support surface deadlocks.
        self.ground_hold=bool(not self.land and
            abs(self.points[-1,2]-settings.get('landing_ground_z_m',0.))<=settings.get('landing_target_tolerance_m',.25))

    def reference(self,position,velocity,now,flight_phase='flying'):
        p=np.asarray(position);v=np.asarray(velocity)
        if flight_phase=='landed':
            self.completed=True;self.index=len(self.points);self.phase='complete';return p.copy(),self.phase
        if self.completed:self.phase='complete';return p.copy(),self.phase
        if flight_phase=='takeoff':
            if self.takeoff_target is None:
                self.takeoff_target=p.copy();self.takeoff_target[2]+=self.c.get('takeoff_height_m',1.5)
            self.phase='takeoff';return self.takeoff_target.copy(),self.phase
        if flight_phase in ('waiting','arming','recovering','failsafe_hold','stopped','failed','shadow'):
            self.phase=flight_phase;return p.copy(),self.phase
        # Collinear mission waypoints are pass-through gates, not hover stops.
        if self.c.get('mode','waypoints')=='waypoints' and flight_phase=='flying' and self.pass_through() and self._passed(position):
            self.index+=1;self.settled_since=None
        target=self.points[self.index].copy();last=self.index==len(self.points)-1
        landing=last and self.land
        if landing or (last and self.ground_hold):target[2]+=self.c.get('landing_handoff_m',1.2)
        if self.landing_started:self.phase='landing';return target,self.phase
        distance=float(np.linalg.norm(target-p))
        settled=distance<=self.c['tolerance_m'] and np.linalg.norm(v)<=self.c.get('arrival_speed_mps',.3)
        if settled:
            if self.settled_since is None:self.settled_since=now
            if now-self.settled_since>=self.c.get('arrival_hold_s',.3):
                if landing:self.landing_started=True;self.phase='landing';return target,self.phase
                self.index+=1;self.settled_since=None
                if self.index==len(self.points):
                    self.completed=True;self.phase='complete';return p.copy(),self.phase
                target=self.points[self.index].copy()
                landing=self.index==len(self.points)-1 and self.land
                if landing or (self.index==len(self.points)-1 and self.ground_hold):target[2]+=self.c.get('landing_handoff_m',1.2)
        else:self.settled_since=None
        self.phase='landing' if flight_phase=='landing' else ('landing_approach' if landing else 'waypoint')
        return target,self.phase

    def pass_through(self):
        if self.c.get('mode','waypoints')!='waypoints' or not 0<self.index<len(self.points)-1:return False
        a=self.points[self.index]-self.points[self.index-1]
        b=self.points[self.index+1]-self.points[self.index]
        norm=np.linalg.norm(a)*np.linalg.norm(b)
        return bool(norm>1e-9 and a@b/norm>.999)

    def _passed(self,position):
        goal=self.points[self.index];relative=np.asarray(position)-goal
        incoming=goal-self.points[self.index-1];incoming/=max(np.linalg.norm(incoming),1e-9)
        along=float(relative@incoming);cross=np.linalg.norm(relative-along*incoming)
        return bool(np.linalg.norm(relative)<=self.c['tolerance_m'] or
                    (0<=along<=self.c['tolerance_m'] and cross<=self.c['tolerance_m']))


class VoxelNavigator:
    """Replans within the observed sphere; never interpolates through a wall.

    All edges and smoothed segments are checked at sub-voxel spacing with an
    inflated vehicle radius. A blocked/over-budget search returns no route.
    """
    def __init__(self,settings):
        self.res=float(settings.get('resolution_m',.5));self.radius=float(settings.get('planning_radius_m',10.))
        self.clearance=float(settings.get('clearance_m',1.));self.max_nodes=int(settings.get('max_nodes',16000))
        self.budget=float(settings.get('search_budget_s',.06));self.lookahead=float(settings.get('lookahead_m',1.5))
        self.path=[];self.reason='waiting';self.expanded=0
        self.path_goal=None
        self.moves=np.array([d for d in itertools.product((-1,0,1),repeat=3) if any(d)],int)
        self.move_cost=np.linalg.norm(self.moves,axis=1)*self.res
        if settings.get('horizontal_only',False):
            self.moves=self.moves[self.moves[:,2]==0]
            self.move_cost=np.linalg.norm(self.moves,axis=1)*self.res
        self.move_cost+=abs(self.moves[:,2])*self.res*float(settings.get('vertical_cost',1.5))

    def set_radius(self,radius):
        if abs(self.radius-radius)>1e-6:self.radius=float(radius);self.path=[];self.path_goal=None

    def plan(self,start,goal,points):
        start=np.asarray(start,float);goal=np.asarray(goal,float);points=np.asarray(points,float).reshape(-1,3)
        points=points[np.isfinite(points).all(axis=1)]
        tree=cKDTree(points) if len(points) else None
        # Extra inflation covers gaps between discrete segment test positions.
        margin=self.clearance+self.res*.15
        def clear(a,b):
            length=float(np.linalg.norm(b-a));n=max(2,int(math.ceil(length/(self.res*.35)))+1)
            samples=a+(b-a)*np.linspace(0.,1.,n)[:,None]
            return tree is None or bool(np.all(tree.query(samples,k=1)[0]>=margin))
        previous=self.path;self.path=[];self.expanded=0
        delta=goal-start;distance=float(np.linalg.norm(delta))
        if distance<1e-6:self.path=[start,goal];self.reason='at_goal';return goal.copy()
        end=start+delta*min(1.,(self.radius-self.res*2)/distance)
        if clear(start,end):self.path=[start,end];self.reason='direct';return self._aim(start)
        if previous and self.path_goal is not None and np.allclose(goal,self.path_goal):
            remaining=[np.asarray(p) for p in previous[1:]]
            while remaining and np.linalg.norm(remaining[0]-start)<self.res*.8:remaining.pop(0)
            route=[start]+remaining
            if remaining and np.linalg.norm(remaining[-1]-start)>self.res and all(clear(a,b) for a,b in zip(route,route[1:])):
                self.path=route;self.reason='detour';return self._aim(start)
        self.path_goal=goal.copy()
        zero=np.zeros(3,dtype=int);target=np.rint((end-start)/self.res).astype(int)
        valid_cache={}
        def position(key):return start+np.asarray(key)*self.res
        def valid(key):
            if key not in valid_cache:
                point=position(key)
                valid_cache[key]=bool(np.linalg.norm(point-start)<=self.radius and
                    (tree is None or tree.query(point)[0]>=margin))
            return valid_cache[key]
        if not valid(tuple(target)):
            if distance<=self.radius-self.res*2:
                self.reason='goal_obstructed';return None
            # A long route's clipped local endpoint can lie on a wall even
            # when the actual waypoint is farther away and reachable. Select
            # a nearby free endpoint to let the graph search go around it.
            extent=max(2,int(math.ceil(2*self.clearance/self.res)))
            alternatives=[]
            for offset in itertools.product(range(-extent,extent+1),repeat=3):
                candidate=tuple(target+offset)
                if valid(candidate):alternatives.append(candidate)
            if not alternatives:self.reason='no_local_route';return None
            target=np.asarray(min(alternatives,key=lambda k:np.linalg.norm(position(k)-goal)))
        begin=tuple(zero);target=tuple(target);cost={begin:0.};parents={};queue=[(float(np.linalg.norm(end-start)),0.,begin)]
        deadline=time.perf_counter()+self.budget;found=None
        while queue and self.expanded<self.max_nodes:
            if time.perf_counter()>deadline:self.reason='planning_budget_exceeded';return None
            _,g,key=heapq.heappop(queue)
            if g>cost.get(key,float('inf'))+1e-9:continue
            self.expanded+=1
            if key==target:found=key;break
            a=position(key)
            # Batch the 26 edge queries. A separate Python/KD-tree call for
            # each neighbor can exhaust a control-cycle budget on Windows.
            neighbors=np.asarray(key)+self.moves
            positions=start+neighbors*self.res
            candidates=[]
            for j,neighbor in enumerate(map(tuple,neighbors)):
                ng=g+self.move_cost[j]
                if ng<cost.get(neighbor,float('inf')) and np.linalg.norm(positions[j]-start)<=self.radius:
                    candidates.append(j)
            if not candidates:continue
            samples=a+(positions[candidates,None,:]-a)*np.linspace(0.,1.,7)[None,:,None]
            free=np.ones(len(candidates),bool) if tree is None else (tree.query(samples.reshape(-1,3))[0].reshape(-1,7)>=margin).all(axis=1)
            for j in np.asarray(candidates)[free]:
                neighbor=tuple(neighbors[j]);ng=g+self.move_cost[j]
                cost[neighbor]=ng;parents[neighbor]=key
                heapq.heappush(queue,(ng+float(np.linalg.norm(positions[j]-position(target))),ng,neighbor))
        if found is None:self.reason='no_local_route';return None
        keys=[found]
        while keys[-1]!=begin:keys.append(parents[keys[-1]])
        raw=[position(k) for k in reversed(keys)];smooth=[raw[0]];i=0
        while i<len(raw)-1:
            j=len(raw)-1
            while j>i+1 and not clear(raw[i],raw[j]):j-=1
            smooth.append(raw[j]);i=j
        if clear(smooth[-1],end):smooth.append(end)
        self.path=smooth;self.reason='detour';return self._aim(start)

    def _aim(self,start):
        # Select along the first collision-checked segment. Skipping a turn
        # would cut the corner and invalidate the checked path.
        for point in self.path[1:]:
            delta=point-start;distance=float(np.linalg.norm(delta))
            if distance>1e-5:return start+delta*min(1.,self.lookahead/distance)
        return np.asarray(start).copy()


class HorizontalFirstNavigator:
    """Bounded horizontal A* first, followed by full 26-neighbor 3-D A*."""
    def __init__(self,settings):
        budget=float(settings.get('search_budget_s',.08))
        self.horizontal=VoxelNavigator(dict(settings,horizontal_only=True,search_budget_s=budget*.4))
        self.spatial=VoxelNavigator(dict(settings,horizontal_only=False,search_budget_s=budget*.6))
        self.path=[];self.reason='waiting';self.settings=settings

    def set_radius(self,radius):
        if abs(self.horizontal.radius-radius)>1e-6:
            self.horizontal.set_radius(radius);self.spatial.set_radius(radius);self.path=[]

    def plan(self,p,goal,points):
        if abs(goal[2]-p[2])<=self.horizontal.res*.5:
            level=np.asarray(goal).copy();level[2]=p[2]
            aim=self.horizontal.plan(p,level,points)
            if aim is not None:
                self.path=self.horizontal.path;self.reason='horizontal_'+self.horizontal.reason
                return aim
        aim=self.spatial.plan(p,goal,points)
        self.path=self.spatial.path;self.reason=('spatial_' if aim is not None else '')+self.spatial.reason
        return aim
