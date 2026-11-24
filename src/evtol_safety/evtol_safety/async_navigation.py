"""One spawned planner process, one overwritten pending request; never block safety."""
import multiprocessing as mp
import time
import queue
import numpy as np
from .navigation import VoxelNavigator,HorizontalFirstNavigator
from scipy.spatial import cKDTree


def _worker(requests,results,settings):
    planner_type=HorizontalFirstNavigator if settings.get('strategy')=='horizontal_first_astar' else VoxelNavigator
    planners={}
    try:
        while True:
            request=requests.get()
            if request is None:return
            version,p,goal,points,submitted,radius=request
            try:
                if radius not in planners:planners[radius]=planner_type(dict(settings,planning_radius_m=radius))
                planner=planners[radius]
                planner.plan(p,goal,points)
                results.put((version,goal,planner.path,planner.reason,submitted))
            except Exception as exc:results.put((version,goal,[],str(exc),submitted))
    except (EOFError,BrokenPipeError):pass


class AsyncNavigator:
    def __init__(self,settings):
        self.settings=settings;self.path=[];self.reason='waiting_for_async_plan'
        self.requests=None;self.results=None;self.process=None;self.busy=False;self.pending=None
        self.version=0;self.active=0;self.received=0.;self.goal=None
        self.ttl=float(settings.get('route_timeout_s',.75))
        self.last_submit=0.
        self.radius=float(settings.get('planning_radius_m',6.))

    def set_radius(self,radius):
        radius=float(radius)
        if abs(radius-self.radius)>1e-6:
            self.radius=radius;self.suspend()

    def _start(self):
        ctx=mp.get_context('spawn');self.requests=ctx.Queue(maxsize=1);self.results=ctx.Queue(maxsize=1)
        self.process=ctx.Process(target=_worker,args=(self.requests,self.results,self.settings),daemon=True,name='evtol_local_planner')
        self.process.start()

    def suspend(self):
        # Do not submit work during emergency control. Any already bounded
        # worker result belongs to the old generation and is discarded later.
        self.version+=1;self.path=[];self.pending=None;self.goal=None;self.received=0.
        self.reason='paused_for_lidar_emergency'

    def plan(self,p,goal,points):
        goal=np.asarray(goal,float);p=np.asarray(p,float);points=np.asarray(points,float).reshape(-1,3)
        now=time.monotonic()
        if self.goal is None or not np.allclose(self.goal,goal):
            self.goal=goal.copy();self.path=[];self.received=0.;self.version+=1;self.reason='waiting_for_async_plan'
        if self.process is not None:
            if not self.process.is_alive():raise RuntimeError('Async planner exited')
            try:
                version,result_goal,path,reason,submitted=self.results.get_nowait();self.busy=False
                if version==self.version and np.allclose(result_goal,self.goal) and now-submitted<=self.ttl:
                    # A failed refresh must not erase a route that passes the
                    # current scan's collision checks below.
                    if path:self.path=path;self.received=submitted
                    self.reason=reason
            except queue.Empty:pass
        look=float(self.settings.get('lookahead_m',1.5))
        tree=cKDTree(points) if len(points) else None
        clearance=float(self.settings.get('clearance_m',1.))+float(self.settings.get('resolution_m',.5))*.15
        def clear(a,b):
            count=max(2,int(np.ceil(np.linalg.norm(b-a)/.15))+1)
            samples=a+(b-a)*np.linspace(0,1,count)[:,None]
            return tree is None or bool(np.all(tree.query(samples)[0]>=clearance))
        def aim(point):
            delta=point-p;length=np.linalg.norm(delta)
            return p+delta*min(1.,look/max(length,1e-9))
        # Keep a checked detour instead of alternating sides every refresh.
        # Validate each remaining edge once, rather than once per candidate.
        if len(self.path)>1:
            path=np.asarray(self.path)
            suffix=np.ones(len(path),bool)
            for j in range(len(path)-2,0,-1):suffix[j]=suffix[j+1] and clear(path[j],path[j+1])
            candidates=[i for i in range(1,len(path)) if np.linalg.norm(path[i]-p)>.2 and suffix[i] and clear(p,path[i])]
            if candidates:
                lengths=np.r_[np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1)[::-1])[::-1],0.]
                i=min(candidates,key=lambda i:np.linalg.norm(p-path[i])+lengths[i])
                self.path=[p.copy()]+list(path[i:]);self.reason='tracking_checked_route'
                return aim(path[i])
            self.path=[];self.reason='route_invalidated'
        # Clear sampled corridor needs no planner process and no first-result
        # pause. The local safety layer still checks braking and dynamics.
        if clear(p,goal):
            self.path=[p.copy(),goal.copy()];self.reason='sampled_corridor_clear'
            return aim(goal)
        if self.process is None:self._start()
        self.pending=(self.version,p.copy(),goal.copy(),points.copy(),now,self.radius)
        if not self.busy and now-self.last_submit>=float(self.settings.get('replan_period_s',.5)):
            try:
                self.requests.put_nowait(self.pending);self.active=self.version;self.busy=True
                self.pending=None;self.last_submit=now
            except queue.Full:pass
        return None

    def close(self):
        if self.process is not None:
            if self.process.is_alive():self.process.terminate()
            self.process.join(timeout=1.)
        for q in (self.requests,self.results):
            if q is not None:q.cancel_join_thread();q.close()
        self.process=None;self.path=[];self.busy=False;self.last_submit=0.;self.goal=None
