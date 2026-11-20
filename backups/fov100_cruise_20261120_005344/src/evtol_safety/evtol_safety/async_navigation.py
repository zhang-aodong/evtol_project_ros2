"""One spawned planner process, one overwritten pending request; never block safety."""
import multiprocessing as mp
import time
import queue
import numpy as np
from .navigation import VoxelNavigator,HorizontalFirstNavigator
from scipy.spatial import cKDTree


def _worker(requests,results,settings):
    planner=HorizontalFirstNavigator(settings) if settings.get('strategy')=='horizontal_first_astar' else VoxelNavigator(settings)
    try:
        while True:
            request=requests.get()
            if request is None:return
            version,p,goal,points,submitted=request
            try:
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

    def _start(self):
        ctx=mp.get_context('spawn');self.requests=ctx.Queue(maxsize=1);self.results=ctx.Queue(maxsize=1)
        self.process=ctx.Process(target=_worker,args=(self.requests,self.results,self.settings),daemon=True,name='evtol_local_planner')
        self.process.start()

    def plan(self,p,goal,points):
        if self.process is None:self._start()
        if not self.process.is_alive():raise RuntimeError('Async planner exited')
        goal=np.asarray(goal)
        if self.goal is None or not np.allclose(self.goal,goal):
            self.goal=goal.copy();self.path=[];self.received=0.;self.version+=1;self.reason='waiting_for_async_plan'
        try:
            version,result_goal,path,reason,completed=self.results.get_nowait();self.busy=False
            if version==self.version and np.allclose(result_goal,self.goal):
                self.path=path;self.reason=reason;self.received=completed
        except queue.Empty:pass
        self.pending=(self.version,np.asarray(p),goal,np.asarray(points),time.monotonic())
        if not self.busy:
            try:
                self.requests.put_nowait(self.pending);self.active=self.version;self.busy=True;self.pending=None
            except queue.Full:pass
        if self.path and time.monotonic()-self.received>self.ttl:
            self.path=[];self.reason='async_route_expired'
        if not self.path:
            return None  # retain goal_obstructed/no_local_route from the worker
        # Revalidate against the latest cloud. A returned asynchronous route
        # may be obsolete after a target moves or the vehicle evades sideways.
        path=np.asarray(self.path);p=np.asarray(p);look=float(self.settings.get('lookahead_m',1.5))
        tree=cKDTree(points) if len(points) else None
        clearance=float(self.settings.get('clearance_m',1.))+float(self.settings.get('resolution_m',.5))*.15
        def clear(a,b):
            count=max(2,int(np.ceil(np.linalg.norm(b-a)/.15))+1)
            samples=a+(b-a)*np.linspace(0,1,count)[:,None]
            return tree is None or bool(np.all(tree.query(samples)[0]>=clearance))
        candidates=[i for i in range(1,len(path)) if clear(p,path[i]) and all(clear(a,b) for a,b in zip(path[i:],path[i+1:]))]
        if not candidates:
            self.path=[];self.reason='route_invalidated';return None
        i=min(candidates,key=lambda i:np.linalg.norm(p-path[i])+np.linalg.norm(np.diff(path[i:],axis=0),axis=1).sum())
        delta=path[i]-p;length=np.linalg.norm(delta)
        return p+delta*min(1.,look/max(length,1e-9))

    def close(self):
        if self.process is not None:
            if self.process.is_alive():self.process.terminate()
            self.process.join(timeout=1.)
        for q in (self.requests,self.results):
            if q is not None:q.cancel_join_thread();q.close()
        self.process=None;self.path=[];self.busy=False
