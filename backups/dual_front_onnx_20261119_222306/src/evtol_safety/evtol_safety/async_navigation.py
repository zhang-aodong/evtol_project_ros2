"""One spawned planner process, one overwritten pending request; never block safety."""
import multiprocessing as mp
import time
import queue
import numpy as np
from .navigation import VoxelNavigator


def _worker(requests,results,settings):
    planner=VoxelNavigator(settings)
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
        # Aim along the returned route from the current measured position.
        path=np.asarray(self.path);dist=np.linalg.norm(path-np.asarray(p),axis=1)
        i=int(np.argmin(dist));look=float(self.settings.get('lookahead_m',1.5))
        for point in path[min(i+1,len(path)-1):]:
            if np.linalg.norm(point-p)>=look:return point
        return path[-1]

    def close(self):
        if self.process is not None:
            if self.process.is_alive():self.process.terminate()
            self.process.join(timeout=1.)
        for q in (self.requests,self.results):
            if q is not None:q.cancel_join_thread();q.close()
        self.process=None;self.path=[];self.busy=False
