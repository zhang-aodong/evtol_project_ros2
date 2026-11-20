"""Timestamp-aligned independent evaluation; no SE(3)/scale truth fitting."""
from bisect import bisect_left
from collections import deque
import math
import numpy as np


def interpolate(history,t,max_gap_ns):
    if not history:return None
    stamps=[x[0] for x in history];i=bisect_left(stamps,t)
    if i<len(history) and stamps[i]==t:return history[i][1:]
    if i==0 or i==len(history):return None
    a,b=history[i-1],history[i]
    if b[0]-a[0]>max_gap_ns:return None
    f=(t-a[0])/(b[0]-a[0]);p=a[1]*(1-f)+b[1]*f
    qa,qb=a[2],b[2]
    if qa@qb<0:qb=-qb
    q=qa*(1-f)+qb*f;q/=np.linalg.norm(q)
    return p,q


class ErrorAudit:
    def __init__(self):
        self.est=deque(maxlen=3000);self.pending=deque(maxlen=2000)
        self.attempts=0;self.matched=0;self.last=None;self.rows=deque(maxlen=200000)

    def estimate(self,t,p,q):
        if self.est and t<=self.est[-1][0]:return
        self.est.append((t,np.asarray(p),np.asarray(q)))

    def truth(self,t,p,q):
        if self.pending and t<=self.pending[-1][0]:return
        self.pending.append((t,np.asarray(p),np.asarray(q)))

    def drain(self,cutoff,phase):
        new=[]
        while self.pending and self.pending[0][0]<=cutoff:
            t,p,q=self.pending.popleft();self.attempts+=1
            value=interpolate(self.est,t,int(.12e9))
            if value is None:continue
            ep,eq=value;delta=ep-p;angle=2*math.acos(min(1.,abs(float(eq@q))))
            row=dict(stamp_ns=t,error_enu_m=delta.tolist(),error_3d_m=float(np.linalg.norm(delta)),
                attitude_error_deg=math.degrees(angle),phase=phase(t))
            self.rows.append(row);self.last=row;self.matched+=1;new.append(row)
        return new

    def summary(self):
        def summarize(rows):
            e=np.array([r['error_3d_m'] for r in rows])
            return dict(samples=len(e),rmse_3d_m=float(np.sqrt(np.mean(e*e))) if len(e) else None,
                p95_3d_m=float(np.percentile(e,95)) if len(e) else None,max_3d_m=float(max(e)) if len(e) else None,
                last_error_3d_m=float(e[-1]) if len(e) else None)
        return dict(**summarize(self.rows),truth_epochs=self.attempts,aligned_epochs=self.matched,
            coverage=self.matched/self.attempts if self.attempts else None,
            normal=summarize([r for r in self.rows if r['phase']=='normal']),
            dropout=summarize([r for r in self.rows if r['phase']=='dropout']),
            bias=summarize([r for r in self.rows if r['phase']=='bias']),
            no_fix=summarize([r for r in self.rows if r['phase']=='no_fix']))
