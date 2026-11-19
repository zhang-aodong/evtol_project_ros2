"""ROS-independent time alignment and source-index-preserving cloud projection."""
from collections import deque
import bisect
import numpy as np
from .projection import quaternion_xyzw_to_rotation_matrix, ProjectionResult


class PoseHistory:
    def __init__(self, max_gap_s=.15, capacity=512):
        self.samples=deque(maxlen=capacity);self.max_gap_ns=int(max_gap_s*1e9)

    def add(self, t, p, q):
        p,q=np.asarray(p,dtype=float),np.asarray(q,dtype=float)
        if p.shape!=(3,) or q.shape!=(4,) or not np.isfinite(p).all() or not np.isfinite(q).all() or np.linalg.norm(q)<1e-9:
            raise ValueError('Invalid pose')
        if self.samples and t<self.samples[-1][0]:self.samples.clear()
        if self.samples and t==self.samples[-1][0]:return
        self.samples.append((t,p.copy(),q/np.linalg.norm(q)))

    def at(self,t):
        samples=tuple(self.samples)
        i=bisect.bisect_left([s[0] for s in samples],t)
        if i<len(samples) and samples[i][0]==t:
            _,p,q=samples[i];return p,quaternion_xyzw_to_rotation_matrix(q)
        if i==0 or i==len(samples):return None
        ta,pa,qa=samples[i-1];tb,pb,qb=samples[i]
        if tb-ta>self.max_gap_ns:return None
        a=(t-ta)/(tb-ta);dot=float(qa@qb)
        if dot<0:qb=-qb;dot=-dot
        if dot>.9995:q=(1-a)*qa+a*qb
        else:
            angle=np.arccos(np.clip(dot,-1,1))
            q=(np.sin((1-a)*angle)*qa+np.sin(a*angle)*qb)/np.sin(angle)
        return (1-a)*pa+a*pb,quaternion_xyzw_to_rotation_matrix(q)


def compensated_projection(points, lidar_pose, camera_pose, body_lidar, body_optical, k, width, height):
    """Returns world points and pixel mapping; handles different ego epochs."""
    pl,rl=lidar_pose;pc,rc=camera_pose
    tl,ql=body_lidar;tc,qc=body_optical
    rbl=quaternion_xyzw_to_rotation_matrix(ql);rbc=quaternion_xyzw_to_rotation_matrix(qc)
    world=(points@rbl.T+tl)@rl.T+pl
    optical=((world-pc)@rc-tc)@rbc
    z=optical[:,2]
    ids=np.flatnonzero(np.isfinite(optical).all(axis=1)&(z>.2))
    pixels=np.rint(optical[ids,:2]/z[ids,None]*[k[0,0],k[1,1]]+[k[0,2],k[1,2]]).astype(np.int32)
    inside=(pixels[:,0]>=0)&(pixels[:,0]<width)&(pixels[:,1]>=0)&(pixels[:,1]<height)
    return world,ProjectionResult(ids[inside],pixels[inside],z[ids[inside]])


def cloud_xyz(msg):
    """Decode padded, organized PointCloud2 without assuming XYZI byte layout."""
    fields={f.name:f for f in msg.fields}
    if not all(n in fields and fields[n].datatype==7 and fields[n].count==1 for n in ('x','y','z')):
        raise ValueError('XYZ float32 fields required')
    if msg.row_step<msg.width*msg.point_step or len(msg.data)<msg.row_step*msg.height:
        raise ValueError('PointCloud2 buffer/row layout invalid')
    if not msg.width or not msg.height:return np.empty((0,3))
    endian='>' if msg.is_bigendian else '<'
    dtype=np.dtype({'names':['x','y','z'],'formats':[endian+'f4']*3,
                    'offsets':[fields[n].offset for n in ('x','y','z')],'itemsize':msg.point_step})
    view=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,strides=(msg.row_step,msg.point_step))
    return np.column_stack([view[n].ravel() for n in ('x','y','z')]).astype(float)
