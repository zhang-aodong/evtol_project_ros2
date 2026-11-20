"""15-state right-error ESKF: [p,v,theta,ba,bg], FLU -> ENU.

The error dynamics and positive process-noise quadrature follow the handoff's
imu_covariance.hpp. The range residual follows FiveGRangeCost, with a causal
innovation gate and Cauchy variance inflation instead of batch Ceres factors.
"""
import math
import numpy as np
from .options import options


def skew(v):
    x,y,z=v
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])


def multiply(a,b):
    av,bv=np.asarray(a[:3]),np.asarray(b[:3])
    q=np.r_[a[3]*bv+b[3]*av+np.cross(av,bv),a[3]*b[3]-av@bv]
    return q/np.linalg.norm(q)


def expq(v):
    n=np.linalg.norm(v)
    return np.r_[np.asarray(v)*(.5 if n<1e-9 else math.sin(n/2)/n),math.cos(n/2)]


def rotation(q):
    v=np.asarray(q[:3]);w=q[3];s=skew(v)
    return np.eye(3)+2*w*s+2*s@s


def transition(f,dt):
    a=f*dt;a2=a@a
    return np.eye(15)+a+a2*.5+a2@a/6+a2@a2/24


class ESKF:
    def __init__(self,config=None):
        self.o=options(config);self.t=None
        self.p=np.zeros(3);self.v=np.zeros(3);self.q=np.array([0.,0.,0.,1.])
        self.ba=np.zeros(3);self.bg=np.zeros(3);self.P=np.eye(15)
        self.g=np.array([0.,0.,-self.o['gravity_mps2']])

    def initialize(self,t,position,q,covariance,ba=None,bg=None,velocity=None,stationary=False):
        q=np.asarray(q,float)
        if not np.isfinite(q).all() or not .5<np.linalg.norm(q)<1.5:raise ValueError('IMU orientation unavailable')
        self.t=int(t);self.p=np.asarray(position,float).copy();self.q=q/np.linalg.norm(q)
        self.v=np.zeros(3) if velocity is None else np.asarray(velocity,float).copy()
        self.ba=np.zeros(3) if ba is None else np.asarray(ba,float).copy()
        self.bg=np.zeros(3) if bg is None else np.asarray(bg,float).copy()
        self.P=np.diag(np.repeat([1.,.05**2 if stationary else self.o['initial_velocity_std_mps']**2,
            math.radians(self.o['initial_attitude_std_deg'])**2,self.o['initial_accel_bias_std']**2,
            self.o['initial_gyro_bias_std']**2],3))
        self.P[:3,:3]=covariance

    def propagate(self,t,acceleration,angular):
        t=int(t)
        if self.t is None:raise ValueError('Not initialized')
        if t<self.t:raise ValueError('State time reversal')
        dt=(t-self.t)*1e-9
        if dt>self.o['max_imu_gap_s']:raise ValueError('IMU integration gap')
        if not dt:return
        a=np.asarray(acceleration,float)-self.ba;w=np.asarray(angular,float)-self.bg
        if not np.isfinite(np.r_[a,w]).all():raise ValueError('Non-finite IMU')
        steps=max(1,int(math.ceil(dt/.005)));h=dt/steps
        for _ in range(steps):
            r=rotation(multiply(self.q,expq(w*h*.5)))
            aw=r@a+self.g
            self.p+=self.v*h+.5*aw*h*h;self.v+=aw*h
            f=np.zeros((15,15));f[:3,3:6]=np.eye(3)
            f[3:6,6:9]=-r@skew(a);f[3:6,9:12]=-r
            f[6:9,6:9]=-skew(w);f[6:9,12:15]=-np.eye(3)
            noise=np.zeros((15,15));noise[3:6,3:6]=np.eye(3)*self.o['accel_noise']**2
            noise[6:9,6:9]=np.eye(3)*self.o['gyro_noise']**2
            noise[9:12,9:12]=np.eye(3)*self.o['accel_bias_noise']**2
            noise[12:15,12:15]=np.eye(3)*self.o['gyro_bias_noise']**2
            phi=transition(f,h);qd=np.zeros((15,15))
            for x,weight in ((.5-math.sqrt(15)/10,5/18),(.5,4/9),(.5+math.sqrt(15)/10,5/18)):
                e=transition(f,x*h);qd+=h*weight*(e@noise@e.T)
            self.P=phi@self.P@phi.T+qd
            self.q=multiply(self.q,expq(w*h))
        self.P=(self.P+self.P.T)*.5;self.t=t

    def update(self,residual,H,covariance,gate,robust=None):
        residual=np.atleast_1d(residual);S=H@self.P@H.T+covariance
        nis=float(residual@np.linalg.solve(S,residual))
        if not math.isfinite(nis) or nis>gate:return dict(accepted=False,reason='innovation_gate',nis=nis,jump_m=0.)
        if robust:
            norm=float(residual@np.linalg.solve(covariance,residual))
            covariance=covariance*(1+norm/(robust*robust));S=H@self.P@H.T+covariance
        K=np.linalg.solve(S,H@self.P).T;dx=K@residual
        self.p+=dx[:3];self.v+=dx[3:6];self.q=multiply(self.q,expq(dx[6:9]))
        self.ba+=dx[9:12];self.bg+=dx[12:15]
        a=np.eye(15)-K@H;self.P=a@self.P@a.T+K@covariance@K.T
        reset=np.eye(15);reset[6:9,6:9]-=.5*skew(dx[6:9])
        self.P=reset@self.P@reset.T;self.P=(self.P+self.P.T)*.5
        return dict(accepted=True,reason='accepted',nis=nis,jump_m=float(np.linalg.norm(dx[:3])))

    def gnss(self,position,covariance):
        lever=np.asarray(self.o['gnss_lever_flu_m']);r=rotation(self.q)
        H=np.zeros((3,15));H[:,:3]=np.eye(3);H[:,6:9]=-r@skew(lever)
        return self.update(np.asarray(position)-self.p-r@lever,H,covariance,self.o['gnss_nis_gate'])

    def range(self,anchor,distance,sigma):
        lever=np.asarray(self.o['fiveg_lever_flu_m']);r=rotation(self.q)
        delta=self.p+r@lever-np.asarray(anchor);pred=float(np.linalg.norm(delta))
        if pred<1e-6:return dict(accepted=False,reason='anchor_coincident',jump_m=0.)
        direction=delta/pred;H=np.zeros((1,15));H[0,:3]=direction;H[0,6:9]=-direction@r@skew(lever)
        variance=max(sigma,self.o['fiveg_sigma_floor_m'])**2
        return self.update([distance-pred],H,np.array([[variance]]),self.o['fiveg_nis_gate'],self.o['fiveg_cauchy_scale'])

    def body_output(self,angular):
        r=rotation(self.q);poseidx=[0,1,2,6,7,8]
        pose_cov=self.P[np.ix_(poseidx,poseidx)].copy()
        # Quaternion covariance is a right/body error; ROS pose rotation error
        # is expressed on the parent axes, hence rotate both cross-blocks.
        T=np.eye(6);T[3:,3:]=r;pose_cov=T@pose_cov@T.T
        # Body velocity depends on attitude as well as world velocity. Retain
        # their cross-covariance and the shared gyro-bias uncertainty.
        J=np.zeros((6,15));J[:3,3:6]=r.T;J[:3,6:9]=skew(r.T@self.v);J[3:,12:15]=-np.eye(3)
        twist_cov=J@self.P@J.T
        twist_cov[3:,3:]+=np.eye(3)*self.o['gyro_noise']**2
        return r.T@self.v,np.asarray(angular)-self.bg,pose_cov,twist_cov
