"""Measured fused obstacles only; no simulator identities or truth inputs."""
from dataclasses import dataclass
import numpy as np


from evtol_targeting.experiment_options import DYNAMIC_DEFAULTS as DEFAULTS


@dataclass(frozen=True)
class Obstacle:
    track_id: int
    position: np.ndarray  # ENU relative to ego at the control instant
    velocity: np.ndarray
    radius: float
    position_std: float = 0.
    velocity_std: float = 0.


def candidate_clearance(commands, measured, obstacles, acceleration, options, delay=0.):
    """Continuous CPA per linear interval, including response/command delay.

    Interval curvature is conservatively covered by a*dt^2/8. Candidates
    represent reachable velocity commands, not instantaneous ego velocities.
    Returned clearance is relative to already inflated obstacle envelopes.
    """
    commands=np.asarray(commands,float).reshape(-1,3)
    if not obstacles:return np.full(len(commands),np.inf)
    h=float(options['horizon_s']);count=max(1,int(np.ceil(h/options['step_s'])))
    step=h/count;ego=np.zeros_like(commands);vel=np.broadcast_to(measured,commands.shape).copy()
    result=np.full(len(commands),np.inf)
    pos=np.asarray([o.position for o in obstacles]);ov=np.asarray([o.velocity for o in obstacles])
    radius=np.asarray([o.radius for o in obstacles])
    ps=np.asarray([o.position_std for o in obstacles]);vs=np.asarray([o.velocity_std for o in obstacles])
    # Split at the actual execution delay, never apply new commands early.
    times=np.unique(np.r_[np.linspace(0,h,count+1),min(h,max(0.,delay))])
    for ta,tb in zip(times[:-1],times[1:]):
        dt=tb-ta;dv=commands-vel
        next_vel=vel if ta<delay-1e-12 else vel+dv*np.minimum(1.,acceleration*dt/np.maximum(np.linalg.norm(dv,axis=1),1e-12))[:,None]
        end=ego+(vel+next_vel)*(.5*dt)
        r=pos[None,:,:]+ov[None,:,:]*ta-ego[:,None,:]
        dr=ov[None,:,:]*dt-(end-ego)[:,None,:]
        fraction=np.clip(-np.sum(r*dr,axis=2)/np.maximum(np.sum(dr*dr,axis=2),1e-12),0.,1.)
        distance=np.linalg.norm(r+dr*fraction[:,:,None],axis=2)
        inflated=radius+options['sigma_scale']*(ps+vs*tb)+acceleration*dt*dt/8
        result=np.minimum(result,np.min(distance-inflated[None,:],axis=1))
        ego=end;vel=next_vel.copy()
    return result


def from_targets(targets, array_stamp_ns, now_ns, ego_position, options):
    """Reject invalid state; tentative/uncertain velocities remain conservative."""
    result=[];rejected=0
    for target in targets:
        stamp=target.last_measurement_stamp.sec*1_000_000_000+target.last_measurement_stamp.nanosec
        age=(now_ns-stamp)*1e-9
        if not 0<=age<=options['max_age_s']:rejected+=1;continue
        p=np.array([target.position.x,target.position.y,target.position.z],float)
        v=np.array([target.velocity.x,target.velocity.y,target.velocity.z],float)
        size=np.array([target.size.x,target.size.y,target.size.z],float)
        covariance=np.asarray(target.covariance,float).reshape(6,6)
        if not all(np.isfinite(x).all() for x in (p,v,size,covariance)) or np.any(size<0):
            rejected+=1;continue
        if np.linalg.eigvalsh((covariance+covariance.T)/2).min()< -1e-7:
            rejected+=1;continue
        ps=float(np.sqrt(max(0.,np.linalg.eigvalsh((covariance[:3,:3]+covariance[:3,:3].T)/2).max())))
        vs=float(np.sqrt(max(0.,np.linalg.eigvalsh((covariance[3:,3:]+covariance[3:,3:].T)/2).max())))
        if target.state=='tentative' or vs>options['max_velocity_std_mps']:
            # Do not ignore new targets; uncertain motion expands their envelope.
            vs=max(vs,options['max_velocity_std_mps']);v=np.zeros(3)
        delta=(now_ns-array_stamp_ns)*1e-9
        if delta<-.05 or delta>options['max_age_s']:rejected+=1;continue
        p=p+v*max(0.,delta)-ego_position
        radius=options['ego_radius_m']+options['clearance_m']+.5*np.linalg.norm(np.maximum(size,options['min_size_m']))
        result.append(Obstacle(int(target.track_id),p,v,float(radius),ps,vs))
    if len(result)>options['max_targets']:raise ValueError('Fused obstacle budget exceeded')
    return result,rejected
