"""Local collision neighbourhood, independent of long-range warning topics."""
import numpy as np


def local_points(world,position,radius):
    p=np.asarray(world,float).reshape(-1,3)
    return p[np.linalg.norm(p-position,axis=1)<=radius]


def near_obstacles(obstacles,ego_velocity,radius,horizon):
    """Keep nearby targets and targets predicted to enter the neighbourhood.

    Exact closest approach for constant measured relative velocity preserves
    fast side entries; this is not a simple current-centre distance cutoff.
    Uncertain targets retain their conservative envelope inside this local
    neighbourhood. Warning generation is not filtered by this function.
    """
    kept=[]
    for obstacle in obstacles:
        r=obstacle.position;v=obstacle.velocity-ego_velocity
        t=float(np.clip(-r@v/max(v@v,1e-12),0.,horizon))
        if np.linalg.norm(r+v*t)<=radius+obstacle.radius+2*obstacle.position_std:
            kept.append(obstacle)
    return kept


def route_aim(position,path,lookahead):
    """Geometric path following only; never skips a corner to a remote vertex.

    Collision-aware velocity selection checks current data at control rate.
    The graph search and full route validation run at planning rate.
    """
    p=np.asarray(position,float);route=np.asarray(path,float).reshape(-1,3)
    if len(route)<2:return None
    a=route[:-1];d=np.diff(route,axis=0);lengths=np.linalg.norm(d,axis=1)
    fractions=np.clip(np.einsum('ij,ij->i',p-a,d)/np.maximum(lengths**2,1e-12),0.,1.)
    projections=a+d*fractions[:,None]
    index=int(np.argmin(np.linalg.norm(projections-p,axis=1)))
    # Once the end of a segment is reached, continue on its immediate successor.
    if lengths[index]*(1-fractions[index])<.2 and index+1<len(d):
        index+=1;fractions[index]=0.
    point=a[index]+d[index]*min(1.,fractions[index]+lookahead/max(lengths[index],1e-9))
    if np.linalg.norm(point-p)<1e-6:return route[-1].copy()
    return point
