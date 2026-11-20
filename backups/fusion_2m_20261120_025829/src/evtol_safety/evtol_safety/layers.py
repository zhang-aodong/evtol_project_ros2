"""Separate raw emergency returns from ordinary fused-object planning."""
import numpy as np
from .corridor import local_points


def emergency_points(world, position, radius=2.):
    return local_points(world,position,radius)


def fused_route_points(obstacles, position, horizon):
    # A single centre point under-represents a drone's collision envelope.
    # Sample its boundary at each prediction slice; the voxel planner adds
    # its own ego clearance, so subtract that part from the obstacle radius.
    axes=np.asarray([[x,y,z] for x in (-1,0,1) for y in (-1,0,1) for z in (-1,0,1) if x or y or z],float)
    axes/=np.linalg.norm(axes,axis=1)[:,None]
    result=[]
    for o in obstacles:
        radius=max(.1,o.radius-.8+2*o.position_std)
        for dt in (0.,horizon*.5,horizon):
            centre=np.asarray(position)+o.position+o.velocity*dt
            result.extend([centre,*list(centre+axes*radius)])
    return np.asarray(result,float).reshape(-1,3)


def descent_limit(position, target, anchor, phase, ground, clearance, braking):
    """Allow planned descents; keep evasion above the current leg's floor."""
    if phase=='takeoff':return 0.
    floor=ground+clearance
    if phase=='waypoint' and anchor is not None:
        floor=max(floor,min(float(anchor[2]),float(target[2]))-.4)
    gap=max(0.,float(position[2])-floor)
    return -min(np.sqrt(2*braking*gap),gap*2.)
