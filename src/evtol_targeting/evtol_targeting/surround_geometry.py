"""Shared, explicit geometry for acquisition and multi-view localization."""
import math
import numpy as np
from evtol_simulation.coordinates import extrinsic, quaternion_matrix, matrix_quaternion


from evtol_simulation.surround_settings import camera_settings, lidar_settings


def optical_pose(view):
    t,q=extrinsic([0,0,0],view['rpy_deg'])
    return t,matrix_quaternion(quaternion_matrix(q)@quaternion_matrix([-.5,.5,-.5,.5]))


def intrinsics(s):
    f=s['width']/(2*math.tan(math.radians(s['horizontal_fov_deg'])/2))
    return np.array([[f,0,s['width']/2],[0,f,s['height']/2],[0,0,1.]])


def merge_views(targets, distance):
    """Fuse overlapping camera observations, never merge distinct classes."""
    result=[]
    for t in sorted(targets,key=lambda t:-t.lidar_point_count):
        match=next((u for u in result if u.class_id==t.class_id and
            np.linalg.norm(np.asarray(u.position_body_m)-t.position_body_m)<distance),None)
        if match is None:result.append(t)
    return result
