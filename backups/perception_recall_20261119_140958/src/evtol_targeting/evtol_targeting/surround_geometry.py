"""Shared, explicit geometry for acquisition and multi-view localization."""
import math
import numpy as np
from evtol_simulation.coordinates import extrinsic, quaternion_matrix, matrix_quaternion


def camera_settings(s, view):
    r,p,y=view['rpy_deg']
    return dict(X=0.,Y=0.,Z=0.,Roll=r,Pitch=p,Yaw=y,CaptureSettings=[
        dict(ImageType=i,Width=s['width'],Height=s['height'],FOV_Degrees=s['horizontal_fov_deg'],MotionBlurAmount=0) for i in (0,5)])


def lidar_settings(lc):
    return dict(SensorType=6,Enabled=True,X=0,Y=0,Z=0,Roll=0,Pitch=0,Yaw=0,
        NumberOfChannels=lc['channels'],PointsPerSecond=lc['points_per_second'],
        RotationsPerSecond=lc['rate_hz'],UpdateFrequency=lc['rate_hz'],
        UseWallClockFrequency=True,CompleteScanPerUpdate=True,AngularDither=True,
        HorizontalFOVStart=-180,HorizontalFOVEnd=179.999,
        VerticalFOVUpper=lc['vertical_fov_deg']/2,VerticalFOVLower=-lc['vertical_fov_deg']/2,
        Range=lc['range_m'],DataFrame='SensorLocalFrame',DrawDebugPoints=False)


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
