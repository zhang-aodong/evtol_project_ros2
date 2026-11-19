"""Explicit sensor settings; no perception or truth processing."""

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


