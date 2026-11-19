"""All-round measured LiDAR clusters and motion; no semantic or simulator IDs.

This is the independent safety fallback. It never claims an unclassified
cluster is a visually detected aircraft or contributes to semantic recall.
"""
from collections import deque
import time
import numpy as np
from .clustering import _density_clusters
from .tracking import MultiTargetTracker


DEFAULTS=dict(enabled=True,rate_hz=10.,cluster_radius_m=.8,
              min_points=3,max_extent_m=4.,range_m=30.,max_age_s=.4,
              scan_window_s=.12,measurement_std_m=.15,min_height_enu_m=.35,
              motion_window_s=.8,motion_min_span_s=.35,motion_min_speed_mps=.5,
              motion_max_residual_m=.25,motion_sigma=2.)


class LidarMotionTracker:
    def __init__(self,options=None):
        self.o={**DEFAULTS,**(options or {})}
        self.tracker=MultiTargetTracker(max_age_s=self.o['max_age_s'],
            association_distance_m=2.,measurement_std_m=self.o['measurement_std_m'],
            acceleration_std_mps2=3.)
        self.meta={};self.observations={};self.motion={};self.last_ns=0;self.stats={}

    def update(self,world,timestamp_ns,ego_position):
        if timestamp_ns<=self.last_ns:return False
        self.last_ns=timestamp_ns
        points=np.asarray(world,float).reshape(-1,3)
        points=points[np.isfinite(points).all(axis=1)]
        # This explicit map-relative ground gate is ONLY for compact airborne
        # motion tracking. Raw 360-degree returns still protect against every
        # ground/wall/low obstacle in SafetyNode, without this filtering.
        points=points[points[:,2]>=self.o['min_height_enu_m']]
        distance=np.linalg.norm(points-np.asarray(ego_position),axis=1)
        points=points[(distance>.8)&(distance<=self.o['range_m'])]
        if len(points)>65536:raise ValueError('LiDAR motion point budget exceeded')
        # The clustering helper voxelizes internally and preserves population;
        # do not discard sparse small-aircraft returns before min_points checks.
        labels=_density_clusters(points,self.o['cluster_radius_m'],int(self.o['min_points']))
        observations=[];sizes=[];counts=[];large=0
        for label in np.unique(labels):
            if label<0:continue
            group=points[labels==label];size=np.ptp(group,axis=0)
            # Ground, large walls and static aircraft remain raw-cloud obstacles;
            # they must not become small moving-aircraft tracks.
            if np.max(size)>self.o['max_extent_m']:
                large+=1;continue
            observations.append(dict(class_id=0,position_enu_m=np.median(group,axis=0).tolist()))
            sizes.append(size);counts.append(len(group))
        if len(observations)>128:raise ValueError('LiDAR compact obstacle budget exceeded')
        measured,_=self.tracker.update(observations,timestamp_ns)
        if len(self.tracker.tracks)>128:raise ValueError('LiDAR active obstacle budget exceeded')
        for detection,size,count in zip(measured,sizes,counts):
            key=detection['track_id'];self.meta[key]=(count,size)
            history=self.observations.setdefault(key,deque())
            history.append((timestamp_ns,np.array(detection['position_enu_m'])))
            while history and (timestamp_ns-history[0][0])*1e-9>self.o['motion_window_s']:history.popleft()
        self.meta={k:v for k,v in self.meta.items() if k in self.tracker.tracks}
        self.observations={k:v for k,v in self.observations.items() if k in self.tracker.tracks}
        self.motion={}
        for key,history in self.observations.items():
            if len(history)<4:continue
            ts=np.array([(t-timestamp_ns)*1e-9 for t,p in history]);span=np.ptp(ts)
            if span<self.o['motion_min_span_s']:continue
            centered=ts-ts.mean();ss=float(centered@centered)
            positions=np.array([p for t,p in history]);mean=positions.mean(axis=0)
            velocity=centered@(positions-mean)/ss
            residual=positions-mean-centered[:,None]*velocity
            rms=float(np.sqrt(np.mean(np.sum(residual**2,axis=1))))
            # A surface fragment is not evidence of a moving aircraft. Raw
            # returns remain in static collision checking throughout this
            # confirmation interval, including new/tentative clusters.
            variance=max(self.o['measurement_std_m']**2,
                         float(np.sum(residual**2))/max(1,3*(len(history)-2)))
            velocity_std=np.sqrt(variance/ss);speed=float(np.linalg.norm(velocity))
            moving=(rms<=self.o['motion_max_residual_m'] and
                    speed>max(self.o['motion_min_speed_mps'],self.o['motion_sigma']*velocity_std))
            if moving:self.motion[key]=(velocity,float(velocity_std))
        self.stats=dict(points=len(points),compact_clusters=len(measured),large_clusters=large,
                        active_tracks=len(self.tracker.tracks),moving_tracks=len(self.motion))
        return True


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock,ClockType
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from visualization_msgs.msg import Marker,MarkerArray
    from evtol_perception_interfaces.msg import TargetArray,Target,ReferenceOrigin
    from .common import args_config,LATCHED,LATEST,Health,ns,xyz,header,stamp,put
    from .geometry import PoseHistory,cloud_xyz
    from evtol_simulation.coordinates import quaternion_matrix
    from evtol_simulation.runtime import TimerResolution

    class LidarObstacles(Node):
        def __init__(self,c):
            super().__init__('lidar_motion_obstacles');self.c=c;self.origin=None
            self.o={**DEFAULTS,**c.get('lidar_obstacles',{})};self.motion=LidarMotionTracker(self.o)
            self.history=PoseHistory(c['fusion']['pose_max_gap_s']);self.clouds=deque(maxlen=8)
            self.health=Health(self,'/'+c['namespace']+'/lidar_obstacles');root='/'+c['namespace']
            self.pub=self.create_publisher(TargetArray,root+'/perception/lidar_obstacles',LATEST)
            self.markers=self.create_publisher(MarkerArray,root+'/perception/lidar_obstacle_markers',LATEST)
            self.create_subscription(ReferenceOrigin,'/simulation/reference_origin',self.receive_origin,LATCHED)
            self.create_subscription(Odometry,root+'/state/odometry',self.receive_pose,LATEST)
            self.create_subscription(PointCloud2,root+'/navigation/lidar/points',self.receive_cloud,LATEST)
            self.create_timer(1./self.o['rate_hz'],self.process,clock=Clock(clock_type=ClockType.STEADY_TIME))
        def receive_origin(self,msg):
            if self.origin is None or self.origin.session_id!=msg.session_id:
                self.motion=LidarMotionTracker(self.o);self.history.samples.clear();self.clouds.clear()
            self.origin=msg
        def receive_pose(self,msg):
            if self.origin is None or msg.header.frame_id!='map_enu':return
            q=msg.pose.pose.orientation
            self.history.add(ns(msg.header.stamp),xyz(msg.pose.pose.position),[q.x,q.y,q.z,q.w])
        def receive_cloud(self,msg):
            if msg.header.frame_id==self.c['namespace']+'/base_link':self.clouds.append(msg)
        def process(self):
            if self.origin is None:return
            try:
                now=self.get_clock().now().nanoseconds
                available=[m for m in self.clouds if 0<=(now-ns(m.header.stamp))*1e-9<=self.o['scan_window_s']
                    and self.history.at(ns(m.header.stamp)) is not None]
                if not available:return
                latest=max(available,key=lambda m:ns(m.header.stamp));t=ns(latest.header.stamp)
                if t<=self.motion.last_ns:return
                scans=[]
                for msg in available:
                    pose=self.history.at(ns(msg.header.stamp));raw=cloud_xyz(msg)
                    scans.append(raw@pose[1].T+pose[0])
                ego=self.history.at(t)[0]
                self.motion.update(np.concatenate(scans),t,ego)
                result=TargetArray(header=header(t),session_id=self.origin.session_id,lidar_stamp=stamp(t),
                    status='LIDAR_ONLY_UNCLASSIFIED')
                for track in self.motion.tracker.tracks.values():
                    if now-track['last_observed_ns']>self.motion.tracker.max_age_ns:continue
                    out=Target(track_id=1_000_000+track['id'],class_id=0,
                        reference_kind='lidar_cluster_unclassified',state='confirmed' if track['id'] in self.motion.motion else 'tentative',
                        measured=track['observed'],last_measurement_stamp=stamp(track['last_observed_ns']))
                    put(out.position,track['x'][:3]);covariance=track['p'].copy()
                    motion=self.motion.motion.get(track['id'])
                    if motion is not None:
                        put(out.velocity,motion[0])
                        covariance[:3,3:]=0.;covariance[3:,:3]=0.
                        covariance[3:,3:]=np.eye(3)*motion[1]**2
                    else:put(out.velocity,track['x'][3:])
                    out.covariance=covariance.ravel().tolist()
                    count,size=self.motion.meta.get(track['id'],(0,np.zeros(3)));out.point_count=count;put(out.size,size)
                    result.targets.append(out)
                self.pub.publish(result);self.health.ok()
                markers=[Marker(action=Marker.DELETEALL)]
                for target in result.targets:
                    marker=Marker(header=result.header,ns='lidar_unclassified',id=int(target.track_id),
                        type=Marker.CUBE,action=Marker.ADD)
                    marker.pose.position=target.position;marker.pose.orientation.w=1.
                    marker.scale.x=max(.3,target.size.x);marker.scale.y=max(.3,target.size.y);marker.scale.z=max(.3,target.size.z)
                    marker.color.b=1.;marker.color.g=.7;marker.color.a=.5;marker.lifetime.nanosec=400_000_000
                    markers.append(marker)
                self.markers.publish(MarkerArray(markers=markers))
                self.health.details=dict(**self.motion.stats,output_age_ms=round((now-t)*1e-6,2))
            except Exception as exc:self.health.error(exc)

    resolution=TimerResolution();c,ros=args_config(args);rclpy.init(args=ros);node=LidarObstacles(c)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown();resolution.close()
