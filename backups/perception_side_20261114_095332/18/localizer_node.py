"""Visual semantics + real LiDAR depth, using acquisition-time ego poses."""
from collections import deque
import copy
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, CameraInfo
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from evtol_perception_interfaces.msg import SemanticFrame, ReferenceOrigin, TargetArray, Target
from evtol_simulation.coordinates import extrinsic, quaternion_matrix, matrix_quaternion
from .common import args_config, ns, stamp, header, xyz, put, clone, LATCHED, Health
from .geometry import PoseHistory, compensated_projection, cloud_xyz
from .clustering import extract_visible_targets
from .tracking import MultiTargetTracker


class Localizer(Node):
    def __init__(self,c):
        super().__init__('target_localizer');self.c=c;root='/'+c['namespace']
        self.health=Health(self,root+'/target_localizer');self.origin=None;self.info=None
        self.poses=PoseHistory(c['fusion']['pose_max_gap_s']);self.clouds=deque(maxlen=64)
        self.pending=deque(maxlen=4);self.last_lidar_ns=0;self.last_image_ns=0
        self.tracker=MultiTargetTracker(**c['tracking']);self.meta={};self.template=None
        lc=c['extrinsics']['lidar'];cc=c['extrinsics']['camera']
        self.body_lidar=extrinsic(lc['xyz_frd_m'],lc['rpy_deg'])
        tc,qc=extrinsic(cc['xyz_frd_m'],cc['rpy_deg'])
        self.body_optical=(tc,matrix_quaternion(quaternion_matrix(qc)@quaternion_matrix([-.5,.5,-.5,.5])))
        self.measurements=self.create_publisher(TargetArray,root+'/perception/targets_measurements',qos_profile_sensor_data)
        self.tracks=self.create_publisher(TargetArray,root+'/perception/tracks',qos_profile_sensor_data)
        self.markers=self.create_publisher(MarkerArray,root+'/perception/markers',qos_profile_sensor_data)
        self.create_subscription(ReferenceOrigin,'/simulation/reference_origin',self.receive_origin,LATCHED)
        self.create_subscription(Odometry,root+'/state/odometry',self.receive_pose,qos_profile_sensor_data)
        self.create_subscription(CameraInfo,root+'/camera/rgb/camera_info',self.receive_info,qos_profile_sensor_data)
        self.create_subscription(PointCloud2,root+'/lidar/points',self.receive_cloud,qos_profile_sensor_data)
        self.create_subscription(SemanticFrame,root+'/perception/semantic',self.receive_mask,qos_profile_sensor_data)
        self.create_timer(.01,self.process);self.create_timer(.05,self.predict)

    def receive_origin(self,msg):
        if self.origin is None or self.origin.session_id!=msg.session_id:
            self.poses.samples.clear();self.clouds.clear();self.pending.clear();self.meta.clear()
            self.tracker.tracks.clear();self.tracker.last_timestamp_ns=None
            self.last_lidar_ns=self.last_image_ns=0;self.template=None
        self.origin=msg

    def receive_info(self,msg):self.info=msg

    def receive_pose(self,msg):
        if self.origin is None or msg.header.frame_id!='map_enu':return
        q=msg.pose.pose.orientation
        try:self.poses.add(ns(msg.header.stamp),xyz(msg.pose.pose.position),[q.x,q.y,q.z,q.w])
        except ValueError as exc:self.health.error(exc)

    def receive_cloud(self,msg):
        if msg.header.frame_id!=self.c['namespace']+'/lidar_link':
            self.health.error('LiDAR frame differs from configured extrinsics');return
        self.clouds.append(msg)

    def receive_mask(self,msg):
        if ns(msg.header.stamp)<=self.last_image_ns:self.health.dropped+=1;return
        if len(self.pending)==self.pending.maxlen:self.health.dropped+=1
        self.pending.append((time.perf_counter(),msg))

    def result(self,t,mask,status):
        return TargetArray(header=header(t),session_id=self.origin.session_id,
            image_stamp=clone(mask.header.stamp),lidar_stamp=stamp(t),status=status)

    def process(self):
        if not self.pending or self.origin is None or self.info is None:return
        begun,mask=self.pending[0];tc=ns(mask.header.stamp)
        if not self.clouds:return
        fresh=[m for m in self.clouds if ns(m.header.stamp)>self.last_lidar_ns]
        cloud=min(fresh,key=lambda m:abs(ns(m.header.stamp)-tc)) if fresh else None
        if cloud is None or abs(ns(cloud.header.stamp)-tc)>self.c['fusion']['max_sync_delta_ms']*1e6:
            if time.perf_counter()-begun<self.c['fusion']['wait_timeout_s']:return
            self.pending.popleft();self.last_image_ns=tc;self.health.dropped+=1
            self.measurements.publish(self.result(tc,mask,'NO_SYNCHRONIZED_LIDAR'));return
        tl=ns(cloud.header.stamp);lp=self.poses.at(tl);cp=self.poses.at(tc)
        if lp is None or cp is None:
            if time.perf_counter()-begun<self.c['fusion']['wait_timeout_s']:return
            self.pending.popleft();self.last_image_ns=tc;self.health.dropped+=1
            self.measurements.publish(self.result(tl,mask,'NO_BRACKETED_POSE'));return
        self.pending.popleft();self.last_image_ns=tc
        try:
            if mask.width*mask.height!=len(mask.class_ids):raise ValueError('Semantic dimensions mismatch')
            labels=np.asarray(mask.class_ids,dtype=np.int32).reshape(mask.height,mask.width)
            points=cloud_xyz(cloud)
            k=np.asarray(self.info.k).reshape(3,3).copy()
            k[0,:]*=mask.width/self.info.width;k[1,:]*=mask.height/self.info.height
            world,projection=compensated_projection(points,lp,cp,self.body_lidar,self.body_optical,k,mask.width,mask.height)
            d=self.c['detection']
            targets=extract_visible_targets(world,projection,labels,np.full(labels.shape,np.nan,dtype=np.float32),
                d['target_class_ids'],min_lidar_points=d['min_lidar_points'],min_mask_pixels=d['min_mask_pixels'],
                cluster_radius_m=d['cluster_radius_m'])
            result=self.result(tl,mask,'OK' if targets else ('EMPTY_LIDAR' if not len(points) else 'NO_TARGET_DEPTH'))
            result.sync_delta_ms=abs(tc-tl)*1e-6
            observations=[dict(class_id=t.class_id,position_enu_m=list(t.position_body_m)) for t in targets]
            detections,_=self.tracker.update(observations,tl)
            for detection,target in zip(detections,targets):
                tid=detection['track_id'];self.meta[tid]=(target.lidar_point_count,
                    np.asarray(target.point_max_body_m)-np.asarray(target.point_min_body_m))
                out=self.target(self.tracker.tracks[tid],tl,True)
                put(out.position,target.position_body_m)
                result.targets.append(out)
            self.meta={key:value for key,value in self.meta.items() if key in self.tracker.tracks}
            self.last_lidar_ns=tl;self.measurements.publish(result);self.template=result
            self.health.ok()
        except Exception as exc:self.health.error(exc)

    def target(self,track,t,measured):
        dt=max(0.,(t-track['timestamp_ns'])*1e-9)
        f=np.eye(6);f[:3,3:]=np.eye(3)*dt
        out=Target(track_id=track['id'],class_id=track['class_id'],reference_kind='visible_surface_center',
            state=('confirmed' if measured else 'predicted') if track['hits']>=3 else 'tentative',
            measured=measured,last_measurement_stamp=stamp(track['last_observed_ns']),orientation_valid=False)
        put(out.position,track['x'][:3]+track['x'][3:]*dt);put(out.velocity,track['x'][3:])
        cov=f@track['p']@f.T
        g=np.vstack((np.eye(3)*dt*dt/2,np.eye(3)*dt))
        cov+=g@g.T*self.tracker.acceleration_variance
        if track.get('ambiguous'):cov*=4.
        out.covariance=cov.ravel().tolist()
        count,size=self.meta.get(track['id'],(0,np.zeros(3)));out.point_count=count;put(out.size,size)
        return out

    def predict(self):
        if self.template is None:return
        now=self.get_clock().now().nanoseconds
        if now<ns(self.template.header.stamp):return
        result=clone(self.template);result.header=header(now);result.targets=[];result.status='PREDICTION'
        for track in self.tracker.tracks.values():
            if 0<=now-track['last_observed_ns']<=self.tracker.max_age_ns:
                result.targets.append(self.target(track,now,False))
        self.tracks.publish(result)
        markers=[Marker(action=Marker.DELETEALL)]
        for target in result.targets:
            m=Marker(header=clone(result.header),ns='target_tracks',id=int(target.track_id),type=Marker.SPHERE,action=Marker.ADD)
            m.pose.position=clone(target.position);m.pose.orientation.w=1.
            m.scale.x=m.scale.y=m.scale.z=.8;m.color.r=1.;m.color.g=.5;m.color.a=.8
            m.lifetime.sec=1;markers.append(m)
            text=clone(m);text.ns='target_labels';text.type=Marker.TEXT_VIEW_FACING;text.scale.z=.6
            text.pose.position.z+=.8;text.text=f'{target.track_id} class={target.class_id} predicted';markers.append(text)
        self.markers.publish(MarkerArray(markers=markers))


def main(args=None):
    c,ros=args_config(args);rclpy.init(args=ros);node=Localizer(c)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
