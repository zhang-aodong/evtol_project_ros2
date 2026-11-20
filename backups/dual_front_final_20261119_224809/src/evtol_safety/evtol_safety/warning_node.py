"""Separate 5 m lidar and 10 m forward fusion warnings; never subscribes to target truth."""
import json
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from evtol_perception_interfaces.msg import ReferenceOrigin,TargetArray,StateStatus
from evtol_targeting.common import args_config,ns,xyz,odom_arrays,LATCHED,LATEST,Health
from evtol_targeting.geometry import PoseHistory,cloud_xyz
from .warnings import DEFAULTS,proximity,predict_target


class WarningNode(Node):
    def __init__(self,c):
        super().__init__('obstacle_warnings');self.c=c;self.o={**DEFAULTS,**c.get('warnings',{})}
        self.origin=None;self.pose=None;self.pose_wall=0.;self.cloud=None;self.cloud_wall=0.
        self.tracks=None;self.track_wall=0.;self.ground=None;self.valid_state=False
        self.history=PoseHistory(c['fusion']['pose_max_gap_s']);root='/'+c['namespace']
        self.health=Health(self,root+'/warnings')
        self.lidar_pub=self.create_publisher(String,root+'/warning/lidar',10)
        self.fusion_pub=self.create_publisher(String,root+'/warning/fusion',10)
        self.create_subscription(ReferenceOrigin,'/simulation/reference_origin',self.receive_origin,LATCHED)
        self.create_subscription(Odometry,root+'/state/odometry',self.receive_pose,LATEST)
        self.create_subscription(StateStatus,root+'/state/status',lambda m:setattr(self,'valid_state',bool(self.origin and m.session_id==self.origin.session_id and m.valid)),10)
        self.create_subscription(PointCloud2,root+'/navigation/lidar/points',self.receive_cloud,LATEST)
        self.create_subscription(TargetArray,root+'/perception/tracks',self.receive_tracks,LATEST)
        self.steady=Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.1,self.tick,clock=self.steady)

    def receive_origin(self,m):
        if self.origin is None or self.origin.session_id!=m.session_id:
            self.pose=None;self.cloud=None;self.tracks=None;self.ground=None;self.valid_state=False;self.history.samples.clear()
        self.origin=m

    def receive_pose(self,m):
        if self.origin is None or m.header.frame_id!='map_enu':return
        q=m.pose.pose.orientation
        self.history.add(ns(m.header.stamp),xyz(m.pose.pose.position),[q.x,q.y,q.z,q.w])
        self.pose=m;self.pose_wall=time.monotonic()

    def receive_cloud(self,m):
        if m.header.frame_id==self.c['namespace']+'/base_link':self.cloud=m;self.cloud_wall=time.monotonic()

    def receive_tracks(self,m):
        if self.origin and m.session_id==self.origin.session_id and m.header.frame_id=='map_enu':
            self.tracks=m;self.track_wall=time.monotonic()

    def tick(self):
        if self.origin is None:return
        now=self.get_clock().now().nanoseconds;wall=time.monotonic()
        base=dict(schema_version=1,session_id=self.origin.session_id,stamp_ns=now)
        lidar=dict(base,valid=False,enabled=False,warning=False,reason='stale_state')
        fusion=dict(base,valid=False,warning=False,targets=[],reason='stale_state',radius_m=self.o['fusion_radius_m'],region='front_hemisphere')
        try:
            if not self.valid_state or self.pose is None or wall-self.pose_wall>self.c['state']['timeout_s']:raise ValueError('stale_state')
            if not 0<=(now-ns(self.pose.header.stamp))*1e-9<=self.c['state']['timeout_s']:raise ValueError('stale_state_stamp')
            p,r,v=odom_arrays(self.pose);forward=r[:,0]
            lidar['reason']='stale_lidar'
            if self.cloud is not None and wall-self.cloud_wall<=self.c['safety']['cloud_timeout_s']:
                stamp=ns(self.cloud.header.stamp);pose=self.history.at(stamp)
                if pose is not None and 0<=(now-stamp)*1e-9<=self.c['safety']['cloud_timeout_s']:
                    raw=cloud_xyz(self.cloud);raw=raw[np.isfinite(raw).all(axis=1)]
                    world=raw@pose[1].T+pose[0];relative=world-p
                    column=relative[(np.linalg.norm(relative[:,:2],axis=1)<=self.o['ground_column_radius_m'])&(relative[:,2]<-.3)]
                    if len(column)>=self.o['ground_min_points']:
                        # Nearest downward surface is the local ground/roof;
                        # ENU launch height alone is not an AGL measurement.
                        self.ground=(wall,p[2]+float(np.quantile(column[:,2],.8)),p[:2].copy())
                    agl=None
                    if self.ground and wall-self.ground[0]<=self.o['ground_max_age_s'] and np.linalg.norm(p[:2]-self.ground[2])<=self.o['ground_column_radius_m']:
                        agl=max(0.,float(p[2]-self.ground[1]))
                    lidar=dict(base,**proximity(relative,agl,self.o),scan_stamp_ns=stamp)
            fresh=bool(self.tracks is not None and wall-self.track_wall<=self.o['max_measurement_age_s']
                and 0<=(now-ns(self.tracks.image_stamp))*1e-9<=self.o['max_measurement_age_s'])
            fusion.update(valid=fresh,reason='ok' if fresh else 'stale_fusion')
            if fresh:
                rows=[]
                for t in self.tracks.targets:
                    age=(now-ns(t.last_measurement_stamp))*1e-9
                    if t.class_id!=6 or not 0<=age<=self.o['max_measurement_age_s']:continue
                    position=xyz(t.position)+xyz(t.velocity)*max(0.,(now-ns(self.tracks.header.stamp))*1e-9)
                    row=predict_target(position,xyz(t.velocity),p,v,forward,self.o)
                    rows.append(dict(track_id=int(t.track_id),last_measurement_stamp_ns=ns(t.last_measurement_stamp),
                        position_enu_m=position.tolist(),velocity_enu_mps=xyz(t.velocity).tolist(),**row))
                fusion.update(targets=rows,warning=any(t['warning'] for t in rows),warning_count=sum(t['warning'] for t in rows))
            self.health.ok();self.health.details=dict(lidar_valid=lidar['valid'],lidar_warning=lidar['warning'],
                fusion_valid=fusion['valid'],fusion_warnings=fusion.get('warning_count',0))
        except Exception as exc:
            lidar['reason']=fusion['reason']=str(exc);self.health.error(exc)
        self.lidar_pub.publish(String(data=json.dumps(lidar,allow_nan=False)))
        self.fusion_pub.publish(String(data=json.dumps(fusion,allow_nan=False)))


def main(args=None):
    c,ros=args_config(args);rclpy.init(args=ros);node=WarningNode(c)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
