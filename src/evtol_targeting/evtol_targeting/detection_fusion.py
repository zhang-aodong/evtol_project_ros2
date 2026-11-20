"""YOLO instance boxes + acquisition-time LiDAR projection -> ENU tracks."""
from collections import deque
from types import SimpleNamespace
import json
import time
import cv2
import numpy as np
import rclpy
from std_msgs.msg import String
from .localizer_node import Localizer
from .common import args_config,header,ns,put,LATEST
from .geometry import cloud_xyz,compensated_projection
from .surround_geometry import optical_pose,intrinsics,merge_views
from .box_localization import normalize_detections,box_rays,localize_boxes
from evtol_simulation.coordinates import quaternion_matrix


class DetectionFusion(Localizer):
    def __init__(self,c):
        if c['model'].get('task')!='detect':raise ValueError('DetectionFusion requires model.task=detect')
        cv2.setNumThreads(1)
        super().__init__(c,surround=True)
        self.s=c['surround'];self.k=intrinsics(self.s)
        self.view_poses=[optical_pose(v) for v in self.s['cameras']]
        self.body_lidar=(np.zeros(3),[0,0,0,1]);self.pending=deque(maxlen=3)
        self.roi={};self.last_roi_views=None;self.last_roi_stamp=0;self.roi_requested=set()
        root='/'+c['namespace']+'/perception'
        self.create_subscription(String,root+'/surround/detections',self.receive_boxes,LATEST)
        self.create_subscription(String,root+'/surround/exposure',self.prefetch,LATEST)
        self.create_subscription(String,root+'/roi_lidar/returns',self.receive_roi,LATEST)
        self.roi_pub=self.create_publisher(String,root+'/roi_lidar/requests',LATEST)

    def receive_origin(self,msg):
        reset=self.origin is None or self.origin.session_id!=msg.session_id
        super().receive_origin(msg)
        if reset and hasattr(self,'roi'):
            self.roi.clear();self.roi_requested.clear();self.last_roi_views=None;self.last_roi_stamp=0

    def receive_roi(self,msg):
        try:
            data=json.loads(msg.data)
            if self.origin is None or data['session_id']!=self.origin.session_id:return
            self.roi[int(data['image_stamp_ns'])]=data['views']
            for key in sorted(self.roi)[:-6]:del self.roi[key]
        except (ValueError,KeyError,TypeError) as exc:self.health.error(exc)

    def receive_boxes(self,msg):
        try:
            data=json.loads(msg.data);stamp=int(data['stamp_ns'])
            if data['schema_version']!=1 or data['width']!=self.s['width'] or data['height']!=self.s['height']:
                raise ValueError('Detection frame geometry/schema mismatch')
            if data['cameras']!=[v['name'] for v in self.s['cameras']]:raise ValueError('Detection camera order mismatch')
            if stamp<=self.last_image_ns:return
            views=len(self.s['cameras'])
            if len(data['detections'])>views*self.c['model']['max_det']:raise ValueError('Detection budget exceeded')
            boxes=normalize_detections(data['detections'],self.c['model'],data['width'],data['height'],views)
            # source class 0 is mapped to canonical drone class 6 exactly once.
            if len(boxes)!=len(data['detections']):raise ValueError('Invalid detector output')
            frame=SimpleNamespace(header=header(stamp),boxes=boxes)
            if len(self.pending)==self.pending.maxlen:self.health.dropped+=1
            self.pending.append((time.perf_counter(),frame))
            if not self.s.get('roi_lidar',False) or self.origin is None:return
            views=[]
            for i,view in enumerate(self.s['cameras']):
                rays=box_rays([b for b in boxes if b['view_index']==i],self.k,self.s['width'],self.s['height'],
                    padding=self.c['detection']['box_roi_padding_px'])
                if rays:views.append(dict(camera=view['name'],rays=rays))
            self.last_roi_views=views;self.last_roi_stamp=stamp
            if stamp not in self.roi_requested:
                self.roi_pub.publish(String(data=json.dumps(dict(image_stamp_ns=stamp,session_id=self.origin.session_id,views=views))))
        except (ValueError,KeyError,TypeError) as exc:self.health.error(exc)

    def prefetch(self,msg):
        if not self.s.get('roi_prefetch',False) or self.origin is None or not self.last_roi_views:return
        try:t=int(json.loads(msg.data)['stamp_ns'])
        except (ValueError,KeyError,TypeError):return
        if not 0<t-self.last_roi_stamp<=350_000_000:return
        old=self.poses.at(self.last_roi_stamp);current=self.poses.at(t)
        if current is None and self.poses.samples and 0<=t-self.poses.samples[-1][0]<=100_000_000:
            current=self.poses.at(self.poses.samples[-1][0])
        if old is None or current is None:return
        views=[]
        for view in self.last_roi_views:
            i=next(i for i,v in enumerate(self.s['cameras']) if v['name']==view['camera'])
            rbc=quaternion_matrix(self.view_poses[i][1]);rays=np.asarray(view['rays'],float)
            directions=np.column_stack((rays,np.ones(len(rays))))@rbc.T@old[1].T@current[1]@rbc
            directions=directions[directions[:,2]>.1];rays=directions[:,:2]/directions[:,2,None]
            rays=rays[np.all(abs(rays)<=4,axis=1)]
            if len(rays):views.append(dict(camera=view['camera'],rays=rays.tolist()))
        self.roi_requested.add(t);self.roi_requested={v for v in self.roi_requested if v>=t-1_000_000_000}
        self.roi_pub.publish(String(data=json.dumps(dict(image_stamp_ns=t,session_id=self.origin.session_id,views=views))))

    def discard(self,status,stamp):
        self.pending.popleft();self.last_image_ns=stamp;self.health.dropped+=1
        self.health.details=dict(task='detect',status=status)

    def process(self):
        if not self.pending or self.origin is None or not self.clouds:return
        for index in range(len(self.pending)-1,-1,-1):
            _,frame=self.pending[index];stamp=ns(frame.header.stamp);roi=self.roi.get(stamp)
            if self.s.get('roi_lidar',False) and roi is None:continue
            tl=max((v['stamp_ns'] for v in roi),default=stamp) if roi is not None else stamp
            if self.poses.at(stamp) is None or self.poses.at(tl) is None:continue
            for _ in range(index):self.pending.popleft();self.health.dropped+=1
            break
        begun,frame=self.pending[0];tc=ns(frame.header.stamp);now=self.get_clock().now().nanoseconds
        if not 0<=now-tc<=self.s['max_age_s']*1e9:
            self.discard('STALE_DETECTIONS',tc);return
        cloud=min(self.clouds,key=lambda m:abs(ns(m.header.stamp)-tc))
        if abs(ns(cloud.header.stamp)-tc)>self.c['fusion']['max_sync_delta_ms']*1e6:
            if time.perf_counter()-begun<.12:return
            self.discard('NO_SYNCHRONIZED_LIDAR',tc);return
        roi=self.roi.get(tc)
        if self.s.get('roi_lidar',False) and roi is None and time.perf_counter()-begun<.08:return
        tl=max((int(v['stamp_ns']) for v in roi),default=ns(cloud.header.stamp)) if roi is not None else ns(cloud.header.stamp)
        if tl<=self.last_lidar_ns:self.discard('OUT_OF_ORDER_DEPTH',tc);return
        lp,cp=self.poses.at(tl),self.poses.at(tc)
        if lp is None or cp is None:
            if time.perf_counter()-begun<.12:return
            self.discard('NO_BRACKETED_POSE',tc);return
        if not 0<=now-tl<=self.s['max_age_s']*1e9:self.discard('STALE_DEPTH',tc);return
        self.pending.popleft();self.last_image_ns=tc
        try:
            processing=time.perf_counter();gather=[];oldest=tl
            for item in self.clouds:
                ti=ns(item.header.stamp)
                if not tl-int(self.s['scan_window_s']*1e9)<=ti<=tl:continue
                pose=self.poses.at(ti)
                if pose is None:continue
                pi,ri=pose;gather.append((cloud_xyz(item)@ri.T+pi-lp[0])@lp[1]);oldest=min(oldest,ti)
            roi_points=0
            for view in roi or []:
                pose=self.poses.at(int(view['stamp_ns']))
                if pose is None:continue
                i=next(i for i,v in enumerate(self.s['cameras']) if v['name']==view['camera'])
                t,q=self.view_poses[i];p,r=pose;points=np.asarray(view['points'],float).reshape(-1,3)
                points=points[np.isfinite(points).all(axis=1)]
                world=(points@quaternion_matrix(q).T+t)@r.T+p
                gather.append((world-lp[0])@lp[1]);roi_points+=len(points)
            points=np.concatenate(gather) if gather else cloud_xyz(cloud)
            options=dict(self.c['detection'],min_lidar_points=self.c['detection']['box_min_lidar_points'])
            floor=self.c['mission'].get('landing_ground_z_m',0.)+self.c['mission'].get('landing_surface_tolerance_m',.15)
            targets=[];per_view=[];stats={}
            for i,bopt in enumerate(self.view_poses):
                boxes=[b for b in frame.boxes if b['view_index']==i]
                if not boxes:per_view.append(0);continue
                world,projection=compensated_projection(points,lp,cp,self.body_lidar,bopt,self.k,self.s['width'],self.s['height'])
                motion=np.minimum(self.s['max_motion_pixels'],
                    self.k[0,0]*self.s['max_target_speed_mps']*abs(tc-tl)*1e-9/np.maximum(projection.depth_m,.2)+1)
                found,details=localize_boxes(world,projection,boxes,options,motion,ground_z=floor)
                targets.extend(found);per_view.append(len(found))
                for k,v in details.items():stats[k]=stats.get(k,0)+v
            count=len(targets);targets=merge_views(targets,self.s['merge_distance_m'])
            result=self.result(tl,frame,'OK' if targets else 'NO_BOX_DEPTH' if frame.boxes else 'NO_DETECTIONS')
            result.sync_delta_ms=abs(tc-tl)*1e-6
            observations,_=self.tracker.update([dict(class_id=t.class_id,position_enu_m=list(t.position_body_m)) for t in targets],tl)
            for detection,target in zip(observations,targets):
                tid=detection['track_id'];self.meta[tid]=(target.lidar_point_count,np.asarray(target.point_max_body_m)-target.point_min_body_m)
                out=self.target(self.tracker.tracks[tid],tl,True);put(out.position,target.position_body_m);result.targets.append(out)
            self.meta={k:v for k,v in self.meta.items() if k in self.tracker.tracks}
            self.last_lidar_ns=tl;self.measurements.publish(result);self.template=result;self.health.ok()
            self.health.details=dict(task='detect',status=result.status,targets=len(targets),per_view=per_view,
                roi_lidar_points=roi_points,duplicate_views_removed=count-len(targets),lidar_points=len(points),
                scan_window_ms=(tl-oldest)*1e-6,image_to_output_ms=round((self.get_clock().now().nanoseconds-tc)*1e-6,2),
                output_age_ms=round((self.get_clock().now().nanoseconds-tl)*1e-6,2),compute_ms=round((time.perf_counter()-processing)*1000,2),**stats)
        except Exception as exc:self.health.error(exc)


def main(args=None):
    c,ros=args_config(args);rclpy.init(args=ros);node=DetectionFusion(c)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
