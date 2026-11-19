"""One multi-view semantic measurement per scan, with acquisition-time poses."""
from collections import deque
import json
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from evtol_perception_interfaces.msg import SemanticFrame
import time
import numpy as np
import rclpy
from .localizer_node import Localizer
from .common import args_config, ns, put, LATEST
from .geometry import cloud_xyz, compensated_projection
from .clustering import extract_visible_targets
from .surround_geometry import optical_pose, intrinsics, merge_views


class SurroundFusion(Localizer):
    def __init__(self,c):
        cv2.setNumThreads(1)
        super().__init__(c,surround=True);self.s=dict(c['surround']);self.s['width']//=2;self.s['height']//=2
        self.s['roi_search_pixels']=max(1,self.s.get('roi_search_pixels',8)//2)
        self.s['max_motion_pixels']=self.s.get('max_motion_pixels',12.)/2
        self.s['min_mask_pixels']=max(2,int(np.ceil(self.s['min_mask_pixels']/4)))
        self.k=intrinsics(self.s)
        self.view_poses=[optical_pose(v) for v in self.s['cameras']]
        self.create_subscription(CompressedImage,'/'+c['namespace']+'/perception/surround/semantic_ids/compressed',self.receive_ids,LATEST)
        self.body_lidar=(np.zeros(3),[0,0,0,1]);self.roi={};self.pending=deque(maxlen=3)
        self.last_roi_views=None;self.last_roi_stamp=0;self.roi_requested=set()
        self.create_subscription(String,'/'+c['namespace']+'/perception/surround/exposure',self.prefetch,LATEST)
        root='/'+c['namespace']+'/perception/roi_lidar'
        self.roi_pub=self.create_publisher(String,root+'/requests',LATEST)
        self.create_subscription(String,root+'/returns',self.receive_roi,LATEST)

    def receive_ids(self,msg):
        labels=cv2.imdecode(np.frombuffer(msg.data,np.uint8),cv2.IMREAD_UNCHANGED)
        if labels is None or labels.dtype!=np.uint8 or labels.ndim!=2 or np.any(labels>6):
            self.health.error('Invalid lossless semantic ID image');return
        if labels.shape!=(4*self.s['height'],6*self.s['width']):
            self.health.error('Surround image geometry mismatch');return
        labels=reduce_semantics(labels,self.c['detection']['target_class_ids'])
        mask=SemanticFrame(header=msg.header,width=labels.shape[1],height=labels.shape[0])
        mask.class_ids.frombytes(labels.astype('<i4').tobytes());self.receive_mask(mask)

    def receive_roi(self,msg):
        data=json.loads(msg.data)
        if self.origin is None or data['session_id']!=self.origin.session_id:return
        self.roi[data['image_stamp_ns']]=data['views']
        for key in sorted(self.roi)[:-4]:del self.roi[key]

    def prefetch(self,msg):
        if not self.s.get('roi_prefetch',False):return
        if self.origin is None or not self.last_roi_views:return
        t=int(json.loads(msg.data)['stamp_ns'])
        if not 0<t-self.last_roi_stamp<=350_000_000:return
        old=self.poses.at(self.last_roi_stamp);current=self.poses.at(t)
        if current is None and self.poses.samples and 0<=t-self.poses.samples[-1][0]<=100_000_000:
            current=self.poses.at(self.poses.samples[-1][0])
        if old is None or current is None:return
        views=[]
        for view in self.last_roi_views:
            index=next(i for i,v in enumerate(self.s['cameras']) if v['name']==view['camera'])
            rays=rotate_optical_rays(view['rays'],self.view_poses[index][1],old[1],current[1])
            if len(rays):views.append(dict(camera=view['camera'],rays=rays.tolist()))
        self.roi_requested.add(t)
        self.roi_pub.publish(String(data=json.dumps(dict(image_stamp_ns=t,session_id=self.origin.session_id,views=views))))
        self.roi_requested={key for key in self.roi_requested if key>=t-1_000_000_000}

    def receive_mask(self,msg):
        super().receive_mask(msg)
        if not self.s.get('roi_lidar',False) or self.origin is None:return
        h,w=self.s['height'],self.s['width']
        if (msg.width,msg.height)!=(3*w,2*h):return
        request_begun=time.perf_counter()
        labels=np.asarray(msg.class_ids,np.int32).reshape(2*h,3*w);views=[]
        for i,view in enumerate(self.s['cameras']):
            face=labels[(i//3)*h:(i//3+1)*h,(i%3)*w:(i%3+1)*w]
            selected=np.isin(face,self.c['detection']['target_class_ids']).astype(np.uint8)
            from .clustering import _connected_components
            count,components,stats=_connected_components(selected)
            ids=[k for k in range(1,count) if stats[k,4]>=2]
            if not ids:continue
            budget=max(4,640//len(ids));samples=[]
            for component in ids:
                left,top,width,height,_=stats[component];radius=self.s.get('roi_search_pixels',8)
                x0,y0=max(0,left-radius),max(0,top-radius)
                x1,y1=min(w,left+width+radius),min(h,top+height+radius)
                region=(components[y0:y1,x0:x1]==component).astype(np.uint8)
                expanded=cv2.dilate(region,np.ones((2*radius+1,2*radius+1),np.uint8))
                xy=component_rays(region,expanded,budget)
                samples.append(xy+np.array([x0,y0]))
            xy=np.concatenate(samples)[:640];x,y=xy.T
            rays=np.column_stack(((x-self.k[0,2])/self.k[0,0],(y-self.k[1,2])/self.k[1,1]))
            views.append(dict(camera=view['name'],rays=rays.tolist()))
        stamp=ns(msg.header.stamp);self.last_roi_views=views;self.last_roi_stamp=stamp
        if stamp not in self.roi_requested:
            self.roi_pub.publish(String(data=json.dumps(dict(image_stamp_ns=stamp,session_id=self.origin.session_id,views=views))))
        self.request_compute_ms=round((time.perf_counter()-request_begun)*1000,2)

    def process(self):
        if not self.pending or self.origin is None or not self.clouds:return
        # Keep waiting transactions, but execute the newest one whose real ROI
        # and exposure poses have arrived. FIFO creates avoidable stale work.
        for index in range(len(self.pending)-1,-1,-1):
            _,candidate=self.pending[index];stamp=ns(candidate.header.stamp)
            roi=self.roi.get(stamp)
            if self.s.get('roi_lidar',False) and roi is None:continue
            depth_stamp=max((v['stamp_ns'] for v in roi),default=stamp) if roi is not None else stamp
            if self.poses.at(stamp) is None or self.poses.at(depth_stamp) is None:continue
            for _ in range(index):self.pending.popleft();self.health.dropped+=1
            break
        begun,mask=self.pending[0];tc=ns(mask.header.stamp)
        if (self.get_clock().now().nanoseconds-tc)*1e-9>self.s['max_age_s']:
            self.pending.popleft();self.last_image_ns=tc;self.health.dropped+=1
            self.health.details=dict(status='STALE_SEMANTICS');return
        # A complete scan can project into all six views in the same cycle.
        # Each image is consumed once. ROI acquisition is newer than that image;
        # using its timestamp as a lower bound discards the next image's scan.
        cloud=min(self.clouds,key=lambda m:abs(ns(m.header.stamp)-tc))
        if cloud is None or abs(ns(cloud.header.stamp)-tc)>self.c['fusion']['max_sync_delta_ms']*1e6:
            if time.perf_counter()-begun<.15:return
            self.pending.popleft();self.last_image_ns=tc;self.health.dropped+=1
            self.health.details=dict(status='NO_SYNCHRONIZED_LIDAR');return
        tl=ns(cloud.header.stamp);roi=self.roi.get(tc)
        if self.s.get('roi_lidar',False) and roi is None and time.perf_counter()-begun<.12:return
        if roi:tl=max(v['stamp_ns'] for v in roi)
        if tl<=self.last_lidar_ns:
            self.pending.popleft();self.last_image_ns=tc;self.health.dropped+=1
            self.health.details=dict(status='OUT_OF_ORDER_DEPTH');return
        lp=self.poses.at(tl);cp=self.poses.at(tc)
        if lp is None or cp is None:
            if time.perf_counter()-begun<.15:return
            self.pending.popleft();self.last_image_ns=tc;self.health.dropped+=1
            self.health.details=dict(status='NO_BRACKETED_POSE',image_pose=cp is not None,lidar_pose=lp is not None,
                pose_age_ms=(self.poses.samples[-1][0]-tl)*1e-6 if self.poses.samples else None);return
        self.pending.popleft();self.last_image_ns=tc
        if (self.get_clock().now().nanoseconds-tl)*1e-9>self.s['max_age_s']:
            self.health.dropped+=1;self.health.details=dict(status='STALE_INPUT');return
        try:
            processing_begun=time.perf_counter()
            h,w=self.s['height'],self.s['width']
            if (mask.height,mask.width)!=(2*h,3*w):raise ValueError('Surround semantic mosaic geometry mismatch')
            labels=np.asarray(mask.class_ids,np.int32).reshape(mask.height,mask.width)
            primary=cloud_xyz(cloud);gather=[];oldest=tl
            for item in self.clouds:
                ti=ns(item.header.stamp)
                if not tl-int(self.s.get('scan_window_s',.15)*1e9)<=ti<=tl:continue
                pose=self.poses.at(ti)
                if pose is None:continue
                pts=cloud_xyz(item);pi,ri=pose
                gather.append((pts@ri.T+pi-lp[0])@lp[1]);oldest=min(oldest,ti)
            roi_points=0
            for rv in roi or []:
                pose=self.poses.at(rv['stamp_ns'])
                if pose is None:continue
                idx=next(i for i,v in enumerate(self.s['cameras']) if v['name']==rv['camera'])
                from evtol_simulation.coordinates import quaternion_matrix
                tb,qb=self.view_poses[idx];rp,rr=pose
                pts=np.asarray(rv['points'],float).reshape(-1,3)
                world=(pts@quaternion_matrix(qb).T+tb)@rr.T+rp
                gather.append((world-lp[0])@lp[1]);roi_points+=len(pts)
            points=np.concatenate(gather) if gather else primary
            targets=[];diagnostics={};per_view=[];d=self.c['detection']
            for i,bopt in enumerate(self.view_poses):
                face=labels[(i//3)*h:(i//3+1)*h,(i%3)*w:(i%3+1)*w]
                if not any(np.any(face==cls) for cls in d['target_class_ids']):
                    per_view.append(0);continue
                world,projection=compensated_projection(points,lp,cp,self.body_lidar,bopt,self.k,w,h)
                stats={}
                motion_radius=np.minimum(self.s.get('max_motion_pixels',12.),
                    self.k[0,0]*self.s.get('max_target_speed_mps',4.)*abs(tc-tl)*1e-9/np.maximum(projection.depth_m,.2)+1.)
                found=extract_visible_targets(world,projection,face,np.full(face.shape,np.nan,np.float32),
                    d['target_class_ids'],min_lidar_points=self.s['min_lidar_points'],
                    min_mask_pixels=self.s['min_mask_pixels'],cluster_radius_m=d['cluster_radius_m'],
                    max_extent_by_class={6:d.get('max_drone_extent_m',4.),3:d.get('max_pedestrian_extent_m',3.)},diagnostics=stats,
                    pixel_motion_radius=motion_radius)
                targets.extend(found);per_view.append(len(found))
                for k,v in stats.items():diagnostics[k]=diagnostics.get(k,0)+int(v)
            before=len(targets);targets=merge_views(targets,self.s['merge_distance_m'])
            floor=self.c['mission'].get('landing_ground_z_m',0.)+self.c['mission'].get('landing_surface_tolerance_m',.15)
            targets=[t for t in targets if not (t.point_max_body_m[2]<=floor and t.point_max_body_m[2]-t.point_min_body_m[2]<=.05)]
            result=self.result(tl,mask,'OK' if targets else 'NO_TARGET_DEPTH');result.sync_delta_ms=abs(tc-tl)*1e-6
            detections,_=self.tracker.update([dict(class_id=t.class_id,position_enu_m=list(t.position_body_m)) for t in targets],tl)
            for detection,target in zip(detections,targets):
                tid=detection['track_id'];self.meta[tid]=(target.lidar_point_count,np.asarray(target.point_max_body_m)-target.point_min_body_m)
                out=self.target(self.tracker.tracks[tid],tl,True);put(out.position,target.position_body_m);result.targets.append(out)
            self.meta={k:v for k,v in self.meta.items() if k in self.tracker.tracks}
            self.last_lidar_ns=tl;self.measurements.publish(result);self.template=result;self.health.ok()
            self.health.details=dict(status=result.status,targets=len(targets),per_view=per_view,
                roi_lidar_points=roi_points,duplicate_views_removed=before-len(targets),lidar_points=len(points),scan_window_ms=(tl-oldest)*1e-6,
                sync_delta_ms=round(result.sync_delta_ms,2),drone_pixels=int(np.count_nonzero(labels==6)),
                output_age_ms=round((self.get_clock().now().nanoseconds-tl)*1e-6,2),
                image_to_output_ms=round((self.get_clock().now().nanoseconds-tc)*1e-6,2),**diagnostics)
            self.health.details['compute_ms']=round((time.perf_counter()-processing_begun)*1000,2)
            self.health.details['request_compute_ms']=getattr(self,'request_compute_ms',None)
        except Exception as exc:self.health.error(exc)


def main(args=None):
    c,ros=args_config(args);rclpy.init(args=ros);node=SurroundFusion(c)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()


def reduce_semantics(labels,target_classes):
    """Preserve tiny learned targets during 2x geometric raster reduction."""
    h,w=labels.shape;blocks=labels.reshape(h//2,2,w//2,2)
    small=labels[::2,::2].copy()
    for class_id in target_classes:small[np.any(blocks==class_id,axis=(1,3))]=class_id
    return small


def component_rays(region,expanded,budget):
    """Keep learned tiny-object pixels in the real ray budget, then the rim.

    Flattened uniform sampling of a dilated region can skip every actual target
    pixel. Subpixel optical rays resolve a narrow fuselage even when geometry
    uses the reduced raster. All returns still require real UE line traces.
    """
    yy,xx=np.nonzero(region)
    core=np.column_stack((xx,yy)).astype(float)
    if not len(core):return np.empty((0,2))
    offsets=np.array([[0,0],[.35,0],[-.35,0],[0,.35],[0,-.35]])
    core=(core[:,None,:]+offsets).reshape(-1,2)
    core[:,0]=np.clip(core[:,0],0,region.shape[1]-1)
    core[:,1]=np.clip(core[:,1],0,region.shape[0]-1)
    core=np.unique(core,axis=0)
    count=min(len(core),max(2,budget//2))
    chosen=core[np.linspace(0,len(core)-1,count,dtype=int)]
    seed=np.array([[xx[len(xx)//2],yy[len(yy)//2]]],float)
    chosen=np.unique(np.vstack((seed,chosen)),axis=0);count=len(chosen)
    yy,xx=np.nonzero((expanded>0)&(region==0));rim=np.column_stack((xx,yy)).astype(float)
    if len(rim):chosen=np.vstack((chosen,rim[np.linspace(0,len(rim)-1,min(len(rim),budget-count),dtype=int)]))
    return chosen


def rotate_optical_rays(rays,optical_quaternion,old_body_rotation,new_body_rotation):
    """Counter-rotate prior visual directions using measured ego attitude only."""
    from evtol_simulation.coordinates import quaternion_matrix
    rbc=quaternion_matrix(optical_quaternion)
    rays=np.asarray(rays,float).reshape(-1,2)
    directions=np.column_stack((rays,np.ones(len(rays))))
    current=directions@rbc.T@old_body_rotation.T@new_body_rotation@rbc
    current=current[current[:,2]>.1]
    projected=current[:,:2]/current[:,2,None]
    return projected[np.all(abs(projected)<=4,axis=1)]
