"""One multi-view semantic measurement per scan, with acquisition-time poses."""
import time
import numpy as np
import rclpy
from .localizer_node import Localizer
from .common import args_config, ns, put
from .geometry import cloud_xyz, compensated_projection
from .clustering import extract_visible_targets
from .surround_geometry import optical_pose, intrinsics, merge_views


class SurroundFusion(Localizer):
    def __init__(self,c):
        super().__init__(c,surround=True);self.s=c['surround'];self.k=intrinsics(self.s)
        self.view_poses=[optical_pose(v) for v in self.s['cameras']]

    def process(self):
        if not self.pending or self.origin is None or not self.clouds:return
        begun,mask=self.pending[0];tc=ns(mask.header.stamp)
        # A complete scan can project into all six views in the same cycle.
        fresh=[m for m in self.clouds if ns(m.header.stamp)>self.last_lidar_ns]
        cloud=min(fresh,key=lambda m:abs(ns(m.header.stamp)-tc)) if fresh else None
        if cloud is None or abs(ns(cloud.header.stamp)-tc)>self.c['fusion']['max_sync_delta_ms']*1e6:
            if time.perf_counter()-begun<.08:return
            self.pending.popleft();self.last_image_ns=tc;self.health.dropped+=1
            self.health.details=dict(status='NO_SYNCHRONIZED_LIDAR');return
        tl=ns(cloud.header.stamp);lp=self.poses.at(tl);cp=self.poses.at(tc)
        if lp is None or cp is None:
            if time.perf_counter()-begun<.08:return
            self.pending.popleft();self.last_image_ns=tc;self.health.dropped+=1
            self.health.details=dict(status='NO_BRACKETED_POSE');return
        self.pending.popleft();self.last_image_ns=tc
        if (self.get_clock().now().nanoseconds-tl)*1e-9>self.s['max_age_s']:
            self.health.dropped+=1;self.health.details=dict(status='STALE_INPUT');return
        try:
            h,w=self.s['height'],self.s['width']
            if (mask.height,mask.width)!=(2*h,3*w):raise ValueError('Surround semantic mosaic geometry mismatch')
            labels=np.asarray(mask.class_ids,np.int32).reshape(mask.height,mask.width)
            points=cloud_xyz(cloud);targets=[];diagnostics={};per_view=[];d=self.c['detection']
            for i,bopt in enumerate(self.view_poses):
                face=labels[(i//3)*h:(i//3+1)*h,(i%3)*w:(i%3+1)*w]
                world,projection=compensated_projection(points,lp,cp,self.body_lidar,bopt,self.k,w,h)
                stats={}
                found=extract_visible_targets(world,projection,face,np.full(face.shape,np.nan,np.float32),
                    d['target_class_ids'],min_lidar_points=self.s['min_lidar_points'],
                    min_mask_pixels=self.s['min_mask_pixels'],cluster_radius_m=d['cluster_radius_m'],
                    max_extent_by_class={6:d.get('max_drone_extent_m',4.),3:d.get('max_pedestrian_extent_m',3.)},diagnostics=stats)
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
                duplicate_views_removed=before-len(targets),lidar_points=len(points),
                sync_delta_ms=round(result.sync_delta_ms,2),drone_pixels=int(np.count_nonzero(labels==6)),
                output_age_ms=round((self.get_clock().now().nanoseconds-tl)*1e-6,2),**diagnostics)
        except Exception as exc:self.health.error(exc)


def main(args=None):
    c,ros=args_config(args);rclpy.init(args=ros);node=SurroundFusion(c)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
