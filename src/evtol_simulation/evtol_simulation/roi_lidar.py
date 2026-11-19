"""Acquire bounded, visually requested LiDAR rays; never reads semantic truth."""
import json
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2,PointField
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from .coordinates import extrinsic,quaternion_matrix,matrix_quaternion
from .acquisition_support import args_config, Health, LATEST, header
from .rpc import AirSimRPC,image_request
from .runtime import TimerResolution,SensorLease


class RoiLidar(Node):
    def __init__(self,c):
        super().__init__('roi_lidar_sensor');self.c=c;self.pending=None;self.lock=threading.Lock();self.stop=threading.Event()
        self.lease=SensorLease(dict(backend='airsim',airsim=c['airsim']),'roi_lidar')
        self.health=Health(self,'/'+c['namespace']+'/roi_lidar')
        root='/'+c['namespace']+'/perception/roi_lidar'
        self.pub=self.create_publisher(String,root+'/returns',LATEST)
        self.point_pubs={v['name']:self.create_publisher(PointCloud2,root+'/'+v['name']+'/points',LATEST) for v in c['surround']['cameras']}
        self.static=StaticTransformBroadcaster(self);transforms=[]
        for v in c['surround']['cameras']:
            t,q=extrinsic([0,0,0],v['rpy_deg']);q=matrix_quaternion(quaternion_matrix(q)@quaternion_matrix([-.5,.5,-.5,.5]))
            tf=TransformStamped(header=header(0,c['namespace']+'/base_link'),child_frame_id=c['namespace']+'/'+v['name']+'_optical')
            tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=map(float,q)
            transforms.append(tf)
        self.static.sendTransform(transforms)
        self.create_subscription(String,root+'/requests',self.request,LATEST)
        self.worker=threading.Thread(target=self.run,daemon=True);self.worker.start()

    def request(self,msg):
        try:
            data=json.loads(msg.data);views=data['views'];total=0
            allowed={v['name'] for v in self.c['surround']['cameras']}
            if len(views)>6:raise ValueError('Too many requested LiDAR views')
            for v in views:
                a=np.asarray(v['rays'],float);total+=len(a)
                if v['camera'] not in allowed or a.ndim!=2 or a.shape[1]!=2 or not np.isfinite(a).all() or np.any(abs(a)>4):
                    raise ValueError('Invalid optical ray request')
            if total>4096:raise ValueError('ROI LiDAR cycle exceeds 4096 rays')
            with self.lock:
                if self.pending is not None:self.health.dropped+=1
                self.pending=data
        except Exception as exc:self.health.error(exc)

    def run(self):
        c=self.c['airsim'];rpc=AirSimRPC(c['host'],c['port'],1.)
        try:
            while not self.stop.wait(.002):
                with self.lock:data=self.pending;self.pending=None
                if data is None:continue
                try:
                    requests=[]
                    for v in data['views']:
                        payload=' '.join('%.7g %.7g'%(a,b) for a,b in v['rays'])
                        key='roi_lidar:'+v['camera']+':'+str(self.c['surround']['lidar']['range_m'])+' '+payload
                        requests.append(image_request(key,0))
                    responses=rpc.call('simGetImages',requests,c['vehicle'],False) if requests else []
                    if len(responses)!=len(requests):raise ValueError('Missing adaptive LiDAR response')
                    views=[];hits=0
                    for v,response in zip(data['views'],responses):
                        if response.get('message')!='roi_lidar_v1|'+v['camera']:raise ValueError('Adaptive LiDAR plugin mismatch: '+str(response.get('message')))
                        pts=np.asarray(response['image_data_float'],np.float32).reshape(-1,3)
                        if len(pts)!=len(v['rays']):raise ValueError('Adaptive ray count mismatch')
                        pts=np.asarray(pts[np.isfinite(pts).all(axis=1)],dtype='<f4');hits+=len(pts)
                        self.point_pubs[v['camera']].publish(PointCloud2(
                            header=header(response['time_stamp'],self.c['namespace']+'/'+v['camera']+'_optical'),height=1,width=len(pts),
                            fields=[PointField(name=n,offset=4*i,datatype=7,count=1) for i,n in enumerate(('x','y','z'))],
                            point_step=12,row_step=len(pts)*12,data=pts.tobytes(),is_dense=True))
                        views.append(dict(camera=v['camera'],stamp_ns=int(response['time_stamp']),points=pts.tolist()))
                    self.pub.publish(String(data=json.dumps(dict(image_stamp_ns=data['image_stamp_ns'],session_id=data['session_id'],views=views),allow_nan=False)))
                    self.health.ok();self.health.details=dict(requested_rays=sum(len(v['rays']) for v in data['views']),hits=hits,views=len(views))
                except Exception as exc:self.health.error(exc)
        finally:rpc.close()

    def close(self):self.stop.set();self.worker.join(timeout=3.);self.lease.close()


def main(args=None):
    c,ros=args_config(args);res=TimerResolution();rclpy.init(args=ros);node=RoiLidar(c)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.close();node.destroy_node();rclpy.shutdown();res.close()
