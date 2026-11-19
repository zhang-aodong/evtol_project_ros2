"""Dense surround LiDAR acquisition only; no semantic IDs or object truth."""
import json
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from sensor_msgs.msg import PointCloud2,PointField
from evtol_targeting.common import args_config, header, Health, LATEST
from evtol_targeting.surround_geometry import lidar_settings
from .rpc import AirSimRPC
from .lidar_ring import LidarSharedMemory
from .coordinates import FRD_FLU
from .runtime import SensorLease,TimerResolution


class PerceptionLidar(Node):
    def __init__(self,c):
        super().__init__('perception_lidar_sensor');self.c=c;self.lc=c['surround']['lidar'];self.last=0;self.skipped=0
        self.lease=SensorLease(dict(backend='airsim',airsim=c['airsim']),'perception_lidar')
        rpc=AirSimRPC(c['airsim']['host'],c['airsim']['port'],3.)
        try:
            actual=json.loads(rpc.call('getSettingsString'))['Vehicles'][c['airsim']['vehicle']]['Sensors'][self.lc['name']]
            for k,v in lidar_settings(self.lc).items():
                if actual.get(k)!=v:raise ValueError('Perception LiDAR settings mismatch: '+k+'; restart UE')
        finally:rpc.close()
        self.reader=LidarSharedMemory('AirSimLidar_'+self.lc['name'],65536,8)
        self.pub=self.create_publisher(PointCloud2,'/'+c['namespace']+'/perception/lidar/points',LATEST)
        self.health=Health(self,'/'+c['namespace']+'/perception_lidar')
        self.steady=Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1/(self.lc['rate_hz']*4),self.tick,clock=self.steady)

    def tick(self):
        try:
            f=self.reader.read_latest(self.last)
            if f is None:return
            if not f.scan_complete:raise ValueError('Incomplete perception scan')
            if self.last:self.skipped+=max(0,f.frame_id-self.last-1)
            self.last=f.frame_id
            p=np.asarray(np.frombuffer(f.xyz_bytes,'<f4').reshape(-1,3)@FRD_FLU,dtype='<f4')
            self.pub.publish(PointCloud2(header=header(f.timestamp_ns,self.c['namespace']+'/base_link'),height=1,width=len(p),
                fields=[PointField(name=n,offset=4*i,datatype=7,count=1) for i,n in enumerate(('x','y','z'))],
                point_step=12,row_step=12*len(p),data=p.tobytes(),is_dense=False))
            self.health.ok();self.health.details=dict(points=len(p),scan_complete=True,skipped_frames=self.skipped,
                output_age_ms=round((self.get_clock().now().nanoseconds-f.timestamp_ns)*1e-6,2))
        except Exception as exc:self.health.error(exc)

    def close(self):self.reader.close();self.lease.close()


def main(args=None):
    c,ros=args_config(args);res=TimerResolution();rclpy.init(args=ros);node=PerceptionLidar(c)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.close();node.destroy_node();rclpy.shutdown();res.close()
