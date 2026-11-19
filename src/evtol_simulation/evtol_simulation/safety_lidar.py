"""Dedicated omnidirectional UE LiDAR acquisition, independent of perception."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2,PointField
from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue
from .rpc import AirSimRPC
from .lidar_ring import LidarSharedMemory
from .runtime import SensorLease,TimerResolution
from .coordinates import FRD_FLU
from .sensor_node import header


def inspect_sensor(settings,vehicle,c):
    try:
        sensor=settings['Vehicles'][vehicle]['Sensors'][c['name']]
        expected={'Enabled':True,'SensorType':6,'DataFrame':'SensorLocalFrame',
            'HorizontalFOVStart':-180,'HorizontalFOVEnd':179.999,'VerticalFOVUpper':90,'VerticalFOVLower':-90,
            'Range':c['range_m'],'UpdateFrequency':c['rate_hz'],'RotationsPerSecond':c['rate_hz'],
            'NumberOfChannels':128,'PointsPerSecond':c['points_per_second'],'UseWallClockFrequency':True,
            'CompleteScanPerUpdate':True,
            'X':0,'Y':0,'Z':0,'Roll':0,'Pitch':0,'Yaw':0}
        problems=[f'{key}={sensor.get(key)} (expected {value})' for key,value in expected.items() if sensor.get(key,0 if key in ('X','Y','Z','Roll','Pitch','Yaw') else None)!=value]
        if problems:raise ValueError('; '.join(problems))
    except KeyError as exc:raise ValueError('Missing full-sphere safety sensor. Restart UE with airsim.navigation_2k.json') from exc


class SafetyLidar(Node):
    def __init__(self,c):
        super().__init__('safety_lidar_sensor');self.c=c;self.s=c['navigation_lidar'];self.count=0;self.last=0;self.wall=0.;self.error='waiting';self.skipped=0
        self.point_count=0;self.complete=False;self.capture_interval_s=0.
        self.lease=SensorLease(dict(backend='airsim',airsim=c['airsim']),'safety_lidar')
        connection=c['airsim'];rpc=AirSimRPC(connection['host'],connection['port'],3.)
        try:inspect_sensor(json.loads(rpc.call('getSettingsString')),connection['vehicle'],self.s)
        finally:rpc.close()
        self.reader=LidarSharedMemory('AirSimLidar_'+self.s['name'],65536,8)
        root='/'+c['namespace'];self.pub=self.create_publisher(PointCloud2,root+'/navigation/lidar/points',qos_profile_sensor_data)
        self.diag=self.create_publisher(DiagnosticArray,'/diagnostics',10)
        self.clock=Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1./(self.s['rate_hz']*4),self.tick,clock=self.clock)
        self.create_timer(1.,self.report,clock=self.clock)
        self.get_logger().info('Verified full-sphere safety LiDAR; shared-memory acquisition enabled')

    def tick(self):
        try:
            # Control needs the latest acquisition after a scheduler stall.
            # Archival sensor streams retain their independent FIFO readers.
            frame=self.reader.read_latest(self.last)
            if frame is None:return
            self.complete=frame.scan_complete;self.point_count=frame.point_count
            if not frame.scan_complete:
                raise ValueError('Incomplete navigation LiDAR frame: apply ue_support, rebuild UE and enable CompleteScanPerUpdate')
            if self.wall and frame.timestamp_ns<=self.stamp:raise ValueError('Safety LiDAR clock reset; restart experiment')
            if self.last:self.skipped+=max(0,frame.frame_id-self.last-1)
            points=np.frombuffer(frame.xyz_bytes,dtype='<f4').reshape(-1,3)@FRD_FLU
            points=np.asarray(points,dtype='<f4')
            msg=PointCloud2(header=header(frame.timestamp_ns,self.c['namespace']+'/base_link'),height=1,width=len(points),
                fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z'))],
                point_step=12,row_step=len(points)*12,data=points.tobytes(),is_dense=False)
            self.capture_interval_s=(frame.timestamp_ns-self.stamp)*1e-9 if self.wall else 0.
            self.pub.publish(msg);self.last=frame.frame_id;self.stamp=frame.timestamp_ns;self.wall=time.perf_counter();self.count+=1;self.error=''
        except Exception as exc:
            if self.error!=str(exc):self.get_logger().error(str(exc))
            self.error=str(exc)

    def report(self):
        age=time.perf_counter()-self.wall if self.wall else 1e9
        status=DiagnosticStatus(name='/'+self.c['namespace']+'/navigation_lidar',hardware_id=self.s['name'],
            level=DiagnosticStatus.OK if not self.error and age<self.s['timeout_s'] else DiagnosticStatus.ERROR,
            message=self.error or ('streaming' if age<self.s['timeout_s'] else 'stale'),
            values=[KeyValue(key='backend',value='airsim'),KeyValue(key='sensor',value='navigation_lidar'),
                    KeyValue(key='errors',value='1' if self.error else '0'),
                    KeyValue(key='samples',value=str(self.count)),KeyValue(key='age_s',value=str(age)),
                    KeyValue(key='scan_complete',value=str(self.complete)),KeyValue(key='point_count',value=str(self.point_count)),
                    KeyValue(key='capture_interval_s',value=str(self.capture_interval_s)),
                    KeyValue(key='skipped_frames',value=str(self.skipped))])
        self.diag.publish(DiagnosticArray(status=[status]))

    def close(self):self.reader.close();self.lease.close()


def main(args=None):
    p=argparse.ArgumentParser();p.add_argument('--config',required=True);opts,ros=p.parse_known_args(args)
    c=yaml.safe_load(Path(opts.config).read_text(encoding='utf-8-sig'));resolution=TimerResolution()
    rclpy.init(args=ros);node=SafetyLidar(c)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.close();node.destroy_node();rclpy.shutdown();resolution.close()
