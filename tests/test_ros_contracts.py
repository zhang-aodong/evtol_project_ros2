import sys
import unittest
from pathlib import Path
import numpy as np
from sensor_msgs.msg import Image, Imu, PointCloud2, PointField
from evtol_simulation.validation import StreamStats, check_payload
from evtol_simulation.configuration import load

CONFIG=Path(__file__).resolve().parents[1]/'src'/'evtol_simulation'/'config'/'simulation.yaml'


class Contracts(unittest.TestCase):
    def test_navigation_cloud_uses_xyz_and_its_own_range(self):
        c=load(CONFIG);c['navigation_lidar']={'range_m':30.}
        points=np.asarray([[3.,4.,0.]],dtype='<f4')
        msg=PointCloud2(height=1,width=1,point_step=12,row_step=12,data=points.tobytes(),
            fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z'))])
        check_payload('navigation/lidar/points',msg,c)
        msg.data=np.asarray([[31.,0.,0.]],dtype='<f4').tobytes()
        with self.assertRaises(ValueError):check_payload('navigation/lidar/points',msg,c)

    def test_windows_ros_diagnostic_level_is_a_byte(self):
        from diagnostic_msgs.msg import DiagnosticStatus
        msg=DiagnosticStatus(level=DiagnosticStatus.OK)
        self.assertEqual(msg.level,DiagnosticStatus.OK)

    def test_subscriber_rate_and_loss(self):
        c=load(CONFIG)
        stats=StreamStats(20.)
        for i in range(200): stats.add(1_000_000_000+i*50_000_000,i*.05)
        self.assertTrue(stats.report(10.,c['acceptance'])['passed'])
        stats.add(stats.stamps[-1],11.)
        self.assertFalse(stats.report(10.,c['acceptance'])['passed'])

    def test_poll_rate_is_not_sensor_rate(self):
        c=load(CONFIG)
        stats=StreamStats(20.)
        for i in range(100): stats.add(1_000_000_000+i*100_000_000,i*.1)
        self.assertFalse(stats.report(10.,c['acceptance'])['passed'])

    def test_image_length(self):
        c=load(CONFIG)
        msg=Image(width=640,height=480,step=1920,encoding='bgr8',data=b'abc')
        with self.assertRaises(ValueError): check_payload('camera/rgb/image_raw',msg,c)

    def test_imu_nan_and_orientation(self):
        c=load(CONFIG)
        msg=Imu()
        msg.orientation.w=0.
        with self.assertRaises(ValueError): check_payload('imu/data',msg,c)
        msg.orientation.w=1.
        msg.linear_acceleration.x=float('nan')
        with self.assertRaises(ValueError): check_payload('imu/data',msg,c)

    def test_empty_no_data_fails(self):
        self.assertFalse(StreamStats(100.).report(60.,load(CONFIG)['acceptance'])['passed'])
