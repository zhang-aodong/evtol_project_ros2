"""ROS transport/geometry contract, independent of UE identities and inference."""
import copy
from pathlib import Path
import unittest
import numpy as np
import cv2
import yaml
import rclpy
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage,PointCloud2,PointField
from evtol_perception_interfaces.msg import ReferenceOrigin
from evtol_targeting.common import header
from std_msgs.msg import String
import json
from evtol_targeting.surround_fusion import SurroundFusion,reduce_semantics,component_rays,rotate_optical_rays

ROOT=Path(__file__).resolve().parents[1]


class SurroundNodeContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):rclpy.init(domain_id=229)
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()

    def test_small_component_gets_subpixel_rays_without_exceeding_budget(self):
        region=np.zeros((11,11),np.uint8);region[5,5]=1
        samples=component_rays(region,np.ones_like(region),12)
        self.assertLessEqual(len(samples),12)
        core=samples[np.linalg.norm(samples-[5,5],axis=1)<.5]
        self.assertGreaterEqual(len(core),3)
        self.assertTrue(np.any(np.all(np.isclose(core,[5,5]),axis=1)))
        self.assertEqual(len(np.unique(samples,axis=0)),len(samples))

    def test_prefetch_counter_rotates_measured_ego_attitude(self):
        from scipy.spatial.transform import Rotation
        body=Rotation.from_euler('z',12,degrees=True).as_matrix()
        rays=rotate_optical_rays([[0,0]],[0,0,0,1],np.eye(3),body)
        np.testing.assert_allclose(rays,[[0,0]],atol=1e-9)
        body=Rotation.from_euler('y',12,degrees=True).as_matrix()
        rays=rotate_optical_rays([[0,0]],[0,0,0,1],np.eye(3),body)
        np.testing.assert_allclose(rays,[[-np.tan(np.radians(12)),0]],atol=1e-9)

    def test_reduced_geometry_preserves_real_small_target_and_not_empty_background(self):
        labels=np.zeros((8,12),np.uint8);labels[3,7]=6;labels[6,10]=3
        small=reduce_semantics(labels,[3,6])
        self.assertEqual(small.shape,(4,6));self.assertEqual(small[1,3],6);self.assertEqual(small[3,5],3)
        self.assertEqual(np.count_nonzero(small),2)

    def test_prefetch_only_requests_real_rays_and_cannot_publish_a_detection(self):
        c=yaml.safe_load((ROOT/'tests/fixtures/legacy_targeting.yaml').read_text(encoding='utf8'));c['_root']=ROOT
        c['surround']['roi_prefetch']=True
        node=SurroundFusion(c)
        try:
            node.receive_origin(ReferenceOrigin(session_id='prefetch-test'))
            for t in (1_000_000_000,1_100_000_000,1_200_000_000):node.poses.add(t,[0,0,0],[0,0,0,1])
            requests=[]
            class Capture:
                def publish(self,msg):requests.append(json.loads(msg.data))
            node.roi_pub=Capture();node.last_roi_views=[dict(camera='surround_front',rays=[[0,0]])];node.last_roi_stamp=1_000_000_000
            node.prefetch(String(data=json.dumps(dict(stamp_ns=1_200_000_000))))
            self.assertEqual(len(requests),1);self.assertEqual(set(requests[0]),{'image_stamp_ns','session_id','views'})
            self.assertEqual(len(node.tracker.tracks),0)
            node.prefetch(String(data=json.dumps(dict(stamp_ns=2_000_000_000))))
            self.assertEqual(len(requests),1)
            node.s['roi_prefetch']=False
            node.prefetch(String(data=json.dumps(dict(stamp_ns=1_250_000_000))))
            self.assertEqual(len(requests),1)
        finally:node.destroy_node()

    def test_lossless_labels_plus_real_depth_geometry_create_a_measurement(self):
        c=yaml.safe_load((ROOT/'tests/fixtures/legacy_targeting.yaml').read_text(encoding='utf8'));c['_root']=ROOT;c['surround']['roi_lidar']=True
        node=SurroundFusion(c)
        try:
            node.receive_origin(ReferenceOrigin(session_id='surround-contract'))
            now=node.get_clock().now().nanoseconds
            node.set_parameters([Parameter('use_sim_time',value=True)])
            node.get_clock().set_ros_time_override(Time(nanoseconds=now))
            t=now-50_000_000
            for dt in (-30_000_000,30_000_000):node.poses.add(t+dt,[0,0,10],[0,0,0,1])
            p=np.asarray([[5,0,0],[5,.03,.03],[5,-.03,-.03]],dtype='<f4')
            cloud=PointCloud2(header=header(t,'drone1/base_link'),height=1,width=len(p),
                fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z'))],
                point_step=12,row_step=12*len(p),data=p.tobytes())
            node.receive_cloud(cloud)
            # The previous image's ROI scan may be newer than this exposure.
            # It must not prevent reuse of a genuine synchronized raw scan.
            node.last_lidar_ns=t+20_000_000
            node.roi[t]=[dict(camera='surround_front',stamp_ns=t+25_000_000,points=[[0,0,5],[.03,.03,5],[-.03,-.03,5]])]
            w,h=c['surround']['width'],c['surround']['height']
            mask=np.zeros((2*h,3*w),np.uint8);mask[h//2-12:h//2+12,w//2-12:w//2+12]=6
            ok,png=cv2.imencode('.png',mask);self.assertTrue(ok)
            results=[]
            class Capture:
                def publish(self,m):results.append(copy.deepcopy(m))
            node.measurements=Capture()
            node.receive_ids(CompressedImage(header=header(t,'drone1/surround_mosaic'),format='png',data=png.tobytes()))
            node.process()
            self.assertEqual(len(results),1,getattr(node.health,'details',{}));self.assertEqual(len(results[0].targets),1)
            target=results[0].targets[0]
            self.assertEqual(target.class_id,6);self.assertTrue(target.measured)
            np.testing.assert_allclose([target.position.x,target.position.y,target.position.z],[5,0,10],atol=.05)
            stable_ids=set(node.tracker.tracks)
            # A later image with only older raw depth must not reset identity
            # or velocity confidence by rewinding the tracker timestamp.
            del node.roi[t]
            node.s['roi_lidar']=False
            stale=t+10_000_000
            node.receive_ids(CompressedImage(header=header(stale,'drone1/surround_mosaic'),format='png',data=png.tobytes()))
            node.process()
            self.assertEqual(len(results),1);self.assertEqual(set(node.tracker.tracks),stable_ids)
            self.assertEqual(node.health.details['status'],'OUT_OF_ORDER_DEPTH')
        finally:node.destroy_node()


if __name__=='__main__':unittest.main()
