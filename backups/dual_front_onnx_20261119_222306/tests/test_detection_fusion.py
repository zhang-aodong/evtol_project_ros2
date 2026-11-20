import copy
import json
from pathlib import Path
import unittest
import numpy as np
import yaml
import rclpy
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2,PointField
from std_msgs.msg import String
from evtol_perception_interfaces.msg import ReferenceOrigin
from evtol_targeting.common import header
from evtol_targeting.detection_fusion import DetectionFusion

ROOT=Path(__file__).resolve().parents[1]


class DetectionFusionContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):rclpy.init(domain_id=230)
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()

    def setup_node(self):
        c=yaml.safe_load((ROOT/'config/targeting.yaml').read_text(encoding='utf8'));c['_root']=ROOT
        node=DetectionFusion(c);self.addCleanup(node.destroy_node)
        node.receive_origin(ReferenceOrigin(session_id='detect-test'))
        now=node.get_clock().now().nanoseconds;node.set_parameters([Parameter('use_sim_time',value=True)])
        node.get_clock().set_ros_time_override(Time(nanoseconds=now));t=now-50_000_000
        for dt in (-30_000_000,30_000_000):node.poses.add(t+dt,[0,0,10],[0,0,0,1])
        p=np.asarray([[5,0,0],[5,.03,.03],[5,-.03,-.03]],dtype='<f4')
        node.receive_cloud(PointCloud2(header=header(t,'drone1/base_link'),height=1,width=len(p),
            fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z'))],
            point_step=12,row_step=12*len(p),data=p.tobytes()))
        results=[];requests=[]
        class Capture:
            def __init__(self,items):self.items=items
            def publish(self,m):self.items.append(copy.deepcopy(m))
        node.measurements=Capture(results);node.roi_pub=Capture(requests)
        payload=dict(schema_version=1,stamp_ns=t,sequence=1,width=c['surround']['width'],height=c['surround']['height'],
            cameras=[v['name'] for v in c['surround']['cameras']],
            detections=[dict(view_index=0,source_class_id=0,class_id=6,confidence=.95,xyxy=[365,365,403,403])])
        return node,t,payload,results,requests

    def test_bbox_plus_real_projected_points_yields_canonical_enu_target(self):
        node,t,payload,results,requests=self.setup_node()
        node.receive_boxes(String(data=json.dumps(payload)))
        self.assertTrue(requests);self.assertLessEqual(len(json.loads(requests[0].data)['views'][0]['rays']),640)
        node.roi[t]=[dict(camera='surround_front',stamp_ns=t+10_000_000,points=[[0,0,5],[.03,.03,5],[-.03,-.03,5]])]
        node.process();self.assertEqual(len(results),1,node.health.details)
        self.assertEqual(len(results[0].targets),1,node.health.details)
        target=results[0].targets[0]
        self.assertEqual(target.class_id,6);self.assertTrue(target.measured)
        np.testing.assert_allclose([target.position.x,target.position.y,target.position.z],[5,0,10],atol=.05)
        self.assertFalse(any('semantic' in s.topic_name for s in node.subscriptions))

    def test_box_without_any_depth_cannot_create_target(self):
        node,t,payload,results,_=self.setup_node();node.clouds.clear()
        node.receive_boxes(String(data=json.dumps(payload)));node.process()
        self.assertFalse(results);self.assertFalse(node.tracker.tracks)

    def test_empty_frame_publishes_negative_observation_and_session_reset_clears_prefetch(self):
        node,t,payload,results,_=self.setup_node();payload['detections']=[]
        node.receive_boxes(String(data=json.dumps(payload)));node.roi[t]=[];node.process()
        self.assertEqual(len(results),1);self.assertEqual(results[0].status,'NO_DETECTIONS')
        self.assertFalse(results[0].targets)
        node.roi_requested.add(t);node.last_roi_views=[{'camera':'surround_front','rays':[[0,0]]}]
        node.receive_origin(ReferenceOrigin(session_id='new-session'))
        self.assertFalse(node.roi);self.assertFalse(node.roi_requested);self.assertIsNone(node.last_roi_views)

    def test_out_of_order_depth_does_not_reset_identity(self):
        node,t,payload,results,_=self.setup_node()
        node.last_lidar_ns=t+20_000_000;node.tracker.tracks[77]={}
        node.receive_boxes(String(data=json.dumps(payload)));node.roi[t]=[];node.process()
        self.assertFalse(results);self.assertIn(77,node.tracker.tracks)
        self.assertEqual(node.health.details['status'],'OUT_OF_ORDER_DEPTH')

    def test_wrong_camera_order_is_rejected(self):
        node,t,payload,results,_=self.setup_node();payload['cameras'].reverse()
        node.receive_boxes(String(data=json.dumps(payload)))
        self.assertFalse(node.pending);self.assertFalse(results)


if __name__=='__main__':unittest.main()
