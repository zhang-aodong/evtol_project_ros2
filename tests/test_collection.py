import copy
import json
from pathlib import Path
import tempfile
import time
import unittest
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Header,String
from builtin_interfaces.msg import Time
from evtol_collection.options import defaults,validate
from evtol_collection.dataset import ExactPairs,DatasetWriter,labels_from_mask,image_array
from evtol_collection.recorder import ByteQueue,Collector

def message(array,t=100,frame='optical',padding=0):
    mask=array.ndim==2;channels=1 if mask else 3;h,w=array.shape[:2]
    data=np.zeros((h,w*channels+padding),np.uint8);data[:,:w*channels]=array.reshape(h,-1)
    return Image(header=Header(stamp=Time(sec=t//1_000_000_000,nanosec=t%1_000_000_000),frame_id=frame),
        width=w,height=h,encoding='mono8' if mask else 'bgr8',step=data.shape[1],data=data.tobytes())

class LabelTests(unittest.TestCase):
    def test_original_masks_and_contiguous_yolo_class_mapping(self):
        o=defaults()['dataset'];o.update(class_ids=[3,6],min_area_px=2)
        mask=np.zeros((20,30),np.uint8);mask[2:8,3:9]=3;mask[10:16,20:27]=6;mask[0,0]=255
        boxes,polygons,stats=labels_from_mask(mask,o)
        self.assertEqual([line.split()[0] for line in boxes],['0','1'])
        np.testing.assert_allclose(list(map(float,boxes[0].split()[1:])),[.2,.25,.2,.3],atol=1e-7)
        self.assertEqual(len(polygons),2);self.assertEqual(stats['unknown_pixels'],1)
        with tempfile.TemporaryDirectory() as tmp:
            writer=DatasetWriter(tmp,o,'test');writer.write(message(np.zeros((20,30,3),np.uint8),padding=2),message(mask,padding=1));writer.close()
            saved=cv2.imdecode(np.frombuffer(next(Path(tmp).glob('semantic/masks/train/*.png')).read_bytes(),np.uint8),0)
            np.testing.assert_array_equal(saved,mask)
            schema=json.loads((Path(tmp)/'label_schema.json').read_text(encoding='utf8'))
            self.assertFalse(schema['instance_labels']);self.assertEqual(schema['yolo_to_source'],{'0':3,'1':6})
            self.assertEqual(len(list(Path(tmp).glob('detection/labels/train/*.txt'))),1)
    def test_touching_regions_and_holes_are_explicit(self):
        o=defaults()['dataset'];o['min_area_px']=2
        mask=np.zeros((30,30),np.uint8);mask[2:20,2:20]=6;mask[6:10,6:10]=0
        boxes,polygons,stats=labels_from_mask(mask,o)
        self.assertEqual(len(boxes),1);self.assertEqual(len(polygons),1);self.assertEqual(stats['regions_holes'],1)
        self.assertTrue(all(0<=v<=1 for v in map(float,polygons[0].split()[1:])))
    def test_negative_frames_have_empty_labels_and_exact_masks(self):
        o=defaults()['dataset']
        with tempfile.TemporaryDirectory() as tmp:
            writer=DatasetWriter(tmp,o,'negative');writer.write(message(np.zeros((20,30,3),np.uint8)),message(np.zeros((20,30),np.uint8)));writer.close()
            self.assertEqual(next(Path(tmp).glob('detection/labels/train/*.txt')).read_text(),'')
            self.assertEqual(writer.empty,1)
    def test_pairing_never_uses_nearest_or_repeated_or_reset_frames(self):
        a=np.zeros((2,2,3),np.uint8);b=np.zeros((2,2),np.uint8);pairs=ExactPairs(2)
        self.assertIsNone(pairs.add(0,message(a,100)));self.assertIsNone(pairs.add(1,message(b,101)))
        self.assertIsNotNone(pairs.add(0,message(a,101)))
        self.assertIsNone(pairs.add(0,message(a,101)))
        with self.assertRaisesRegex(ValueError,'reset'):pairs.add(0,message(a,99))
        pairs.finish();self.assertEqual(pairs.unmatched,1)
    def test_invalid_frames_options_and_byte_budget_fail_clearly(self):
        q=ByteQueue(4);self.assertTrue(q.put('a',3));self.assertFalse(q.put('b',2));self.assertEqual(q.get(),'a');q.finish();self.assertIsNone(q.get())
        for change in [lambda d:d['dataset'].update(class_ids=[0]),lambda d:d['bag'].update(topics=['/a','/a']),
            lambda d:d['dataset'].update(rate_hz=float('nan')),lambda d:d['dataset'].update(labels=[]),
            lambda d:d['dataset'].update(namespace='bad/../camera')]:
            o=defaults();change(o)
            with self.assertRaises(ValueError):validate(o)
        mask=np.full((3,3),14,np.uint8)
        with self.assertRaisesRegex(ValueError,'Unknown truth'):labels_from_mask(mask,defaults()['dataset'])
    def test_independent_splits_combine_with_valid_class_maps(self):
        from evtol_collection.build_index import build
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);sessions=[]
            for split in ('train','val'):
                folder=root/split;folder.mkdir();o=defaults()['dataset'];o['split']=split
                writer=DatasetWriter(folder,o,split);writer.write(message(np.zeros((20,30,3),np.uint8)),message(np.zeros((20,30),np.uint8)));writer.close()
                (folder/'collection.json').write_text(json.dumps(dict(complete=True,mode='dataset')))
                sessions.append(folder)
            output=build(sessions,root/'index','detection')
            self.assertTrue(output.exists())
            self.assertEqual(len((root/'index/train.txt').read_text().splitlines()),1)
            with self.assertRaisesRegex(ValueError,'train AND val'):build(sessions[:1],root/'bad','detection')

class RosBagIntegration(unittest.TestCase):
    def test_selected_topics_flush_readback_and_transient_local(self):
        import rclpy,rosbag2_py
        from rclpy.node import Node
        from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.serialization import deserialize_message
        rclpy.init();pub=Node('collection_test_publisher');node=None;executor=SingleThreadedExecutor()
        try:
            normal=pub.create_publisher(String,'/collection_test/text',10)
            omitted=pub.create_publisher(String,'/collection_test/omitted',10)
            qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            latched=pub.create_publisher(String,'/collection_test/latch',qos);latched.publish(String(data='origin'))
            for storage in ('mcap','sqlite3'):
                with tempfile.TemporaryDirectory() as tmp:
                    o=defaults();o['bag'].update(topics=['/collection_test/text','/collection_test/latch'],storage_id=storage,use_sim_time=False)
                    node=Collector(o,'bag',tmp,'bag_test');executor.add_node(pub);executor.add_node(node)
                    end=time.monotonic()+5
                    while not node.ready and not node.error and time.monotonic()<end:executor.spin_once(timeout_sec=.05)
                    self.assertFalse(node.error);self.assertTrue(node.ready)
                    for _ in range(15):
                        normal.publish(String(data='selected'));omitted.publish(String(data='not selected'))
                        executor.spin_once(timeout_sec=.04)
                    end=time.monotonic()+1
                    while time.monotonic()<end:executor.spin_once(timeout_sec=.02)
                    node.stop_requested=True;node.finish()
                    self.assertEqual(node.state,'completed');self.assertGreater(node.counts['/collection_test/text'],0)
                    reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(Path(tmp)/'rosbag2'),storage_id=storage),rosbag2_py.ConverterOptions('',''))
                    self.assertEqual({t.name for t in reader.get_all_topics_and_types()},set(o['bag']['topics']))
                    seen=[]
                    while reader.has_next():
                        topic,data,t=reader.read_next();seen.append((topic,deserialize_message(data,String).data))
                    self.assertIn(('/collection_test/latch','origin'),seen)
                    del reader
                    executor.remove_node(node);executor.remove_node(pub);node.destroy_node();node=None
        finally:
            if node:
                if not node.finalized:node.stop_requested=True;node.finish()
                node.destroy_node()
            executor.shutdown();pub.destroy_node();rclpy.shutdown()

if __name__=='__main__':unittest.main()
