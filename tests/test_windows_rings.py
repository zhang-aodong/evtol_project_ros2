import mmap
import os
from pathlib import Path
import struct
import sys
import unittest
import uuid

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'/'evtol_simulation'))
from evtol_simulation import camera_ring as camera
from evtol_simulation.lidar_ring import LidarSharedMemory


@unittest.skipUnless(os.name=='nt','Windows named mappings required')
class WindowsRings(unittest.TestCase):
    def test_camera_real_mapping_and_guard(self):
        name='evtol_test_camera_'+uuid.uuid4().hex
        w,h,slots=4,3,8
        size=camera.expected_mapping_size(w,h,slots)
        slot_size=camera.aligned_slot_size(w*h*3)
        producer=mmap.mmap(-1,size,tagname=name,access=mmap.ACCESS_WRITE)
        try:
            camera.RING_HEADER.pack_into(producer,0,camera.MAGIC,1,camera.RING_HEADER.size,
                slots,camera.SLOT_HEADER.size,w,h,3,w*3,1,slot_size,size,1,0)
            offset=camera.RING_HEADER.size+slot_size
            camera.SLOT_HEADER.pack_into(producer,offset,2,1,123456789,w,h,w*3,w*h*3,0)
            payload=bytes(range(w*h*3))
            begin=offset+camera.SLOT_HEADER.size
            producer[begin:begin+len(payload)]=payload
            reader=camera.SharedMemoryCameraReader(name,w,h,slots)
            try:
                frame=reader.read_latest()
                self.assertEqual(frame.bgr_bytes,payload)
                self.assertEqual(frame.timestamp_ns,123456789)
                struct.pack_into('<q',producer,offset,3)
                self.assertIsNone(reader._try_read_frame(1))
            finally: reader.close()
        finally: producer.close()

    def test_lidar_real_mapping_and_fresh_frame(self):
        name='evtol_test_lidar_'+uuid.uuid4().hex
        writer=LidarSharedMemory(name,16,8,create=True)
        try:
            reader=LidarSharedMemory(name,16,8)
            try:
                self.assertIsNone(reader.read_latest())
                points=struct.pack('<ffffff',1,2,3,4,5,6)
                writer.publish(1,123456789,points,2)
                frame=reader.read_latest()
                self.assertEqual(frame.xyz_bytes,points)
                self.assertEqual(frame.point_count,2)
                self.assertFalse(frame.scan_complete)  # old producer: unknown/partial
                self.assertIsNone(reader.read_latest(after_id=1))
                for i in range(2,9):writer.publish(i,123456789+i,points,2)
                self.assertEqual(reader.read_next(after_id=1).frame_id,2)
                self.assertEqual(reader.read_latest(after_id=1).frame_id,8)
                writer.publish(9,123456800,points,2,scan_complete=True)
                self.assertTrue(reader.read_latest(after_id=8).scan_complete)
                writer.publish(10,123456801,points*9,18,scan_complete=True)
                truncated=reader.read_latest(after_id=9)
                self.assertEqual(truncated.point_count,16)
                self.assertFalse(truncated.scan_complete)
            finally: reader.close()
        finally: writer.close()
