import os
from pathlib import Path
import struct
import sys
import threading
import time
import unittest
import uuid
import mmap

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'/'evtol_simulation'))
from evtol_simulation.runtime import AsyncRaycasts, SensorLease, BufferedImu, RateGate, AcquisitionWait
from evtol_simulation.camera_ring import (SharedMemoryCameraReader, RING_HEADER,
    SLOT_HEADER, MAGIC, aligned_slot_size, expected_mapping_size)
from evtol_simulation.lidar_ring import LidarSharedMemory
from evtol_simulation.backend import AirSimBackend


class AcquisitionRuntimeTests(unittest.TestCase):
    def test_waiter_stops_without_waiting_and_closes_idempotently(self):
        waiter=AcquisitionWait()
        stop=threading.Event();stop.set()
        try:
            begun=time.perf_counter()
            waiter.wait(.5,stop)
            self.assertLess(time.perf_counter()-begun,.1)
        finally:
            waiter.close();waiter.close()

    def test_bulk_validator_scopes_observe_every_stream_exactly_once(self):
        import rclpy
        from evtol_simulation.validation import Validator
        from evtol_simulation.configuration import load
        config=load(Path(__file__).resolve().parents[1]/'src/evtol_simulation/config/simulation.yaml')
        rclpy.init()
        try:
            whole=Validator(config,0.,scope='all')
            expected=set(whole.stats)-{'/clock'};whole.destroy_node()
            seen=set()
            for scope in ('core','rgb','semantic','labels','imu'):
                node=Validator(config,0.,scope=scope)
                try:
                    topics=set(node.report(5.,False)['streams'])-{'/clock'}
                    self.assertTrue(topics)
                    self.assertFalse(seen&topics)
                    seen.update(topics)
                finally:node.destroy_node()
            self.assertEqual(seen,expected)
        finally:rclpy.shutdown()

    def test_imu_deadline_recovers_jitter_with_bounded_spacing_and_skips_long_stall(self):
        gate=RateGate(100.,recover_phase=True)
        gate.accepted(0.)
        gate.accepted(.023)
        self.assertAlmostEqual(gate.due,.028)
        self.assertFalse(gate.ready(.027))
        gate.accepted(.028)
        self.assertAlmostEqual(gate.due,.033)
        gate.accepted(.033)
        self.assertAlmostEqual(gate.due,.040)
        gate.accepted(.200)
        self.assertGreater(gate.dropped_slots,0)
        self.assertGreaterEqual(gate.due,.205)

    def test_imu_buffer_delays_decimates_and_never_reuses_or_restamps(self):
        # Exercise selection deterministically, without relying on OS scheduling.
        sampler=BufferedImu.__new__(BufferedImu)
        from collections import deque
        sampler.latency=.05;sampler.lock=threading.Lock()
        sampler.samples=deque(maxlen=4);sampler.last_ns=0
        sampler.error=None;sampler.overruns=0
        for received,ns in [(1.,100),(1.003,103),(1.006,106),(1.009,109)]:
            sampler.store(received,ns,{'sample':ns})
        self.assertIsNone(sampler.read(1.04))
        self.assertEqual(sampler.read(1.057),(106,{'sample':106}))
        self.assertIsNone(sampler.read(1.057))
        self.assertEqual(sampler.read(1.06),(109,{'sample':109}))
        sampler.store(1.01,109,{'sample':'duplicate'})
        self.assertIsNone(sampler.read(2.))
        with self.assertRaises(ValueError):sampler.store(1.02,108,{})

    def test_scoped_validator_reports_without_absent_lidar_stream(self):
        import rclpy
        from evtol_simulation.validation import Validator
        from evtol_simulation.configuration import load
        config=load(Path(__file__).resolve().parents[1]/'src/evtol_simulation/config/simulation.yaml')
        rclpy.init()
        try:
            node=Validator(config,0.,scope='imu')
            try:
                report=node.report(5.,False)
                self.assertEqual(set(report['streams']),{'imu/data','/clock'})
                self.assertFalse(report['passed'])
            finally:node.destroy_node()
        finally:rclpy.shutdown()

    def test_radio_capture_uses_truth_position_and_waits_for_real_los(self):
        class Cache:
            def update(self, position): self.position=tuple(position)
            def ready(self): return True
        backend=AirSimBackend.__new__(AirSimBackend)
        backend.c={'camera':{}}
        backend.kind='radio'
        backend.los_cache=Cache()
        backend.camera_readers=[]
        truth=(123,{'position':{'x_val':1.,'y_val':2.,'z_val':3.}})
        backend.truth=lambda:truth
        self.assertEqual(backend.capture(),truth)
        self.assertEqual(backend.los_cache.position,(1.,2.,3.))

    def test_gpu_completion_reordered_without_changing_stamps(self):
        backend=AirSimBackend.__new__(AirSimBackend)
        backend.kind='camera';backend.camera_readers=[object()]
        backend.c={'camera':{'reorder_window_s':.2}}
        backend.camera_pending=[];backend.camera_latest_ns=backend.camera_emitted_ns=0
        frames=iter([1000,1100,1050,1150,1200,1250])
        def capture():
            ns=next(frames)*1_000_000
            backend.last_frame_id=getattr(backend,'last_frame_id',0)+1
            return ns,('rgb','semantic')
        backend._capture=capture
        result=[backend.capture() for _ in range(6)]
        self.assertEqual([item[0] for item in result if item],[1_000_000_000,1_050_000_000])

    def test_raycasts_do_not_block_publisher_or_invent_results(self):
        entered, release = threading.Event(), threading.Event()
        def query(position, station):
            entered.set()
            release.wait(2.)
            return station[0] > position[0]
        cache = AsyncRaycasts(query, [(1.,0.,0.),(-1.,0.,0.)], .05, .5)
        try:
            cache.update([0.,0.,0.])
            self.assertTrue(entered.wait(1.))
            self.assertFalse(cache.ready())
            with self.assertRaises(RuntimeError): cache.lookup((1.,0.,0.))
            release.set()
            deadline=time.perf_counter()+1.
            while not cache.ready() and time.perf_counter()<deadline: time.sleep(.001)
            self.assertTrue(cache.lookup((1.,0.,0.)))
            self.assertFalse(cache.lookup((-1.,0.,0.)))
            with cache.lock: cache.updated=time.perf_counter()-1.
            with self.assertRaises(RuntimeError): cache.ready()
        finally:
            release.set()
            cache.close()

    @unittest.skipUnless(os.name=='nt','Windows mappings and mutexes')
    def test_camera_drains_surviving_frames_in_order(self):
        name='evtol_order_'+uuid.uuid4().hex
        w,h,slots=2,2,4
        size=expected_mapping_size(w,h,slots)
        slot_size=aligned_slot_size(w*h*3)
        writer=mmap.mmap(-1,size,tagname=name,access=mmap.ACCESS_WRITE)
        RING_HEADER.pack_into(writer,0,MAGIC,1,RING_HEADER.size,slots,SLOT_HEADER.size,
            w,h,3,w*3,1,slot_size,size,6,0)
        for i in range(3,7):
            offset=RING_HEADER.size+(i%slots)*slot_size
            SLOT_HEADER.pack_into(writer,offset,2,i,i*1000,w,h,w*3,w*h*3,0)
            writer[offset+SLOT_HEADER.size:offset+SLOT_HEADER.size+w*h*3]=bytes([i]*12)
        reader=SharedMemoryCameraReader(name,w,h,slots)
        try:
            self.assertEqual(reader.wait_for_new_frame(1,.1).frame_id,3)
            self.assertEqual(reader.wait_for_new_frame(3,.1).frame_id,4)
            self.assertEqual(reader.wait_for_new_frame(0,.1).frame_id,6)
        finally: reader.close();writer.close()

    @unittest.skipUnless(os.name=='nt','Windows mappings and mutexes')
    def test_lidar_drains_burst_in_order(self):
        name='evtol_order_'+uuid.uuid4().hex
        writer=LidarSharedMemory(name,16,4,create=True)
        reader=LidarSharedMemory(name,16,4)
        try:
            for i in range(1,7): writer.publish(i,1000*i,struct.pack('<3f',i,0,0),1)
            self.assertEqual(reader.read_next(1).frame_id,3)
            self.assertEqual(reader.read_next(3).frame_id,4)
            self.assertEqual(reader.read_latest(3).frame_id,6)
        finally: reader.close();writer.close()

    @unittest.skipUnless(os.name=='nt','Windows mutexes')
    def test_duplicate_writer_rejected_and_lease_released(self):
        config={'backend':'airsim','airsim':{'host':'test_'+uuid.uuid4().hex,'port':41451,'vehicle':'Drone1'}}
        lease=SensorLease(config,'camera')
        try:
            with self.assertRaises(RuntimeError): SensorLease(config,'camera')
            other=SensorLease(config,'lidar');other.close()
        finally: lease.close()
        lease=SensorLease(config,'camera');lease.close()


if __name__=='__main__': unittest.main()
