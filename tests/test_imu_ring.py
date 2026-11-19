import mmap
import os
import struct
import unittest
import uuid
from evtol_simulation.imu_ring import SharedMemoryImu,HEADER,SLOT,MAGIC


@unittest.skipUnless(os.name=='nt','Windows named mappings')
class ImuHistoryTests(unittest.TestCase):
    def setUp(self):
        self.name='evtol_imu_test_'+uuid.uuid4().hex
        self.writer=mmap.mmap(-1,HEADER.size+1024*SLOT.size,tagname=self.name,access=mmap.ACCESS_WRITE)
        HEADER.pack_into(self.writer,0,MAGIC,1,HEADER.size,1024,SLOT.size,0)
        self.reader=SharedMemoryImu(self.name,100.,.1)

    def tearDown(self):
        self.reader.close();self.reader.close();self.writer.close()

    def publish(self,frame,ns):
        offset=HEADER.size+(frame%1024)*SLOT.size
        SLOT.pack_into(self.writer,offset,2,frame,ns,0.,0.,0.,1.,float(frame),2.,3.,4.,5.,6.)
        struct.pack_into('<Q',self.writer,16,frame)

    def test_catch_up_burst_retains_real_samples_and_does_not_repeat(self):
        self.publish(1,1_000_000_000)
        self.assertIsNone(self.reader.read())
        for frame in range(2,101):self.publish(frame,1_000_000_000+(frame-1)*3_000_000)
        samples=[]
        for _ in range(21):
            sample=self.reader.read()
            if sample:samples.append(sample)
        self.assertEqual(len(samples),20)
        stamps=[sample[0] for sample in samples]
        self.assertEqual(len(set(stamps)),len(stamps))
        self.assertTrue(all((ns-1_000_000_000)%3_000_000==0 for ns in stamps))
        self.assertLessEqual(max(b-a for a,b in zip(stamps,stamps[1:])),12_000_000)
        for ns,data in samples:
            self.assertEqual(data['angular_velocity']['x_val'],1+(ns-1_000_000_000)//3_000_000)
        self.assertIsNone(self.reader.read())

    def test_overrun_recovers_without_fabricating_samples(self):
        self.publish(1,1_000_000_000);self.reader.read()
        for i in range(2,1101):self.publish(i,1_000_000_000+i*3_000_000)
        sample=self.reader.read()
        self.assertIsNotNone(sample)
        self.assertEqual((sample[0]-1_000_000_000)%3_000_000,0)
        self.assertGreater(self.reader.overruns,0)
        self.assertGreater(self.reader.resyncs,0)
        self.assertTrue(all((ns-1_000_000_000)%3_000_000==0 for ns,_ in self.reader.samples))

    def test_torn_slot_is_not_consumed(self):
        self.publish(1,1_000_000_000)
        offset=HEADER.size+SLOT.size
        struct.pack_into('<q',self.writer,offset,3)
        self.assertIsNone(self.reader.read())
        self.assertEqual(self.reader.last_id,0)

    def test_empty_history_and_clock_reset_are_not_fabricated(self):
        self.assertIsNone(self.reader.read())
        self.publish(1,1_000_000_000);self.reader.read()
        self.publish(2,900_000_000)
        with self.assertRaisesRegex(RuntimeError,'clock reset'):self.reader.read()
