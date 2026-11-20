"""Independent ROS collector with bounded I/O queues and explicit finalization."""
import argparse
from collections import Counter,deque
import json
from pathlib import Path
import threading
import time
import traceback
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from .dataset import ExactPairs,DatasetWriter,atomic_json,timestamp
from .options import validate
from evtol_simulation.runtime import SensorLease

class ByteQueue:
    def __init__(self,maximum):self.maximum=maximum;self.bytes=0;self.queue=deque();self.condition=threading.Condition();self.closed=False
    def put(self,item,size):
        with self.condition:
            if self.closed or self.bytes+size>self.maximum:return False
            self.queue.append((item,size));self.bytes+=size;self.condition.notify();return True
    def get(self):
        with self.condition:
            while not self.queue and not self.closed:self.condition.wait(.2)
            if not self.queue:return None
            item,size=self.queue.popleft();self.bytes-=size;return item
    def finish(self):
        with self.condition:self.closed=True;self.condition.notify_all()

class Collector(Node):
    def __init__(self,options,mode,folder,session,duration=0):
        super().__init__('training_data_collector');self.o=validate(options);self.mode=mode;self.folder=Path(folder)
        self.session=session;self.started=time.monotonic();self.duration=duration;self.stop_requested=False
        self.error='';self.finalized=False;self.ready=False;self.last_bag_time=0;self.clock_wait=0;self.queue_dropped=0
        self.bytes_written=0;self.counts=Counter();self.types={};self.subs={};self.publisher_qos={}
        self.bag=None;self.dataset=None;self.pending=None;self.last_dataset_time=0;self.worker_error=''
        self.pairs=ExactPairs(self.o['dataset']['pair_capacity']);self.sampled_out=0;self.dataset_written=0
        self.received=[0,0];self.paired=0;self.last_pair_wall=None;self.waiting_latched={}
        self.state='starting';self.status_path=self.folder/'status.json'
        self.pub=self.create_publisher(String,'/simulation/collection/status',10)
        self.service=self.create_service(Trigger,'/simulation/collection/stop',self.request_stop)
        self.steady=Clock(clock_type=ClockType.STEADY_TIME)
        if mode=='bag':
            import rosbag2_py
            self.set_parameters([Parameter('use_sim_time',value=self.o['bag']['use_sim_time'])])
            self.bag=rosbag2_py.SequentialWriter()
            self.bag.open(rosbag2_py.StorageOptions(uri=str(self.folder/'rosbag2'),storage_id=self.o['bag']['storage_id'],
                max_bagfile_size=self.o['bag']['split_size_mb']*1024*1024),rosbag2_py.ConverterOptions('',''))
            self.pending=ByteQueue(self.o['bag']['queue_mb']*1024*1024)
            self.create_timer(.5,self.discover,clock=self.steady)
            # All writer operations stay on the executor until topics are
            # created; the worker starts only when discovery has completed.
        else:
            self.dataset=DatasetWriter(self.folder,self.o['dataset'],session)
            self.pending=ByteQueue(self.o['dataset']['queue_frames']*1920*1080*4)
            self.worker=threading.Thread(target=self.write_loop,daemon=True);self.worker.start()
            root='/'+self.o['dataset']['namespace']+'/camera/'
            qos=QoSProfile(depth=5,reliability=ReliabilityPolicy.BEST_EFFORT)
            self.subs['rgb']=self.create_subscription(Image,root+'rgb/image_raw',lambda m:self.receive_image(0,m),qos)
            self.subs['truth']=self.create_subscription(Image,root+'semantic/class_ids',lambda m:self.receive_image(1,m),qos)
            self.ready=True;self.state='waiting_frames'
        self.create_timer(.5,self.status,clock=self.steady);self.status()

    def request_stop(self,request,response):
        self.stop_requested=True;response.success=True;response.message='Stop accepted; draining queues and finalizing files';return response

    def fail(self,exc):
        if not self.error:self.error=str(exc);self.get_logger().error(self.error)
        self.stop_requested=True

    def discover(self):
        if self.ready or self.stop_requested:return
        try:
            import rosbag2_py
            from rosbag2_py import _storage
            available=dict(self.get_topic_names_and_types())
            missing=[t for t in self.o['bag']['topics'] if t not in available]
            self.missing=missing
            if missing:
                self.state='waiting_topics'
                if time.monotonic()-self.started>20:raise RuntimeError('Selected topics not available: '+', '.join(missing))
                return
            # Topic discovery can precede endpoint discovery on Fast DDS. Do not
            # commit a volatile subscription before the publisher QoS is known:
            # a static reference may only ever have one historical sample.
            endpoints={topic:self.get_publishers_info_by_topic(topic) for topic in self.o['bag']['topics']}
            self.missing=[topic for topic,infos in endpoints.items() if not infos]
            if self.missing:
                self.state='waiting_publishers'
                if time.monotonic()-self.started>20:raise RuntimeError('Selected publishers not discovered: '+', '.join(self.missing))
                return
            for i,topic in enumerate(self.o['bag']['topics']):
                types=available[topic]
                if len(types)!=1:raise ValueError('Ambiguous topic type: '+topic)
                infos=endpoints[topic]
                latched=bool(infos) and all(info.qos_profile.durability==DurabilityPolicy.TRANSIENT_LOCAL for info in infos)
                reliable=bool(infos) and all(info.qos_profile.reliability==ReliabilityPolicy.RELIABLE for info in infos)
                qos=QoSProfile(depth=20,reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL if latched else DurabilityPolicy.VOLATILE)
                # rosbag2 QoS metadata must preserve transient-local replay.
                offered=[]
                for info in infos:
                    q=info.qos_profile;record=_storage.QoS(max(1,q.depth))
                    if q.reliability==ReliabilityPolicy.BEST_EFFORT:record.best_effort()
                    else:record.reliable()
                    if q.durability==DurabilityPolicy.TRANSIENT_LOCAL:record.transient_local()
                    else:record.durability_volatile()
                    offered.append(record)
                self.bag.create_topic(rosbag2_py.TopicMetadata(id=i+1,name=topic,type=types[0],serialization_format='cdr',offered_qos_profiles=offered))
                self.types[topic]=types[0];self.publisher_qos[topic]=dict(transient_local=latched,publishers=len(infos))
                self.subs[topic]=self.create_subscription(get_message(types[0]),topic,lambda data,t=topic:self.receive_raw(t,data),qos,raw=True)
            self.ready=True;self.state='recording'
            self.worker=threading.Thread(target=self.write_loop,daemon=True);self.worker.start()
        except Exception as exc:self.fail(exc)

    def receive_raw(self,topic,data):
        if self.stop_requested:return
        now=self.get_clock().now().nanoseconds
        if now<=0:
            self.clock_wait+=1
            if self.publisher_qos.get(topic,{}).get('transient_local'):self.waiting_latched[topic]=data
            return
        if self.last_bag_time and now<self.last_bag_time:self.fail('ROS clock reset: collection stopped to prevent mixed sessions');return
        self.last_bag_time=now
        if not self.pending.put((topic,data,now),len(data)):self.queue_dropped+=1

    def receive_image(self,kind,msg):
        if self.stop_requested:return
        try:
            self.received[kind]+=1;pair=self.pairs.add(kind,msg)
            if pair is None:return
            self.paired+=1;self.last_pair_wall=time.monotonic();self.state='recording'
            t=timestamp(pair[0]);interval=1e9/self.o['dataset']['rate_hz']
            if self.last_dataset_time and t-self.last_dataset_time<interval:self.sampled_out+=1;return
            self.last_dataset_time=t
            size=len(pair[0].data)+len(pair[1].data)
            # Bound both frame count and bytes, regardless of image resolution.
            with self.pending.condition:count=len(self.pending.queue)
            if count>=self.o['dataset']['queue_frames'] or not self.pending.put(pair,size):self.queue_dropped+=1
        except Exception as exc:self.fail(exc)

    def write_loop(self):
        try:
            while True:
                item=self.pending.get()
                if item is None:break
                if self.mode=='bag':
                    topic,data,t=item;self.bag.write(topic,data,t);self.counts[topic]+=1;self.bytes_written+=len(data)
                else:
                    self.dataset.write(*item);self.dataset_written=self.dataset.saved
        except BaseException as exc:self.worker_error=str(exc);self.stop_requested=True

    def snapshot(self):
        with self.pending.condition:queued=len(self.pending.queue);queued_bytes=self.pending.bytes
        return dict(version=1,session_id=self.session,mode=self.mode,state=self.state,ready=self.ready,
            finalized=self.finalized,output_dir=str(self.folder),elapsed_s=round(time.monotonic()-self.started,2),
            error=self.error or self.worker_error,bag_messages=dict(self.counts),serialized_bytes=self.bytes_written,
            dataset_frames=self.dataset_written,queue_dropped=self.queue_dropped,queue_frames=queued,queue_bytes=queued_bytes,
            unmatched_frames=self.pairs.unmatched,sampled_out=self.sampled_out,clock_wait_messages=self.clock_wait,
            received_rgb=self.received[0],received_truth=self.received[1],paired_frames=self.paired,
            missing_topics=getattr(self,'missing',[]),last_saved_stamp_ns=self.dataset.last if self.dataset else None,
            dataset_objects=self.dataset.objects if self.dataset else 0,negative_frames=self.dataset.empty if self.dataset else 0,
            label_source='UE semantic truth connected regions; no instance IDs' if self.mode=='dataset' else None)

    def status(self):
        if self.worker_error:self.fail(self.worker_error)
        if (self.folder/'stop.request').exists():self.stop_requested=True
        if self.mode=='bag' and self.waiting_latched and not self.stop_requested:
            now=self.get_clock().now().nanoseconds
            if now>0:
                pending=self.waiting_latched;self.waiting_latched={}
                for topic,data in pending.items():self.receive_raw(topic,data)
        if self.mode=='dataset' and self.state=='recording' and self.last_pair_wall and time.monotonic()-self.last_pair_wall>3:
            self.state='waiting_frames'
        if self.duration and time.monotonic()-self.started>=self.duration:self.stop_requested=True
        value=self.snapshot();atomic_json(self.status_path,value);self.pub.publish(String(data=json.dumps(value,ensure_ascii=False)))

    def finish(self):
        self.state='finalizing';self.status();self.pending.finish()
        worker=getattr(self,'worker',None)
        if worker:
            worker.join(timeout=25.)
            if worker.is_alive():
                self.error='Writer did not drain within 25 seconds';self.state='error';self.status()
                raise TimeoutError(self.error)
        if self.worker_error:self.error=self.worker_error
        try:
            if self.bag:self.bag.close();self.bag=None
            if self.dataset:self.dataset.close()
        except Exception as exc:self.error=str(exc)
        self.pairs.finish();self.finalized=True;self.state='error' if self.error else 'completed'
        value=self.snapshot();value.update(options=self.o,topic_types=self.types,publisher_qos=self.publisher_qos,
            complete=not bool(self.error),completed_wall_ns=time.time_ns(),
            empty=not bool(self.dataset_written if self.mode=='dataset' else sum(self.counts.values())))
        atomic_json(self.folder/'collection.json',value);self.status()

def main(args=None):
    p=argparse.ArgumentParser();p.add_argument('--config',required=True);p.add_argument('--mode',choices=['bag','dataset'],required=True)
    p.add_argument('--output',required=True);p.add_argument('--session',required=True);p.add_argument('--duration',type=float,default=0)
    opts,ros=p.parse_known_args(args);folder=Path(opts.output)
    if not folder.is_dir() or any(folder.iterdir()):raise ValueError('Collector output must be a new empty session directory')
    options=validate(json.loads(Path(opts.config).read_text(encoding='utf-8-sig')))
    atomic_json(folder/'collection.json',dict(version=1,state='starting',complete=False,options=options,mode=opts.mode,session_id=opts.session))
    if not 0<=opts.duration<float('inf'):raise ValueError('Duration must be finite and nonnegative')
    rclpy.init(args=ros);node=None;code=0;lease=None
    try:
        # Status and stop service are shared within a ROS domain. Reject a
        # second CLI/console recorder rather than stopping the wrong session.
        domain=rclpy.get_default_context().get_domain_id()
        lease=SensorLease(dict(backend='airsim',airsim=dict(host='collection',port=domain,vehicle='recorder')),'collection')
        node=Collector(options,opts.mode,folder,opts.session,opts.duration)
        while rclpy.ok() and not node.stop_requested:rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:pass
    except BaseException as exc:
        code=1
        if node:node.error=str(exc)
        else:atomic_json(folder/'status.json',dict(state='error',ready=False,finalized=False,error=str(exc),output_dir=str(folder)))
        traceback.print_exc()
    finally:
        if node:
            try:node.finish()
            finally:node.destroy_node()
            code=code or int(bool(node.error))
        if rclpy.ok():rclpy.shutdown()
        if lease:lease.close()
    return code

if __name__=='__main__':raise SystemExit(main())
