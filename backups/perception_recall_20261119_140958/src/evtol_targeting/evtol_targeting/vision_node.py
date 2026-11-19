"""Bounded latest-image inference; the ROS environment never imports torch."""
import json
import os
from multiprocessing import shared_memory
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CompressedImage
from evtol_perception_interfaces.msg import SemanticFrame
from .common import args_config, project_path, clone, Health, LATEST, ns


class ModelClient:
    def __init__(self,c):
        m=c['model'];python=os.environ.get('EVTOL_MODEL_PYTHON',m['python'])
        machine=project_path(c,'config/machine.local.json')
        if not python and machine.exists():python=json.loads(machine.read_text(encoding='utf-8-sig')).get('ModelPython','')
        if not Path(python).is_file():raise FileNotFoundError('Configure ModelPython in config/machine.local.json or EVTOL_MODEL_PYTHON')
        worker=Path(__file__).with_name('model_worker.py')
        env=os.environ.copy();env.pop('PYTHONHOME',None)
        env['PYTHONPATH']=str(project_path(c,m['vendor']));env['YOLO_AUTOINSTALL']='false'
        env['OMP_NUM_THREADS']='1';env['OPENBLAS_NUM_THREADS']='1'
        env['PATH']=os.pathsep.join([str(Path(python).parent/p) for p in ('','Library/bin','Scripts')]+[env['PATH']])
        self.process=subprocess.Popen([python,'-u',str(worker),'--weights',str(project_path(c,m['weights'])),
            '--vendor',str(project_path(c,m['vendor'])),'--device',str(m['device']),'--imgsz',str(m['imgsz']),
            '--parent-pid',str(os.getpid())],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=None,text=True,encoding='utf8',env=env,
            creationflags=subprocess.CREATE_NO_WINDOW if os.name=='nt' else 0)
        self.responses=queue.Queue(maxsize=2);self.mapping=None
        self.reader=threading.Thread(target=self.read,daemon=True);self.reader.start()
        self.timeout=float(m['timeout_s'])
        try:
            self.info=self.responses.get(timeout=float(m['startup_timeout_s']))
            if not self.info.get('ready'):raise RuntimeError(str(self.info))
        except Exception:self.close();raise

    def read(self):
        try:
            for line in self.process.stdout:
                self.responses.put(json.loads(line))
        except Exception as exc:self.responses.put({'error':str(exc)})
        finally:
            try:self.responses.put_nowait({'error':'Model worker exited'})
            except queue.Full:pass

    def predict(self,image,sequence):
        h,w=image.shape[:2];size=h*w*7
        if self.mapping is None or self.mapping.size<size:
            if self.mapping:self.mapping.close();self.mapping.unlink()
            self.mapping=shared_memory.SharedMemory(create=True,size=size)
        input_view=np.ndarray((h,w,3),np.uint8,buffer=self.mapping.buf);input_view[:]=image;del input_view
        self.process.stdin.write(json.dumps(dict(sequence=sequence,height=h,width=w,shm=self.mapping.name))+'\n');self.process.stdin.flush()
        response=self.responses.get(timeout=self.timeout)
        if response.get('error') or response.get('sequence')!=sequence:raise RuntimeError(str(response))
        mask=np.ndarray((h,w),np.int32,buffer=self.mapping.buf,offset=h*w*3).copy()
        return mask,response['ms']

    def close(self):
        if getattr(self,'process',None):
            self.process.terminate()
            try:self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait(timeout=5)
        if self.mapping:self.mapping.close();self.mapping.unlink();self.mapping=None


class VisionNode(Node):
    def __init__(self,c):
        super().__init__('semantic_inference');self.c=c;self.health=Health(self,'/'+c['namespace']+'/vision')
        self.pending=None;self.lock=threading.Lock();self.wake=threading.Event();self.stop=threading.Event();self.sequence=0
        self.pub=self.create_publisher(SemanticFrame,'/'+c['namespace']+'/perception/semantic',qos_profile_sensor_data)
        self.preview=self.create_publisher(CompressedImage,'/'+c['namespace']+'/perception/semantic_preview/compressed',qos_profile_sensor_data)
        self.input_topic=c['model'].get('input_topic','camera/rgb/image_realtime')
        self.create_subscription(Image,'/'+c['namespace']+'/'+self.input_topic,self.receive,LATEST)
        self.client=None;self.worker=threading.Thread(target=self.run,daemon=True);self.worker.start()

    def receive(self,msg):
        with self.lock:
            if self.pending is not None:self.health.dropped+=1
            self.pending=msg
        self.wake.set()

    def run(self):
        try:
            self.client=ModelClient(self.c);self.get_logger().info('Model ready: '+str(self.client.info))
            while not self.stop.is_set():
                self.wake.wait(.1)
                with self.lock:msg=self.pending;self.pending=None;self.wake.clear()
                if msg is None:continue
                begun=time.perf_counter()
                if msg.encoding not in ('bgr8','rgb8') or msg.step<msg.width*3:raise ValueError('Expected packed RGB/BGR Image')
                image=np.ndarray((msg.height,msg.width,3),np.uint8,buffer=msg.data,strides=(msg.step,3,1))
                if msg.encoding=='rgb8':image=image[:,:,::-1]
                scale=min(1.,self.c['model']['imgsz']/max(msg.height,msg.width))
                image=cv2.resize(image,(round(msg.width*scale),round(msg.height*scale)),interpolation=cv2.INTER_AREA)
                self.sequence+=1;mask,ms=self.client.predict(image,self.sequence)
                result=SemanticFrame(header=clone(msg.header),sequence=self.sequence,width=mask.shape[1],height=mask.shape[0],
                    inference_ms=float(ms),model_sha256=self.client.info['sha256'])
                result.class_ids.frombytes(np.ascontiguousarray(mask,dtype='<i4').tobytes())
                self.pub.publish(result);self.health.ok()
                self.health.details=dict(input_topic=self.input_topic,inference_ms=round(ms,2),
                    output_age_ms=round((self.get_clock().now().nanoseconds-ns(msg.header.stamp))*1e-6,2),
                    drone_pixels=int(np.count_nonzero(mask==6)),input_width=msg.width,input_height=msg.height)
                palette=np.array([[0,0,0],[6,108,153],[191,105,112],[72,121,89],[64,225,190],[59,190,206],[36,13,81]],np.uint8)
                colored=palette[np.clip(mask,0,6)]
                valid,jpeg=cv2.imencode('.jpg',colored,[cv2.IMWRITE_JPEG_QUALITY,80])
                if valid:self.preview.publish(CompressedImage(header=clone(msg.header),format='jpeg',data=jpeg.tobytes()))
                self.stop.wait(max(0.,1./self.c['model']['max_hz']-(time.perf_counter()-begun)))
        except Exception as exc:
            self.health.error(exc)
            # A timed-out worker might still write its mapping. Never reuse it.
        finally:
            if self.client:self.client.close()

    def close(self):
        self.stop.set();self.wake.set()
        if self.client:self.client.process.terminate()
        self.worker.join(timeout=8.)


def main(args=None):
    from evtol_simulation.runtime import TimerResolution
    resolution=TimerResolution();cv2.setNumThreads(1)
    c,ros=args_config(args);rclpy.init(args=ros);node=VisionNode(c)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.close();node.destroy_node();rclpy.shutdown();resolution.close()
