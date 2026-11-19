"""Latest synchronized RGB cube faces -> learned semantic mosaic (no truth)."""
import copy
import time
import threading
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from evtol_perception_interfaces.msg import SemanticFrame
from evtol_simulation.camera_ring import SharedMemoryCameraReader
from evtol_simulation.runtime import TimerResolution
from .vision_node import ModelClient
from .common import args_config, Health, LATEST, header


class SurroundVision(Node):
    def __init__(self,c):
        super().__init__('surround_vision');self.c=c;self.s=c['surround']
        root='/'+c['namespace']+'/perception/surround'
        self.pub=self.create_publisher(SemanticFrame,root+'/semantic',LATEST)
        self.rgb=self.create_publisher(CompressedImage,root+'/rgb/compressed',LATEST)
        self.preview=self.create_publisher(CompressedImage,root+'/semantic_preview/compressed',LATEST)
        self.health=Health(self,'/'+c['namespace']+'/surround_vision')
        self.stop=threading.Event();self.client=None;self.readers=[]
        self.worker=threading.Thread(target=self.run,daemon=True);self.worker.start()

    def run(self):
        try:
            c=copy.deepcopy(self.c);c['model']['imgsz']=self.s['imgsz']
            self.client=ModelClient(c)
            self.get_logger().info('Surround semantic model ready: '+str(self.client.info))
            w,h=self.s['width'],self.s['height'];deadline=time.monotonic()+30.
            for v in self.s['cameras']:
                name='Local\\AirSim_'+self.c['airsim']['vehicle']+'_'+v['name']+f'_{w}x{h}_0'
                while not self.stop.is_set():
                    try:self.readers.append(SharedMemoryCameraReader(name,w,h,8));break
                    except OSError:
                        if time.monotonic()>deadline:raise
                        self.stop.wait(.1)
            last=0;seq=0
            while not self.stop.is_set():
                begun=time.perf_counter();first=self.readers[0].read_latest()
                if first.timestamp_ns<=last:self.stop.wait(.003);continue
                frames=[first]
                for reader in self.readers[1:]:
                    latest=reader.latest_frame_id();found=None
                    for i in range(latest,max(0,latest-8),-1):
                        try:f=reader._try_read_frame(i)
                        except RuntimeError:continue  # A resident slot was overwritten while scanning.
                        if f and f.timestamp_ns==first.timestamp_ns:found=f;break
                    if found is None:break
                    frames.append(found)
                if len(frames)!=len(self.readers):self.stop.wait(.003);continue
                last=first.timestamp_ns
                age=(self.get_clock().now().nanoseconds-last)*1e-9
                if age>self.s['max_age_s']:self.health.dropped+=1;continue
                mosaic=np.empty((h*2,w*3,3),np.uint8)
                for i,f in enumerate(frames):
                    mosaic[(i//3)*h:(i//3+1)*h,(i%3)*w:(i%3+1)*w]=np.frombuffer(f.bgr_bytes,np.uint8).reshape(h,w,3)
                seq+=1;mask,ms=self.client.predict(mosaic,seq)
                msg=SemanticFrame(header=header(last,self.c['namespace']+'/surround_mosaic'),sequence=seq,
                    width=mask.shape[1],height=mask.shape[0],inference_ms=float(ms),model_sha256=self.client.info['sha256'])
                msg.class_ids.frombytes(mask.astype('<i4').tobytes());self.pub.publish(msg);self.health.ok()
                self.health.details=dict(inference_ms=round(ms,2),output_age_ms=round((self.get_clock().now().nanoseconds-last)*1e-6,2),
                    views=len(frames),drone_pixels=int(np.count_nonzero(mask==6)))
                palette=np.array([[0,0,0],[6,108,153],[191,105,112],[72,121,89],[64,225,190],[59,190,206],[36,13,81]],np.uint8)
                for pub,im in [(self.rgb,mosaic),(self.preview,palette[np.clip(mask,0,6)])]:
                    ok,jpg=cv2.imencode('.jpg',im,[cv2.IMWRITE_JPEG_QUALITY,75])
                    if ok:pub.publish(CompressedImage(header=msg.header,format='jpeg',data=jpg.tobytes()))
                self.stop.wait(max(0,1/self.s['max_hz']-(time.perf_counter()-begun)))
        except Exception as exc:self.health.error(exc)
        finally:
            if self.client:self.client.close()
            for reader in self.readers:reader.close()

    def close(self):
        self.stop.set()
        if self.client:self.client.process.terminate()
        self.worker.join(timeout=8.)


def main(args=None):
    c,ros=args_config(args);res=TimerResolution();cv2.setNumThreads(1);rclpy.init(args=ros);node=SurroundVision(c)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.close();node.destroy_node();rclpy.shutdown();res.close()
