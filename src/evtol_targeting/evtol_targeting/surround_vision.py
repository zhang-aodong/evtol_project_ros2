"""Synchronized RGB cube faces -> per-instance detections (no truth input)."""
import copy
import time
import threading
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import json
from evtol_perception_interfaces.msg import SemanticFrame
from evtol_simulation.camera_ring import SharedMemoryCameraReader
from evtol_simulation.runtime import TimerResolution
from .vision_node import ModelClient
from .common import args_config, Health, LATEST, header


class SurroundVision(Node):
    def __init__(self,c):
        super().__init__('surround_vision');self.c=c;self.s=c['surround']
        root='/'+c['namespace']+'/perception/surround'
        self.detect=c['model'].get('task','semantic')=='detect'
        if self.detect:
            self.boxes=self.create_publisher(String,root+'/detections',LATEST)
        else:
            self.pub=self.create_publisher(SemanticFrame,root+'/semantic',LATEST)
            self.labels=self.create_publisher(CompressedImage,root+'/semantic_ids/compressed',LATEST)
        self.exposure=self.create_publisher(String,root+'/exposure',LATEST)
        self.rgb=self.create_publisher(CompressedImage,root+'/rgb/compressed',LATEST)
        self.preview=self.create_publisher(CompressedImage,root+('/detections_preview/compressed' if self.detect else '/semantic_preview/compressed'),LATEST)
        self.health=Health(self,'/'+c['namespace']+'/surround_vision')
        self.stop=threading.Event();self.client=None;self.readers=[]
        self.worker=threading.Thread(target=self.run,daemon=True);self.worker.start()

    def run(self):
        try:
            cols=len(self.s['cameras']) if self.detect else 3
            rows=1 if self.detect else 2
            c=copy.deepcopy(self.c);c['model']['imgsz']=self.s['imgsz'];c['model'].update(tile_rows=rows,tile_cols=cols)
            if self.s.get('weights'):c['model']['weights']=self.s['weights']
            self.client=ModelClient(c)
            self.get_logger().info('Surround '+self.client.info['task']+' model ready: '+str(self.client.info))
            w,h=self.s['width'],self.s['height'];deadline=time.monotonic()+30.
            for v in self.s['cameras']:
                name='Local\\AirSim_'+self.c['airsim']['vehicle']+'_'+v['name']+f'_{w}x{h}_0'
                while not self.stop.is_set():
                    try:self.readers.append(SharedMemoryCameraReader(name,w,h,8));break
                    except OSError:
                        if time.monotonic()>deadline:raise
                        self.stop.wait(.1)
            last=0;seq=0;last_preview=0.
            while not self.stop.is_set():
                begun=time.perf_counter()
                try:first=self.readers[0].read_latest()
                except RuntimeError:
                    self.health.dropped+=1;self.stop.wait(.003);continue
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
                self.exposure.publish(String(data=json.dumps(dict(stamp_ns=last))))
                mosaic=np.empty((h*rows,w*cols,3),np.uint8)
                for i,f in enumerate(frames):
                    mosaic[(i//cols)*h:(i//cols+1)*h,(i%cols)*w:(i%cols+1)*w]=np.frombuffer(f.bgr_bytes,np.uint8).reshape(h,w,3)
                seq+=1;prediction,ms=self.client.predict(mosaic,seq)
                if self.detect:
                    from .box_localization import normalize_detections
                    boxes=normalize_detections(prediction,c['model'],w,h,len(frames))
                    message=dict(schema_version=1,stamp_ns=last,sequence=seq,width=w,height=h,
                        cameras=[v['name'] for v in self.s['cameras']],detections=boxes,
                        inference_ms=float(ms),model_sha256=self.client.info['sha256'])
                    self.boxes.publish(String(data=json.dumps(message,allow_nan=False)))
                    self.health.ok();self.health.details=dict(task='detect',inference_ms=round(ms,2),
                        output_age_ms=round((self.get_clock().now().nanoseconds-last)*1e-6,2),
                        views=len(frames),backend=self.client.info.get('backend','torch'),boxes=len(boxes),drone_boxes=sum(b['class_id']==6 for b in boxes))
                    images=[]
                    if time.monotonic()-last_preview>=1/self.s.get('preview_hz',5.):
                        annotated=mosaic.copy()
                        for b in boxes:
                            i=b['view_index'];x1,y1,x2,y2=b['xyxy'];dx,dy=(i%cols)*w,(i//cols)*h
                            p=(round(x1)+dx,round(y1)+dy);q=(round(x2)+dx,round(y2)+dy)
                            cv2.rectangle(annotated,p,q,(0,255,80),2)
                            cv2.putText(annotated,'drone %.2f'%b['confidence'],(p[0],max(dy+14,p[1]-4)),
                                cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,80),1,cv2.LINE_AA)
                        images=[(self.rgb,mosaic),(self.preview,annotated)]
                else:
                    mask=prediction
                    self.publish_semantics(mask,last,seq,ms,frames)
                    palette=np.array([[0,0,0],[6,108,153],[191,105,112],[72,121,89],[64,225,190],[59,190,206],[36,13,81]],np.uint8)
                    images=[(self.rgb,mosaic),(self.preview,palette[np.clip(mask,0,6)])]
                for pub,im in images:
                    width=int(self.s.get('preview_width',1536))
                    if im.shape[1]>width:im=cv2.resize(im,(width,round(im.shape[0]*width/im.shape[1])),interpolation=cv2.INTER_AREA)
                    ok,jpg=cv2.imencode('.jpg',im,[cv2.IMWRITE_JPEG_QUALITY,75])
                    if ok:pub.publish(CompressedImage(header=header(last,self.c['namespace']+'/surround_mosaic'),format='jpeg',data=jpg.tobytes()))
                if images:last_preview=time.monotonic()
                self.stop.wait(max(0,1/self.s['max_hz']-(time.perf_counter()-begun)))
        except Exception as exc:self.health.error(exc)
        finally:
            if self.client:self.client.close()
            for reader in self.readers:reader.close()

    def publish_semantics(self,mask,last,seq,ms,frames):
        msg=SemanticFrame(header=header(last,self.c['namespace']+'/surround_mosaic'),sequence=seq,
            width=mask.shape[1],height=mask.shape[0],inference_ms=float(ms),model_sha256=self.client.info['sha256'])
        msg.class_ids.frombytes(mask.astype('<i4').tobytes())
        if self.pub.get_subscription_count():self.pub.publish(msg)
        ok,png=cv2.imencode('.png',mask.astype(np.uint8),[cv2.IMWRITE_PNG_COMPRESSION,1])
        if not ok:raise RuntimeError('Semantic ID PNG encoding failed')
        self.labels.publish(CompressedImage(header=msg.header,format='png; semantic_ids_0_6',data=png.tobytes()))
        self.health.ok()
        self.health.details=dict(inference_ms=round(ms,2),output_age_ms=round((self.get_clock().now().nanoseconds-last)*1e-6,2),
            views=len(frames),drone_pixels=int(np.count_nonzero(mask==6)))

    def close(self):
        self.stop.set()
        if self.client:self.client.process.terminate()
        self.worker.join(timeout=8.)


def main(args=None):
    c,ros=args_config(args);res=TimerResolution();cv2.setNumThreads(1);rclpy.init(args=ros);node=SurroundVision(c)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.close();node.destroy_node();rclpy.shutdown();res.close()
