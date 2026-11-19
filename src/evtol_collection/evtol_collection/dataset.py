"""Exact-stamp RGB/truth pairing, YOLO connected-region labels and lossless masks."""
from collections import OrderedDict
import json
import os
from pathlib import Path
import time
import cv2
import numpy as np
import yaml
from .options import CLASSES

def atomic_json(path,value):
    path=Path(path);temporary=path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf8')
    os.replace(temporary,path)

def timestamp(msg):return msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec

def image_array(msg,mask=False):
    channels=1 if mask else 3
    if msg.is_bigendian or msg.encoding not in (('mono8','8UC1') if mask else ('bgr8','rgb8')):
        raise ValueError('Expected mono8 truth or BGR/RGB image')
    if msg.width<=0 or msg.height<=0 or msg.step<msg.width*channels or len(msg.data)<msg.height*msg.step:
        raise ValueError('Invalid image buffer/stride')
    shape=(msg.height,msg.width) if mask else (msg.height,msg.width,3)
    strides=(msg.step,1) if mask else (msg.step,3,1)
    array=np.ndarray(shape,np.uint8,buffer=msg.data,strides=strides)
    if not mask and msg.encoding=='rgb8':array=array[:,:,::-1]
    return np.ascontiguousarray(array)

class ExactPairs:
    def __init__(self,capacity):
        self.capacity=capacity;self.pending=[OrderedDict(),OrderedDict()];self.unmatched=0;self.last=[0,0]
    def add(self,kind,msg):
        t=timestamp(msg)
        if t<=0:raise ValueError('Missing acquisition timestamp')
        if t<self.last[kind]:raise ValueError('Acquisition clock reset; start a new collection session')
        if t==self.last[kind]:return None
        self.last[kind]=t;other=self.pending[1-kind]
        if t in other:
            paired=other.pop(t)
            return (msg,paired) if kind==0 else (paired,msg)
        self.pending[kind][t]=msg
        while len(self.pending[kind])>self.capacity:
            self.pending[kind].popitem(last=False);self.unmatched+=1
        return None
    def finish(self):
        self.unmatched+=sum(map(len,self.pending))
        for queue in self.pending:queue.clear()

def labels_from_mask(mask,options):
    """One visible connected component, NOT one simulator instance.

    Holes are retained exactly in semantic PNGs. YOLO polygons use external
    contours and can fill holes; record that limitation instead of inventing IDs.
    """
    if mask.dtype!=np.uint8 or mask.ndim!=2:raise ValueError('Expected uint8 class mask')
    unknown=set(np.unique(mask).tolist())-set(CLASSES)-{255}
    if unknown:raise ValueError('Unknown truth class IDs: '+str(sorted(unknown)))
    h,w=mask.shape;detection=[];segmentation=[];objects=[];small=degenerate=holes=0
    for yolo_id,source_id in enumerate(options['class_ids']):
        count,components,stats,_=cv2.connectedComponentsWithStats((mask==source_id).astype(np.uint8),8)
        for label in range(1,count):
            x,y,bw,bh,area=map(int,stats[label])
            if area<options['min_area_px']:small+=1;continue
            detection.append(f'{yolo_id} {(x+bw/2)/w:.8f} {(y+bh/2)/h:.8f} {bw/w:.8f} {bh/h:.8f}')
            component=(components[y:y+bh,x:x+bw]==label).astype(np.uint8)
            contours,hierarchy=cv2.findContours(component,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
            object_holes=0
            if hierarchy is not None:
                object_holes=int(np.count_nonzero(hierarchy[0,:,3]>=0));holes+=object_holes
            contour=max((c for i,c in enumerate(contours) if hierarchy[0,i,3]<0),key=cv2.contourArea)
            polygon=cv2.approxPolyDP(contour,float(options['polygon_epsilon_px']),True).reshape(-1,2)
            if len(polygon)<3 or cv2.contourArea(polygon.astype(np.float32))<=0:
                degenerate+=1;polygon=None
            else:
                polygon=(polygon+[x,y])/np.array([w,h]);polygon=np.clip(polygon,0,1)
                segmentation.append(str(yolo_id)+' '+' '.join(f'{v:.8f}' for v in polygon.ravel()))
            objects.append(dict(source_class_id=source_id,yolo_class_id=yolo_id,bbox_xywh=[x,y,bw,bh],
                visible_pixels=area,holes=object_holes,polygon_exported=polygon is not None))
    return detection,segmentation,dict(objects=objects,small_regions_skipped=small,
        degenerate_polygons_skipped=degenerate,regions_holes=holes,unknown_pixels=int(np.count_nonzero(mask==255)))

class DatasetWriter:
    def __init__(self,folder,options,session):
        self.folder=Path(folder);self.o=options;self.session=session;self.saved=0;self.objects=0;self.empty=0
        self.first=None;self.last=None;self.created=[]
        self.index=(self.folder/'frames.jsonl').open('x',encoding='utf8')
        self.split=options['split']
        for task in ['semantic']+options['labels']:
            for split in ('train','val','test'):
                for kind in (('images','masks') if task=='semantic' else ('images','labels')):
                    (self.folder/task/kind/split).mkdir(parents=True,exist_ok=True)
            config=dict(train='images/train',val='images/val',test='images/test',
                names=CLASSES if task=='semantic' else dict(enumerate(CLASSES[i] for i in options['class_ids'])))
            if task=='semantic':config.update(mask_encoding='original_class_id_uint8_png',ignore_index=255)
            (self.folder/task/'dataset.yaml').write_text(yaml.safe_dump(config,allow_unicode=True,sort_keys=False),encoding='utf8')
        atomic_json(self.folder/'label_schema.json',dict(version=1,source='UE palette-decoded class truth, not model predictions',
            instance_labels=False,association='8-connected same-class visible regions; touching/overlapping instances may merge',
            semantic_class_ids=CLASSES,ignore_index=255,yolo_to_source=dict(enumerate(options['class_ids'])),
            polygons='external contour only; holes and fine detail remain exact in original semantic PNG',
            split_policy='one split for the entire run; collect independent val/test runs to prevent temporal leakage',options=options))
    def write(self,rgb,truth):
        t=timestamp(rgb)
        if t!=timestamp(truth) or rgb.header.frame_id!=truth.header.frame_id:raise ValueError('RGB/truth exact stamp/frame mismatch')
        if self.last is not None and t<=self.last:raise ValueError('Repeated/regressed dataset frame')
        image=image_array(rgb);mask=image_array(truth,True)
        if image.shape[:2]!=mask.shape:raise ValueError('RGB/truth dimensions mismatch')
        boxes,polygons,details=labels_from_mask(mask,self.o)
        ok,jpeg=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,self.o['jpeg_quality']])
        if not ok:raise OSError('JPEG encoding failed')
        ok,png=cv2.imencode('.png',mask)
        if not ok:raise OSError('Mask PNG encoding failed')
        stem=f'{self.session}_{t}';written=[]
        try:
            for task in ['semantic']+self.o['labels']:
                photo=self.folder/task/'images'/self.split/(stem+'.jpg');written.append(photo);photo.write_bytes(jpeg.tobytes())
                if task=='semantic':
                    label=self.folder/task/'masks'/self.split/(stem+'.png');written.append(label);label.write_bytes(png.tobytes())
                else:
                    label=self.folder/task/'labels'/self.split/(stem+'.txt')
                    written.append(label)
                    lines=boxes if task=='detection' else polygons
                    label.write_text('\n'.join(lines)+('\n' if lines else ''),encoding='utf8')
            self.index.write(json.dumps(dict(stem=stem,stamp_ns=t,frame_id=rgb.header.frame_id,width=rgb.width,height=rgb.height,
                split=self.split,label_source='semantic_truth_connected_regions',**details),ensure_ascii=False)+'\n');self.index.flush()
        except BaseException:
            for path in written:path.unlink(missing_ok=True)
            raise
        self.saved+=1;self.objects+=len(boxes);self.empty+=int(not boxes)
        if self.first is None:self.first=t
        self.last=t
    def close(self):self.index.close()
