"""Runs only in the model environment. Small JSON control, shared pixel buffers."""
import argparse
import contextlib
import hashlib
import json
from multiprocessing import shared_memory
from pathlib import Path
import sys
import time
import os
import threading


def watch_parent(pid):
    # ROS launch terminates Windows children forcibly. Do not leave a CUDA
    # worker alive after its owning vision node has exited.
    if os.name!='nt' or not pid:return
    import ctypes
    kernel=ctypes.WinDLL('kernel32',use_last_error=True)
    kernel.OpenProcess.argtypes=[ctypes.c_ulong,ctypes.c_bool,ctypes.c_ulong]
    kernel.OpenProcess.restype=ctypes.c_void_p
    kernel.WaitForSingleObject.argtypes=[ctypes.c_void_p,ctypes.c_ulong]
    kernel.CloseHandle.argtypes=[ctypes.c_void_p]
    handle=kernel.OpenProcess(0x00100000,False,pid)
    if not handle:raise RuntimeError('Cannot monitor model owner process')
    def wait():
        try:
            result=kernel.WaitForSingleObject(handle,0xFFFFFFFF)
            if result==0:os._exit(0)
        finally:kernel.CloseHandle(handle)
    threading.Thread(target=wait,daemon=True).start()


def main():
    p=argparse.ArgumentParser();p.add_argument('--weights',required=True);p.add_argument('--vendor',required=True)
    p.add_argument('--imgsz',type=int,default=640);p.add_argument('--device',default='0')
    p.add_argument('--tile-rows',type=int,default=1);p.add_argument('--tile-cols',type=int,default=1)
    p.add_argument('--task',choices=['detect','semantic'],default='semantic')
    p.add_argument('--confidence',type=float,default=.25);p.add_argument('--iou',type=float,default=.5)
    p.add_argument('--max-det',type=int,default=128)
    p.add_argument('--backend',choices=['torch','onnx'],default='torch')
    p.add_argument('--require-gpu',action='store_true')
    p.add_argument('--parent-pid',type=int,default=0);opts=p.parse_args()
    watch_parent(opts.parent_pid)
    sys.path.insert(0,opts.vendor)
    if opts.backend=='onnx':
        return onnx_loop(opts)
    with contextlib.redirect_stdout(sys.stderr):
        import numpy as np
        import cv2
        import torch
        from ultralytics import YOLO
        torch.set_num_threads(1);cv2.setNumThreads(1)
        model=YOLO(opts.weights)
        if model.task!=opts.task:raise ValueError('Model task mismatch: expected '+opts.task+', got '+model.task)
        device=opts.device
        if device!='cpu' and not torch.cuda.is_available():
            print('CUDA unavailable: using CPU; real-time performance must be revalidated',file=sys.stderr);device='cpu'
        half=device!='cpu'
        model.predict(np.zeros((opts.imgsz,opts.imgsz,3),dtype=np.uint8),imgsz=opts.imgsz,device=device,half=half,verbose=False)
    print(json.dumps(dict(ready=True,task=model.task,names=model.names,device=device,precision='float16' if half else 'float32',
        sha256=hashlib.sha256(Path(opts.weights).read_bytes()).hexdigest())),flush=True)
    for line in sys.stdin:
        request=json.loads(line);mapping=None
        try:
            h,w=request['height'],request['width'];mapping=shared_memory.SharedMemory(name=request['shm'])
            image=np.ndarray((h,w,3),dtype=np.uint8,buffer=mapping.buf)
            begun=time.perf_counter()
            with contextlib.redirect_stdout(sys.stderr):
                rows,cols=opts.tile_rows,opts.tile_cols
                if h%rows or w%cols:raise ValueError('Mosaic must contain equal complete camera faces')
                th,tw=h//rows,w//cols
                images=[np.ascontiguousarray(image[y*th:(y+1)*th,x*tw:(x+1)*tw]) for y in range(rows) for x in range(cols)]
                results=model.predict(images,imgsz=opts.imgsz,device=device,half=half,verbose=False,
                    conf=opts.confidence,iou=opts.iou,max_det=opts.max_det)
                if opts.task=='detect':
                    detections=[]
                    for i,result in enumerate(results):
                        boxes=result.boxes
                        if boxes is None:continue
                        for xyxy,conf,cls in zip(boxes.xyxy.detach().cpu().tolist(),
                                boxes.conf.detach().cpu().tolist(),boxes.cls.detach().cpu().tolist()):
                            detections.append(dict(view_index=i,source_class_id=int(cls),confidence=float(conf),xyxy=xyxy))
                    print(json.dumps(dict(sequence=request['sequence'],detections=detections,
                        ms=(time.perf_counter()-begun)*1000)),flush=True,file=sys.__stdout__)
                    del image
                    continue
                mask=np.empty((h,w),np.int32)
                for i,result in enumerate(results):
                    semantic=getattr(result,'semantic_mask',None)
                    if semantic is None:raise RuntimeError('semantic_mask missing; incompatible custom Ultralytics runtime')
                    face=semantic.data.detach().cpu().numpy().astype(np.int32)
                    if face.shape!=(th,tw):face=cv2.resize(face.astype(np.float32),(tw,th),interpolation=cv2.INTER_NEAREST).astype(np.int32)
                    mask[(i//cols)*th:(i//cols+1)*th,(i%cols)*tw:(i%cols+1)*tw]=face
            target=np.ndarray((h,w),dtype=np.int32,buffer=mapping.buf,offset=h*w*3);target[:]=mask
            del image,target
            print(json.dumps(dict(sequence=request['sequence'],ms=(time.perf_counter()-begun)*1000)),flush=True)
        except Exception as exc:print(json.dumps(dict(sequence=request.get('sequence'),error=str(exc))),flush=True)
        finally:
            if mapping is not None:mapping.close()


def onnx_loop(opts):
    import numpy as np
    from onnx_detector import OnnxDetector
    if opts.task!='detect':raise ValueError('ONNX worker supports detection only')
    with contextlib.redirect_stdout(sys.stderr):
        detector=OnnxDetector(opts.weights,opts.imgsz,opts.tile_rows*opts.tile_cols,opts.device,opts.require_gpu)
    print(json.dumps(dict(ready=True,task='detect',backend='onnx',names=detector.names,device=opts.device,
        providers=detector.providers,precision=str(detector.dtype),
        sha256=hashlib.sha256(Path(opts.weights).read_bytes()).hexdigest())),flush=True)
    for line in sys.stdin:
        mapping=None;image=None;request={}
        try:
            request=json.loads(line);h,w=request['height'],request['width']
            if h%opts.tile_rows or w%opts.tile_cols:raise ValueError('Incomplete camera mosaic')
            mapping=shared_memory.SharedMemory(name=request['shm'])
            image=np.ndarray((h,w,3),np.uint8,buffer=mapping.buf)
            th,tw=h//opts.tile_rows,w//opts.tile_cols
            images=[image[y*th:(y+1)*th,x*tw:(x+1)*tw] for y in range(opts.tile_rows) for x in range(opts.tile_cols)]
            detections,timing=detector.predict(images,opts.confidence,opts.max_det)
            print(json.dumps(dict(sequence=request['sequence'],detections=detections,ms=timing['inference_ms'],**timing)),flush=True)
        except Exception as exc:print(json.dumps(dict(sequence=request.get('sequence'),error=str(exc))),flush=True)
        finally:
            image=None;images=None
            if mapping is not None:mapping.close()


if __name__=='__main__':main()
