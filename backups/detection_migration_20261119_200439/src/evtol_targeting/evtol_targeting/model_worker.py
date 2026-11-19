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
    p.add_argument('--parent-pid',type=int,default=0);opts=p.parse_args()
    watch_parent(opts.parent_pid)
    sys.path.insert(0,opts.vendor)
    with contextlib.redirect_stdout(sys.stderr):
        import numpy as np
        import cv2
        import torch
        from ultralytics import YOLO
        torch.set_num_threads(1);cv2.setNumThreads(1)
        model=YOLO(opts.weights)
        if model.task!='semantic':raise ValueError('The supplied best.pt must be the project semantic model')
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
                results=model.predict(images,imgsz=opts.imgsz,device=device,half=half,verbose=False)
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


if __name__=='__main__':main()
