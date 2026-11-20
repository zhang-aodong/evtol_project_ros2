"""Fixed-batch YOLO end-to-end ONNX inference with explicit GPU enforcement."""
import ast
import time
import cv2
import numpy as np


def letterbox(image, size):
    h,w=image.shape[:2];scale=min(size/w,size/h)
    if h==size and w==size:return image[:,:,::-1].transpose(2,0,1),(1.,0,0,w,h)
    nw,nh=round(w*scale),round(h*scale)
    left,top=(size-nw)//2,(size-nh)//2
    resized=cv2.resize(image,(nw,nh),interpolation=cv2.INTER_LINEAR)
    canvas=np.full((size,size,3),114,np.uint8);canvas[top:top+nh,left:left+nw]=resized
    return canvas[:,:,::-1].transpose(2,0,1), (scale,left,top,w,h)


def decode(output, transforms, confidence, max_det):
    a=np.asarray(output)
    if a.ndim!=3 or a.shape[0]!=len(transforms) or a.shape[2]!=6:
        raise ValueError('Expected YOLO end-to-end ONNX output [batch, candidates, 6]')
    boxes=[]
    for i,(rows,transform) in enumerate(zip(a,transforms)):
        scale,left,top,w,h=transform
        rows=rows[np.isfinite(rows).all(axis=1)&(rows[:,4]>=confidence)]
        rows=rows[np.argsort(-rows[:,4])[:max_det]]
        for x1,y1,x2,y2,score,cls in rows.astype(float):
            if cls<0 or abs(cls-round(cls))>1e-4:continue
            xyxy=[max(0.,min(w,(x1-left)/scale)),max(0.,min(h,(y1-top)/scale)),
                  max(0.,min(w,(x2-left)/scale)),max(0.,min(h,(y2-top)/scale))]
            if xyxy[2]-xyxy[0]<1 or xyxy[3]-xyxy[1]<1:continue
            boxes.append(dict(view_index=i,source_class_id=int(cls),confidence=float(score),xyxy=xyxy))
    return boxes


class OnnxDetector:
    def __init__(self, weights, imgsz, batch, device='0', require_gpu=True):
        cv2.setNumThreads(1)
        # Importing torch preloads the CUDA/cuDNN DLLs from the existing GPU
        # environment. It never loads a .pt model or performs model inference.
        if device!='cpu':
            import torch
            torch.set_num_threads(1)
        import onnxruntime as ort
        if device!='cpu' and hasattr(ort,'preload_dlls'):ort.preload_dlls()
        if device!='cpu' and 'CUDAExecutionProvider' not in ort.get_available_providers():
            raise RuntimeError('ONNX Runtime CUDA provider unavailable; install onnxruntime-gpu')
        options=ort.SessionOptions();options.intra_op_num_threads=1;options.inter_op_num_threads=1
        options.graph_optimization_level=ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        providers=['CPUExecutionProvider'] if device=='cpu' else [('CUDAExecutionProvider',
            {'device_id':int(device),'cudnn_conv_algo_search':'HEURISTIC','do_copy_in_default_stream':True})]
        self.session=ort.InferenceSession(str(weights),sess_options=options,providers=providers)
        self.session.disable_fallback();self.providers=self.session.get_providers()
        if require_gpu and self.providers[0]!='CUDAExecutionProvider':
            raise RuntimeError('ONNX GPU is required; refusing CPU fallback')
        inp=self.session.get_inputs()[0]
        if inp.shape!=[batch,3,imgsz,imgsz]:raise ValueError('ONNX input shape does not match configured camera batch/imgsz: '+str(inp.shape))
        self.dtype=np.float16 if inp.type=='tensor(float16)' else np.float32
        if inp.type not in ('tensor(float16)','tensor(float)'):raise ValueError('Unsupported ONNX input dtype '+inp.type)
        metadata=self.session.get_modelmeta().custom_metadata_map
        if metadata.get('task')!='detect':raise ValueError('ONNX model must declare task=detect')
        self.names=ast.literal_eval(metadata['names']);self.input=inp.name;self.size=imgsz;self.batch=batch
        self.gpu=device!='cpu';self.device=int(device) if self.gpu else None
        self.output=self.session.get_outputs()[0].name
        self.session.run(None,{self.input:np.zeros((batch,3,imgsz,imgsz),self.dtype)})

    def predict(self,images,confidence=.25,max_det=128):
        if len(images)!=self.batch:raise ValueError('Camera count differs from ONNX fixed batch')
        begun=time.perf_counter();items=[letterbox(im,self.size) for im in images]
        if self.gpu:
            # NumPy float16 arithmetic is slow on Windows CPUs. Normalize on
            # CUDA and bind the device buffer directly to ONNX Runtime.
            # Torch is used only for pixel preparation, never model inference.
            import torch
            dtype=torch.float16 if self.dtype==np.float16 else torch.float32
            pixels=np.stack([x[0].transpose(1,2,0)[:,:,::-1] for x in items])
            tensor=torch.from_numpy(pixels).to('cuda:'+str(self.device))
            tensor=tensor.permute(0,3,1,2).flip(1).contiguous().to(dtype=dtype).mul_(1./255.)
            torch.cuda.synchronize(self.device)
            binding=self.session.io_binding()
            binding.bind_input(self.input,'cuda',self.device,self.dtype,tuple(tensor.shape),tensor.data_ptr())
            binding.bind_output(self.output,'cuda',self.device)
            prepared=time.perf_counter();self.session.run_with_iobinding(binding)
            output=binding.copy_outputs_to_cpu()[0]
        else:
            pixels=np.ascontiguousarray(np.stack([x[0] for x in items]),dtype=np.uint8)
            tensor=pixels.astype(np.float32)*(1./255.)
            tensor=np.ascontiguousarray(tensor,dtype=self.dtype)
            prepared=time.perf_counter();output=self.session.run(None,{self.input:tensor})[0]
        inferred=time.perf_counter();boxes=decode(output,[x[1] for x in items],confidence,max_det)
        return boxes,dict(preprocess_ms=(prepared-begun)*1000,onnx_ms=(inferred-prepared)*1000,
                          inference_ms=(time.perf_counter()-begun)*1000)
