"""Export a fixed two-camera FP16 YOLO26 ONNX model, using this project's paths."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1])
    p.add_argument('--weights',type=Path,default=Path('models/drone_detection.pt'))
    p.add_argument('--output',type=Path,default=Path('models/drone_detection_b2_1536.onnx'))
    p.add_argument('--size',type=int,default=1536)
    p.add_argument('--device',default='0')
    a=p.parse_args();root=a.root.resolve()
    source=a.weights if a.weights.is_absolute() else root/a.weights
    target=a.output if a.output.is_absolute() else root/a.output
    if a.size<=0 or a.size%32:p.error('--size must be a positive multiple of 32')
    if not source.is_file():p.error('Source checkpoint missing: '+str(source))
    if target.suffix!='.onnx':p.error('--output must end with .onnx')
    sys.path[:0]=[str(root/'models/onnx_runtime'),str(root/'models/semantic_runtime')]
    os.environ['YOLO_AUTOINSTALL']='false'
    from ultralytics import YOLO
    target.parent.mkdir(parents=True,exist_ok=True)
    # Export beside a temporary checkpoint so the source remains untouched.
    with tempfile.TemporaryDirectory(prefix='evtol_onnx_') as directory:
        checkpoint=Path(directory)/target.with_suffix('.pt').name
        shutil.copyfile(source,checkpoint)
        model=YOLO(str(checkpoint))
        if model.task!='detect':raise ValueError('A detection checkpoint is required')
        exported=Path(model.export(format='onnx',imgsz=a.size,batch=2,half=True,dynamic=False,
            device=a.device,opset=17,simplify=False,nms=False))
        shutil.copyfile(exported,target)
        names={str(k):v for k,v in model.names.items()}
    manifest=dict(task='detect',backend='onnxruntime_cuda',batch=2,imgsz=a.size,precision='float16',opset=17,
        weights=target.name,sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
        source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),class_map={'0':6},names=names)
    target.with_suffix('.json').write_text(json.dumps(manifest,indent=2),encoding='utf8')
    print(json.dumps(manifest,indent=2))


if __name__=='__main__':main()
