"""Exact integer IoU>=.5 matching on held-out images through deployed ONNX code."""
from pathlib import Path
import argparse,sys,json,hashlib,time
import numpy as np
import cv2

def main():
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--weights',type=Path,required=True);p.add_argument('--data',type=Path,required=True);p.add_argument('--confidence',type=float,required=True);a=p.parse_args()
    root=a.root.resolve();data=a.data.resolve();sys.path[:0]=[str(root/'models/onnx_runtime'),str(root/'src/evtol_targeting')]
    from evtol_targeting.onnx_detector import OnnxDetector
    detector=OnnxDetector(a.weights,1536,2,'0',True)
    files=sorted((data/'images/test').glob('*.jpg'));counts=dict(tp=0,fp=0,fn=0,images=len(files));buckets={k:dict(tp=0,fn=0) for k in ('<16','16-32','32-64','>=64')};elapsed=[];examples=[]
    out=data/'training/onnx_test';out.mkdir(parents=True,exist_ok=True)
    for start in range(0,len(files),2):
        batch=files[start:start+2];images=[cv2.imread(str(f)) for f in batch]
        if len(images)==1:images.append(images[0])
        predictions,timing=detector.predict(images,a.confidence,128);elapsed.append(timing)
        for i,f in enumerate(batch):
            boxes=np.asarray([b['xyxy'] for b in predictions if b['view_index']==i],float).reshape(-1,4)
            labels=np.asarray([list(map(float,line.split()))[1:] for line in (data/'labels/test'/(f.stem+'.txt')).read_text().splitlines()],float).reshape(-1,4)*1536
            gt=np.column_stack((labels[:,:2]-labels[:,2:]/2,labels[:,:2]+labels[:,2:]/2))
            matched=set();predmatched=set()
            if len(boxes) and len(gt):
                left=np.maximum(gt[:,None,:2],boxes[None,:,:2]);right=np.minimum(gt[:,None,2:],boxes[None,:,2:]);area=np.prod(np.maximum(0,right-left),axis=2)
                iou=area/np.maximum(np.prod(gt[:,2:]-gt[:,:2],axis=1)[:,None]+np.prod(boxes[:,2:]-boxes[:,:2],axis=1)[None,:]-area,1e-12)
                assignments={}
                def augment(row,seen):
                    for col in np.argsort(-iou[row]):
                        col=int(col)
                        if iou[row,col]<.5 or col in seen:continue
                        seen.add(col)
                        if col not in assignments or augment(assignments[col],seen):
                            assignments[col]=row;return True
                    return False
                for row in np.argsort(-iou.max(axis=1)):augment(int(row),set())
                matched=set(assignments.values());predmatched=set(assignments)
            counts['tp']+=len(matched);counts['fn']+=len(gt)-len(matched);counts['fp']+=len(boxes)-len(predmatched)
            for j,label in enumerate(labels):
                size=min(label[2:]);bucket='<16' if size<16 else '16-32' if size<32 else '32-64' if size<64 else '>=64'
                buckets[bucket]['tp' if j in matched else 'fn']+=1
            if len(examples)<12 and (len(matched)<len(gt) or len(predmatched)<len(boxes)):
                preview=images[i].copy()
                for j,box in enumerate(gt):cv2.rectangle(preview,tuple(box[:2].astype(int)),tuple(box[2:].astype(int)),(0,255,0) if j in matched else (0,0,255),2)
                for j,box in enumerate(boxes):
                    if j not in predmatched:cv2.rectangle(preview,tuple(box[:2].astype(int)),tuple(box[2:].astype(int)),(255,0,255),2)
                cv2.imwrite(str(out/(f.stem+'.jpg')),preview);examples.append(f.name)
    counts.update(recall=counts['tp']/max(1,counts['tp']+counts['fn']),precision=counts['tp']/max(1,counts['tp']+counts['fp']))
    report=dict(counts=counts,box_size=buckets,confidence=a.confidence,model=str(a.weights),sha256=hashlib.sha256(a.weights.read_bytes()).hexdigest(),
        inference_batch_ms_median=float(np.median([v['inference_ms'] for v in elapsed])),providers=detector.providers,
        note='Independent-session image detections at IoU .5; not full-hemisphere online warning success. Red=missed GT; magenta=false positive.')
    (out/'report.json').write_text(json.dumps(report,indent=2),encoding='utf8');print(json.dumps(report,indent=2),flush=True)

if __name__=='__main__':main()
