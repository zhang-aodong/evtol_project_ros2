"""Fine tune YOLO26 detection; choose threshold on validation, test once."""
from pathlib import Path
import argparse,json,os,sys,hashlib

def main():
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--data',type=Path,required=True);p.add_argument('--epochs',type=int,default=24);a=p.parse_args()
    root=a.root.resolve();data=a.data.resolve();output=data/'training'
    os.environ.update(YOLO_AUTOINSTALL='false',YOLO_CONFIG_DIR=str(output/'yolo_config'),MPLCONFIGDIR=str(output/'mpl'))
    sys.path.insert(0,str(root/'models/semantic_runtime'))
    import torch,cv2
    from ultralytics import YOLO
    torch.set_num_threads(4);cv2.setNumThreads(1)
    checkpoint=root/'models/drone_detection.pt'
    settings=dict(data=str(data/'dataset.yaml'),epochs=a.epochs,patience=8,imgsz=1536,batch=12,device=0,workers=4,
        seed=20261120,deterministic=True,project=str(output),name='yolo26n_diverse',exist_ok=False,
        optimizer='AdamW',lr0=.0005,cos_lr=True,warmup_epochs=1.,close_mosaic=5,mosaic=.35,
        scale=.25,translate=.12,degrees=10.,fliplr=.5,flipud=0.,cache=False,amp=True,plots=True)
    output.mkdir(parents=True,exist_ok=True)
    (output/'settings.json').write_text(json.dumps(dict(settings=settings,initial=str(checkpoint),sha256=hashlib.sha256(checkpoint.read_bytes()).hexdigest(),torch=torch.__version__,gpu=torch.cuda.get_device_name(0)),indent=2),encoding='utf8')
    model=YOLO(checkpoint);assert model.task=='detect';model.train(**settings)
    best=Path(model.trainer.best);trained=YOLO(best)
    val=trained.val(data=str(data/'dataset.yaml'),split='val',imgsz=1536,batch=8,device=0,workers=2,conf=.001,plots=True,project=str(output/'evaluation'),name='validation')
    # Threshold is selected on independent validation images, never on test/live truth.
    import numpy as np
    metrics=val.box
    curve=np.asarray(metrics.r_curve);pc=np.asarray(metrics.p_curve);xs=np.asarray(metrics.px)
    choices=[]
    for threshold in (.05,.1,.15,.2,.25,.3,.4):
        j=int(np.argmin(abs(xs-threshold)));rec=float(curve[0,j]);prec=float(pc[0,j]);choices.append(dict(conf=threshold,recall=rec,precision=prec,f1=2*rec*prec/max(rec+prec,1e-9)))
    eligible=[c for c in choices if c['precision']>=.90 and c['recall']>=.90]
    chosen=max(eligible or choices,key=lambda c:c['f1'])
    test=trained.val(data=str(data/'dataset.yaml'),split='test',imgsz=1536,batch=8,device=0,workers=2,conf=.001,plots=True,project=str(output/'evaluation'),name='heldout_test')
    j=int(np.argmin(abs(np.asarray(test.box.px)-chosen['conf'])))
    report=dict(best=str(best),validation=val.results_dict,threshold_candidates=choices,selected=chosen,test=test.results_dict,
        test_at_selected_threshold=dict(conf=chosen['conf'],recall=float(test.box.r_curve[0,j]),precision=float(test.box.p_curve[0,j])),
        test_speed_ms=test.speed,note='Image semantic-region detection metrics; not online full-hemisphere warning rate.')
    (output/'report.json').write_text(json.dumps(report,indent=2,default=str),encoding='utf8');print(json.dumps(report,indent=2,default=str),flush=True)

if __name__=='__main__':main()
