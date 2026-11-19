"""Load the shipped semantic runtime and exercise the cross-Python SHM protocol."""
import json
import numpy as np
from evtol_targeting.common import args_config,project_path
from evtol_targeting.vision_node import ModelClient

c,_=args_config()
client=ModelClient(c)
try:
    timings=[]
    for i in range(5):
        mask,ms=client.predict(np.zeros((480,640,3),dtype=np.uint8),i+1)
        assert mask.shape==(480,640) and mask.dtype==np.int32
        assert np.all((mask>=0)&(mask<=6))
        timings.append(ms)
    report=dict(model=client.info,shape=list(mask.shape),classes=np.unique(mask).tolist(),inference_ms=timings,
        input='zero_image_protocol_smoke_not_accuracy_test')
    path=project_path(c,'reports/model_smoke.json');path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(report,indent=2),encoding='utf8');print(json.dumps(report,indent=2))
finally:client.close()
