"""Check CUDA inference and fixed model shape without starting UE or ROS."""
import argparse
import json
from pathlib import Path
import sys


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1])
    p.add_argument('--weights',type=Path,default=Path('models/drone_detection_diverse_20261120_b3_1536.onnx'))
    p.add_argument('--batch',type=int,choices=(2,3),default=3,help='Must equal the configured camera count')
    p.add_argument('--size',type=int,default=1536)
    p.add_argument('--device',default='0')
    a=p.parse_args();root=a.root.resolve()
    sys.path[:0]=[str(root/'models/onnx_runtime'),str(root/'src/evtol_targeting')]
    import numpy as np
    from evtol_targeting.onnx_detector import OnnxDetector
    weights=a.weights if a.weights.is_absolute() else root/a.weights
    detector=OnnxDetector(weights,a.size,a.batch,a.device,require_gpu=True)
    _,timing=detector.predict([np.zeros((a.size,a.size,3),np.uint8) for _ in range(a.batch)])
    print(json.dumps(dict(providers=detector.providers,shape=[a.batch,3,a.size,a.size],
        precision=str(detector.dtype),timing=timing,note='Synthetic input checks execution, not detection accuracy.'),indent=2))


if __name__=='__main__':main()
