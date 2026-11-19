"""Real independent-truth recall acceptance; never uses peak/maximum recall."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import threading
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import CompressedImage
from evtol_console.engine import Engine
from evtol_console.server import ConsoleBridge,ros_stop
from evtol_targeting.common import LATEST


def main():
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1])
    p.add_argument('--seconds',type=float,default=150.);p.add_argument('--label',default='surround_v1');p.add_argument('--visible',action='store_true')
    p.add_argument('--collect-directory',type=Path);p.add_argument('--route-alt',action='store_true')
    a=p.parse_args();root=a.root.resolve();cfg=root/'config/console.local.json';original_hash=hashlib.sha256(cfg.read_bytes()).hexdigest()
    origin=root/'data/session_origin.json';original_origin=origin.read_bytes() if origin.exists() else None
    out=root/'reports'/('perception_'+a.label);out.mkdir(exist_ok=True)
    if a.collect_directory:
        import evtol_console.engine as engine_module
        original_artifacts=engine_module.artifacts
        def record_artifacts(*args,**kwargs):
            value=original_artifacts(*args,**kwargs);value[1]['surround']['record_truth']=True;return value
        engine_module.artifacts=record_artifacts
    rclpy.init(domain_id=223);engine=Engine(root,ros_stop=ros_stop)
    if a.route_alt:engine.config['mission']['waypoints_enu_m']=[[0,0,8],[0,55,8],[15,65,15],[0,35,12]]
    engine.config['ue']['visible']=a.visible;engine.config['run'].update(execute=True,actors=True,evaluate=True,new_session=True)
    done=threading.Event();bridge=ConsoleBridge(engine,'drone1',0);executor=SingleThreadedExecutor();executor.add_node(bridge)
    lastsave={};samples=[];error=None;collector=None;collect_stop=threading.Event()
    def image(kind,m):
        if time.monotonic()-lastsave.get(kind,0)<10:return
        lastsave[kind]=time.monotonic();(out/(kind+'_'+str(m.header.stamp.sec)+'.jpg')).write_bytes(bytes(m.data))
    for kind,topic in [('rgb','rgb/compressed'),('semantic','semantic_preview/compressed')]:
        bridge.create_subscription(CompressedImage,'/drone1/perception/surround/'+topic,lambda m,k=kind:image(k,m),LATEST)
    def spin():
        while not done.is_set():executor.spin_once(timeout_sec=.01)
    thread=threading.Thread(target=spin,daemon=True);thread.start()
    def sample():
        t=engine.status()['telemetry'];row=dict(wall=time.time(),telemetry=t);samples.append(row)
        print(json.dumps(dict(position=t.get('position_enu_m'),flight=t.get('flight',{}).get('phase'),
            startup=t.get('startup'),diagnostics={k:v.get('values') for k,v in t.get('diagnostics',{}).items() if any(s in k for s in ('surround','localizer','perception_lidar'))}),ensure_ascii=False),flush=True)
    def collect():
        import cv2,numpy as np,yaml
        from evtol_simulation.camera_ring import SharedMemoryCameraReader
        from evtol_simulation.semantic import SemanticDecoder
        classes=yaml.safe_load((root/'src/evtol_simulation/config/source_parameters.yaml').read_text(encoding='utf8'))['semantic_segmentation']['classes']
        decode=SemanticDecoder(classes)
        target=a.collect_directory;target.mkdir(parents=True,exist_ok=True)
        for sub in ('images','masks','raw_masks'):(target/sub).mkdir(exist_ok=True)
        readers=[];last={}
        try:
            for view in engine_module.artifacts(engine.config,root)[1]['surround']['cameras']:
                prefix='Local\\AirSim_Drone1_'+view['name']+'_512x512_'
                readers.append((view['name'],SharedMemoryCameraReader(prefix+'0',512,512,8),SharedMemoryCameraReader(prefix+'5',512,512,8)))
            while not collect_stop.wait(.4):
                for name,rgb,sem in readers:
                    f=rgb.read_latest()
                    if last.get(name)==f.timestamp_ns:continue
                    sf=None;latest=sem.latest_frame_id()
                    for i in range(latest,max(0,latest-8),-1):
                        try:v=sem._try_read_frame(i)
                        except RuntimeError:continue
                        if v and v.timestamp_ns==f.timestamp_ns:sf=v;break
                    if sf is None:continue
                    im=np.frombuffer(f.bgr_bytes,np.uint8).reshape(512,512,3);raw=np.frombuffer(sf.bgr_bytes,np.uint8).reshape(512,512,3);mask=decode(raw)
                    if np.mean(mask==255)>.01:continue
                    stem=name+'_'+str(f.timestamp_ns);last[name]=f.timestamp_ns
                    cv2.imwrite(str(target/'images'/(stem+'.png')),im);cv2.imwrite(str(target/'masks'/(stem+'.png')),mask)
                    cv2.imwrite(str(target/'raw_masks'/(stem+'.png')),raw)
        except Exception as exc:print('COLLECT ERROR: '+str(exc),flush=True)
        finally:
            for _,r,g in readers:r.close();g.close()

    def operation(action,timeout):
        result=engine.request(dict(action=action));assert result['ok'],result
        identifier=result['data']['operation_id'];deadline=time.monotonic()+timeout;nextlog=0
        while time.monotonic()<deadline:
            item=next(x for x in engine.status()['operations'] if x['id']==identifier)
            if item['state']=='done':return
            if item['state'] in ('failed','cancelled'):raise RuntimeError(item['message'])
            if time.monotonic()>nextlog:sample();nextlog=time.monotonic()+5
            time.sleep(.2)
        raise TimeoutError(action)
    try:
        operation('start_ue',140);operation('start_ros',250)
        if a.collect_directory:
            collector=threading.Thread(target=collect,daemon=True);collector.start()
        deadline=time.monotonic()+a.seconds
        while time.monotonic()<deadline:sample();done.wait(5.)
    except BaseException as exc:error=str(exc);print('PROBE ERROR: '+error,flush=True)
    finally:
        collect_stop.set()
        if collector:collector.join(timeout=5.)
        run=engine.active;engine.close();done.set();thread.join(timeout=3);executor.shutdown();bridge.destroy_node();rclpy.shutdown()
        if original_origin is None:origin.unlink(missing_ok=True)
        else:origin.write_bytes(original_origin)
        report=dict(error=error,run=str(run),config_unchanged=hashlib.sha256(cfg.read_bytes()).hexdigest()==original_hash,owned_remaining=list(engine.processes))
        sums=Counter();latency=[]
        if run:
            metrics=list((run/'evaluation').glob('range_*/metrics.jsonl'))
            if metrics:
                for line in metrics[-1].open(encoding='utf8'):
                    try:m=json.loads(line)
                    except json.JSONDecodeError:continue
                    sums['epochs']+=1
                    if not m.get('valid',False):sums['invalid_epochs']+=1;continue
                    if not m.get('scoring',True):continue
                    sums['scoring_epochs']+=1
                    for k in ('truth_count','detected_count','false_positive_count','fov_truth_count','fov_detected_count'):sums[k]+=m.get(k,0)
                    v=m.get('pipeline',{}).get('last_latency_ms')
                    if isinstance(v,(int,float)):latency.append(v)
                recall=sums['detected_count']/sums['truth_count'] if sums['truth_count'] else None
                precision=sums['detected_count']/(sums['detected_count']+sums['false_positive_count']) if sums['detected_count']+sums['false_positive_count'] else None
                report.update(counts=dict(sums),recall=recall,precision=precision,
                    meets_95=recall is not None and precision is not None and recall>=.95 and precision>=.95)
        (out/'result.json').write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf8')
        (out/'telemetry.json').write_text(json.dumps(samples,ensure_ascii=False),encoding='utf8')
        print(json.dumps(report,ensure_ascii=False),flush=True)
    if error:raise RuntimeError(error)


if __name__=='__main__':main()
