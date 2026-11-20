"""Real independent-truth recall acceptance; never uses peak/maximum recall."""
import argparse
from collections import Counter
import hashlib
import json
import os
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
    p.add_argument('--collect-directory',type=Path);p.add_argument('--route-alt',action='store_true');p.add_argument('--radial',action='store_true')
    p.add_argument('--radial-holdout',action='store_true')
    p.add_argument('--radial-verification',action='store_true')
    p.add_argument('--dual-front',action='store_true',help='32 aircraft, two altitude layers, 10 m forward hemisphere warning audit')
    a=p.parse_args();root=a.root.resolve();cfg=root/'config/console.local.json';original_hash=hashlib.sha256(cfg.read_bytes()).hexdigest()
    origin=root/'data/session_origin.json';original_origin=origin.read_bytes() if origin.exists() else None
    out=root/'reports'/('perception_'+a.label);out.mkdir(exist_ok=True)
    if a.collect_directory:
        import evtol_console.engine as engine_module
        original_artifacts=engine_module.artifacts
        def record_artifacts(*args,**kwargs):
            value=original_artifacts(*args,**kwargs);value[1]['surround']['record_truth']=True;return value
        engine_module.artifacts=record_artifacts
    # Child processes must share the private domain even if the invoking
    # terminal uses the user's normal domain. Avoid a silent readiness timeout.
    os.environ['ROS_DOMAIN_ID']='223'
    rclpy.init(domain_id=223);engine=Engine(root,ros_stop=ros_stop)
    if a.route_alt:engine.config['mission']['waypoints_enu_m']=[[0,0,8],[0,55,8],[15,65,15],[0,35,12]]
    if a.radial or a.radial_holdout or a.radial_verification or a.dual_front:
        import math
        holdout=a.radial_holdout;outer,inner=(28.,9.) if holdout else (26.,8.)
        if a.radial_verification:outer,inner=30.,10.
        engine.config['mission'].update(waypoints_enu_m=[[0,0,12 if holdout else 10]],land_at_last=False)
        if a.radial_verification:engine.config['mission']['waypoints_enu_m']=[[0,0,12.5]]
        agents=[]
        for i in range(32):
            angle=2*math.pi*i/32+(math.radians(7) if holdout else 0);east,north=math.sin(angle),math.cos(angle);height=(8. if holdout else 7.)+3*(i%4)
            speed=1.7+.1*(i%8) if holdout else 2.
            if a.radial_verification:
                angle=2*math.pi*i/32+math.radians(19);east,north=math.sin(angle),math.cos(angle)
                height=9.+3*(i%4);speed=1.8+.07*(i%8)
            if a.dual_front:
                outer,inner=20.,7.
                angle=2*math.pi*(i%16)/16+math.radians(11)
                east,north=math.sin(angle),math.cos(angle);height=8. if i<16 else 12.;speed=1.5+.05*(i%4)
            agents.append(dict(name='Drone'+str(i+2),speed_mps=speed,spawn_ned_m=[outer*north,outer*east,0.],
                waypoints_enu_m=[[outer*east,outer*north,height],[inner*east,inner*north,height]],repeat=0))
        engine.config['actors'].update(count=32,agents=agents,initialization='first_waypoint')
        if a.dual_front:
            engine.config['mission'].update(waypoints_enu_m=[[0,0,10]],land_at_last=False)
            engine.config['experiment'].update(region='front_hemisphere',radius_m=10.)
            engine.config['warnings']['fusion_radius_m']=10.
    engine.config['ue']['visible']=a.visible;engine.config['run'].update(execute=True,actors=True,evaluate=True,new_session=True)
    done=threading.Event();bridge=ConsoleBridge(engine,'drone1',0);executor=SingleThreadedExecutor();executor.add_node(bridge)
    lastsave={};samples=[];error=None;collector=None;collect_stop=threading.Event()
    def image(kind,m):
        if time.monotonic()-lastsave.get(kind,0)<10:return
        lastsave[kind]=time.monotonic();(out/(kind+'_'+str(m.header.stamp.sec)+'.jpg')).write_bytes(bytes(m.data))
    for kind,topic in [('rgb','rgb/compressed'),('detections','detections_preview/compressed')]:
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
            settings=engine_module.artifacts(engine.config,root)[1]['surround'];w,h=settings['width'],settings['height']
            for view in settings['cameras']:
                prefix='Local\\AirSim_Drone1_'+view['name']+f'_{w}x{h}_'
                readers.append((view['name'],SharedMemoryCameraReader(prefix+'0',w,h,8),SharedMemoryCameraReader(prefix+'5',w,h,8)))
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
                    im=np.frombuffer(f.bgr_bytes,np.uint8).reshape(h,w,3);raw=np.frombuffer(sf.bgr_bytes,np.uint8).reshape(h,w,3);mask=decode(raw)
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
        flown=[x['telemetry'] for x in samples if x['telemetry'].get('flight',{}).get('phase')=='flying' and x['telemetry'].get('position_enu_m')]
        airborne=sum(x['position_enu_m'][2]>=1. for x in flown)/max(1,len(flown))
        report.update(airborne_fraction_of_flying_samples=airborne,
            maximum_height_m=max((x['telemetry'].get('position_enu_m') or [0,0,0])[2] for x in samples) if samples else 0.)
        latest_diag=next((x['telemetry'].get('diagnostics',{}) for x in reversed(samples) if x['telemetry'].get('diagnostics')), {})
        report['sensor_diagnostics']={k:v['values'] for k,v in latest_diag.items() if 'target_hz' in v['values']}
        sums=Counter();latency=[];last_scoring={}
        if run:
            metrics=list((run/'evaluation').glob('range_*/metrics.jsonl'))
            if metrics:
                for line in metrics[-1].open(encoding='utf8'):
                    try:m=json.loads(line)
                    except json.JSONDecodeError:continue
                    sums['epochs']+=1
                    if not m.get('valid',False):
                        sums['invalid_epochs']+=1
                        if m.get('scoring',True):sums['invalid_scoring_epochs']+=1
                        continue
                    if not m.get('scoring',True):continue
                    sums['scoring_epochs']+=1
                    last_scoring=m
                    for k in ('truth_count','detected_count','false_positive_count','fov_truth_count','fov_detected_count'):sums[k]+=m.get(k,0)
                    v=m.get('pipeline',{}).get('last_latency_ms')
                    if isinstance(v,(int,float)):latency.append(v)
                recall=sums['detected_count']/sums['truth_count'] if sums['truth_count'] else None
                precision=sums['detected_count']/(sums['detected_count']+sums['false_positive_count']) if sums['detected_count']+sums['false_positive_count'] else None
                summary_path=metrics[-1].parent/'run_summary.json'
                summary=json.loads(summary_path.read_text(encoding='utf8')) if summary_path.exists() else {}
                last=summary.get('last',{})
                if 'encounters' not in last:last=last_scoring
                enc=last.get('encounters',{});timely=last.get('timely_success_rate')
                completed=sum(enc.get(k,0) for k in ('success','late','missed'))
                valid=sums['scoring_epochs'];invalid=sums['invalid_scoring_epochs']
                coverage=valid/(valid+invalid) if valid+invalid else 0.
                report.update(counts=dict(sums),recall=recall,precision=precision,timely_success_rate=timely,
                    encounters=enc,completed_encounters=completed,scoring_valid_fraction=coverage,last_scoring_stamp_s=last_scoring.get('stamp_s'),
                    primary_metric='Detected within 0.5 s of each entry into the complete 20 m sphere',
                    meets_95=completed>=20 and timely is not None and timely>=.95 and coverage>=.95 and airborne>=.8 and len(flown)>=5 and error is None)
                if a.dual_front:
                    warning=last.get('hemisphere_warning') or summary.get('hemisphere_warning') or {}
                    events=warning.get('encounters',{});count=sum(events.get(k,0) for k in ('success','late','missed'))
                    timely=warning.get('timely_success_rate')
                    report.update(hemisphere_warning=warning,primary_metric='Actual warning within 0.5 s of each entry into 10 m forward hemisphere',
                        timely_warning_success_rate=timely,warning_encounters=count,
                        meets_90=count>=20 and timely is not None and timely>=.9 and coverage>=.95 and airborne>=.8 and error is None)
        (out/'result.json').write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf8')
        (out/'telemetry.json').write_text(json.dumps(samples,ensure_ascii=False),encoding='utf8')
        print(json.dumps(report,ensure_ascii=False),flush=True)
    if error:raise RuntimeError(error)


if __name__=='__main__':main()
