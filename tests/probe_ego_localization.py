"""Actual UE run + sensor bag. Only owns its own processes and private DDS domain."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import threading
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from evtol_console.engine import Engine
from evtol_console.server import ConsoleBridge,ros_stop
from evtol_localization.options import options
from evtol_collection.options import defaults as collection_defaults


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1])
    parser.add_argument('--seconds',type=float,default=90);parser.add_argument('--fused',action='store_true')
    a=parser.parse_args();root=a.root.resolve();label='ego_localization_'+time.strftime('%Y%m%d_%H%M%S')
    out=root/'reports'/label;out.mkdir(parents=True)
    cfg=root/'config/console.local.json';config_hash=hashlib.sha256(cfg.read_bytes()).hexdigest()
    origin=root/'data/session_origin.json';saved_origin=origin.read_bytes() if origin.exists() else None
    os.environ['ROS_DOMAIN_ID']='223';rclpy.init(domain_id=223)
    engine=Engine(root,ros_stop=ros_stop)
    engine.config['actors'].update(count=0,agents=[],pedestrians_enabled=False)
    engine.config['ue']['visible']=False
    engine.config['mission'].update(mode='waypoints',waypoints_enu_m=[[0.,0.,8.],[0.,60.,8.]],land_at_last=False,speed_mps=.7)
    engine.config['run'].update(execute=True,actors=False,evaluate=False,new_session=True)
    engine.config['localization']=options(dict(mode='gnss_imu_5g' if a.fused else 'gnss_imu',state_source='fused' if a.fused else 'truth'))
    engine.config['localization']['gnss_fault']['enabled']=True
    bridge=ConsoleBridge(engine,'drone1',0);executor=SingleThreadedExecutor();executor.add_node(bridge)
    done=threading.Event();samples=[];error=None;collection=None
    def spin():
        while not done.is_set():executor.spin_once(timeout_sec=.01)
    thread=threading.Thread(target=spin,daemon=True);thread.start()
    def operation(action,payload=None,timeout=250):
        result=engine.request(dict(action=action,payload=payload or {}))
        if not result['ok']:raise RuntimeError(result)
        identifier=result['data']['operation_id'];deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            op=next(x for x in engine.status()['operations'] if x['id']==identifier)
            if op['state']=='done':return op
            if op['state'] in ('failed','cancelled'):raise RuntimeError(op['message'])
            done.wait(.2)
        raise TimeoutError(action)
    try:
        operation('start_ue');operation('start_ros')
        settings=collection_defaults();prefix='/drone1/'
        settings['output_dir']=str(out/'sensor_bag')
        settings['bag']['topics']=['/clock','/simulation/reference_origin','/simulation/geodetic_reference']+[prefix+x for x in
            ('imu/data','gnss/home','gnss/fix','five_ga/observations','ground_truth/local_odometry','ground_truth/odometry','control/flight_status','localization/status','localization/odometry')]
        operation('start_collection',dict(mode='bag',config=settings),timeout=40)
        deadline=time.monotonic()+a.seconds
        while time.monotonic()<deadline:
            s=engine.status();t=s['telemetry'];samples.append(dict(wall=time.time(),telemetry=t))
            print(json.dumps(dict(position=t.get('position_enu_m'),flight=t.get('flight'),localization=t.get('localization_status'),metrics=t.get('localization_metrics')),ensure_ascii=False),flush=True)
            if (t.get('flight') or {}).get('phase')=='failed':raise RuntimeError('Flight failed: '+str(t.get('flight')))
            done.wait(5.)
        operation('stop_collection',timeout=40);collection=engine.status().get('collection')
    except BaseException as exc:error=str(exc);print('PROBE ERROR: '+error,flush=True)
    finally:
        run=engine.active
        engine.close();done.set();thread.join(timeout=3);executor.shutdown();bridge.destroy_node();rclpy.shutdown()
        if saved_origin is None:origin.unlink(missing_ok=True)
        else:origin.write_bytes(saved_origin)
        final=samples[-1]['telemetry'] if samples else {}
        result=dict(error=error,run=str(run),mode='fused' if a.fused else 'shadow',report=str(out),
            collection=collection,last_localization=final.get('localization_status'),last_metrics=final.get('localization_metrics'),
            config_unchanged=hashlib.sha256(cfg.read_bytes()).hexdigest()==config_hash,owned_remaining=list(engine.processes))
        (out/'result.json').write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf8')
        (out/'telemetry.json').write_text(json.dumps(samples,ensure_ascii=False),encoding='utf8')
        print(json.dumps(result,ensure_ascii=False),flush=True)
    if error:raise SystemExit(1)


if __name__=='__main__':main()
