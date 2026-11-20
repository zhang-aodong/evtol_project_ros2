"""Isolated UE smoke test: sampled straight flight, no user configuration writes."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import threading
import time
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from nav_msgs.msg import Odometry
from evtol_perception_interfaces.msg import VelocityCommand
from evtol_console.engine import Engine
from evtol_console.server import ConsoleBridge,ros_stop
from evtol_targeting.common import LATEST


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds',type=float,default=65)
    p.add_argument('--label',default='fov100_cruise')
    p.add_argument('--height',type=float,default=8.)
    a=p.parse_args();root=Path(__file__).resolve().parents[1]
    cfg=root/'config/console.local.json';original_hash=hashlib.sha256(cfg.read_bytes()).hexdigest()
    origin=root/'data/session_origin.json';original_origin=origin.read_bytes() if origin.exists() else None
    out=root/'reports'/a.label;out.mkdir(exist_ok=True)
    os.environ['ROS_DOMAIN_ID']='224';rclpy.init(domain_id=224)
    engine=Engine(root,ros_stop=ros_stop)
    engine.config['actors'].update(count=0,agents=[],pedestrians_enabled=False)
    engine.config['mission'].update(mode='waypoints',speed_mps=2.,max_speed_mps=2.,max_acceleration_mps2=1.,
        waypoints_enu_m=[[0,0,a.height],[0,12,a.height],[0,24,a.height]],land_at_last=False)
    engine.config['run'].update(execute=True,actors=False,evaluate=False,new_session=True)
    engine.config['ue']['visible']=False
    bridge=ConsoleBridge(engine,'drone1',0);executor=SingleThreadedExecutor();executor.add_node(bridge)
    done=threading.Event();rows=[];latest={};error=None
    def pose(m):
        p=m.pose.pose.position;v=m.twist.twist.linear
        latest.update(position=[p.x,p.y,p.z],velocity=[v.x,v.y,v.z])
    def command(m,key):latest[key]=dict(velocity=[m.velocity.x,m.velocity.y,m.velocity.z],reason=m.reason)
    bridge.create_subscription(Odometry,'/drone1/state/odometry',pose,LATEST)
    for key in ('safe','desired'):
        bridge.create_subscription(VelocityCommand,'/drone1/control/'+('safe_velocity' if key=='safe' else 'desired_velocity'),lambda m,k=key:command(m,k),LATEST)
    def spin():
        while not done.is_set():executor.spin_once(timeout_sec=.01)
    thread=threading.Thread(target=spin,daemon=True);thread.start()
    def operation(action,timeout):
        response=engine.request(dict(action=action))
        if not response['ok']:raise RuntimeError(response)
        identifier=response['data']['operation_id'];deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            item=next(x for x in engine.status()['operations'] if x['id']==identifier)
            if item['state']=='done':return
            if item['state'] in ('failed','cancelled'):raise RuntimeError(item['message'])
            time.sleep(.2)
        raise TimeoutError(action)
    try:
        operation('start_ue',140);operation('start_ros',250)
        deadline=time.monotonic()+a.seconds;last_print=0.
        while time.monotonic()<deadline:
            status=engine.status();telemetry=status['telemetry']
            row=json.loads(json.dumps(dict(wall=time.time(),**latest,mission=telemetry.get('mission'),flight=telemetry.get('flight'))))
            rows.append(row)
            if time.monotonic()-last_print>5:
                print(json.dumps(row),flush=True);last_print=time.monotonic()
            time.sleep(.1)
    except BaseException as exc:error=str(exc);print('PROBE ERROR: '+error,flush=True)
    finally:
        run=engine.active;engine.close();done.set();thread.join(timeout=3)
        executor.shutdown();bridge.destroy_node();rclpy.shutdown()
        if original_origin is None:origin.unlink(missing_ok=True)
        else:origin.write_bytes(original_origin)
        cruise=[r for r in rows if 'position' in r and 4<r['position'][1]<19 and abs(r['position'][2]-a.height)<1.]
        def summary(values):
            return dict(median=float(np.median(values)),p05=float(np.percentile(values,5)),p95=float(np.percentile(values,95))) if values else None
        planning=[];seen=set()
        for row in rows:
            mission=row.get('mission') or {};data=mission.get('safety_data',{});count=data.get('planning_updates')
            if count is not None and count not in seen:
                seen.add(count);planning.append((mission['stamp_ns']*1e-9,count))
        plan_hz=(planning[-1][1]-planning[0][1])/(planning[-1][0]-planning[0][0]) if len(planning)>1 else None
        result=dict(error=error,run=str(run),height_m=a.height,samples=len(rows),cruise_samples=len(cruise),
            observed_planning_hz=plan_hz,safety_compute_ms=summary([r['mission']['safety_data']['compute_ms'] for r in rows if r.get('mission') and 'compute_ms' in r['mission'].get('safety_data',{})]),
            measured_speed_mps=summary([np.linalg.norm(r['velocity']) for r in cruise]),
            safe_speed_mps=summary([np.linalg.norm(r['safe']['velocity']) for r in cruise if 'safe' in r]),
            desired_speed_mps=summary([np.linalg.norm(r['desired']['velocity']) for r in cruise if 'desired' in r]),
            final_position=rows[-1].get('position') if rows else None,final_mission=rows[-1].get('mission') if rows else None,
            config_unchanged=hashlib.sha256(cfg.read_bytes()).hexdigest()==original_hash,owned_remaining=list(engine.processes))
        (out/'samples.json').write_text(json.dumps(rows),encoding='utf8')
        (out/'result.json').write_text(json.dumps(result,indent=2),encoding='utf8')
        print(json.dumps(result),flush=True)
    if error:raise SystemExit(1)


if __name__=='__main__':main()
