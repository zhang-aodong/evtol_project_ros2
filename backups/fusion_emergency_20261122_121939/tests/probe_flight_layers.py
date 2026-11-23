"""Isolated UE smoke test: sampled straight flight, no user configuration writes."""
import re
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
    p.add_argument('--root',type=Path)
    p.add_argument('--case',choices=['straight','turns','ground_hold','crossing','raw_emergency','raw_side'],default='straight')
    a=p.parse_args();root=a.root.resolve() if a.root else Path(__file__).resolve().parents[1]
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
    if a.case=='turns':engine.config['mission']['waypoints_enu_m']=[[0,0,a.height],[0,12,a.height],[8,12,a.height],[8,4,a.height]]
    if a.case=='ground_hold':engine.config['mission']['waypoints_enu_m']=[[0,0,a.height],[0,12,a.height],[0,12,0]]
    if a.case=='raw_emergency':
        engine.config['mission']['waypoints_enu_m']=[[0,0,a.height],[0,24,a.height]]
        engine.config['actors'].update(count=1,agents=[dict(name='Drone2',speed_mps=1.,spawn_ned_m=[12.,0.,0.],waypoints_enu_m=[[0.,12.,a.height],[0.,12.,a.height]],repeat=0)],initialization='first_waypoint')
        engine.config['dynamic']['enabled']=False
        engine.config['run'].update(actors=True,evaluate=True)
    if a.case=='raw_side':
        engine.config['mission']['waypoints_enu_m']=[[0.,0.,a.height]]
        engine.config['actors'].update(count=1,agents=[dict(name='Drone2',speed_mps=.8,
            spawn_ned_m=[0.,7.,0.],waypoints_enu_m=[[7.,0.,a.height],[-7.,0.,a.height]],repeat=0)],
            initialization='first_waypoint')
        engine.config['dynamic']['enabled']=False
        engine.config['localization']['state_source']='truth'
        engine.config['raw_emergency'].update(enabled=True,radius_m=2.)
        engine.config['run'].update(actors=True,evaluate=True)
    if a.case=='crossing':
        engine.config['mission']['waypoints_enu_m']=[[0,0,a.height],[0,24,a.height]]
        agents=[]
        for i in range(6):
            y=10.+i*3.;x=(-1 if i%2 else 1)*(4.+i*.4)
            agents.append(dict(name=f'Drone{i+2}',speed_mps=.7+.1*i,spawn_ned_m=[y,x,0.],waypoints_enu_m=[[x,y,a.height],[ -x,y,a.height]],repeat=0))
        engine.config['actors'].update(count=6,agents=agents,initialization='first_waypoint')
        engine.config['run'].update(actors=True,evaluate=True)
    bridge=ConsoleBridge(engine,'drone1',0);executor=SingleThreadedExecutor();executor.add_node(bridge)
    done=threading.Event();rows=[];latest={};error=None;collisions=[];last_collision=0;last_check=0
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
            row=json.loads(json.dumps(dict(wall=time.time(),**latest,mission=telemetry.get('mission'),
                flight=telemetry.get('flight'),actors=telemetry.get('actors') if a.case=='raw_side' else None)))
            rows.append(row)
            if time.monotonic()-last_check>1.:
                last_check=time.monotonic();rpc=engine.rpc(1.)
                try:
                    c=rpc.call('simGetCollisionInfo','Drone1');ts=int(c.get('time_stamp',0))
                    if c.get('has_collided') and ts>last_collision:
                        last_collision=ts
                        if row.get('position',[0,0,0])[2]>1. :collisions.append(dict(wall=time.time(),collision=c,position=row.get('position')))
                finally:rpc.close()
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
            case=a.case,airborne_collisions=collisions,completed=any((r.get('mission') or {}).get('phase')=='complete' for r in rows),
            config_unchanged=hashlib.sha256(cfg.read_bytes()).hexdigest()==original_hash,owned_remaining=list(engine.processes))
        stopped=maximum_stopped=0.
        for row in rows:
            if (row.get('mission') or {}).get('phase')=='waypoint' and np.linalg.norm(row.get('velocity',[0,0,0]))<.15:stopped+=.1
            else:stopped=0.
            maximum_stopped=max(maximum_stopped,stopped)
        result['longest_stationary_waypoint_s']=maximum_stopped
        result['collision_records_raw']=list(collisions)
        if run and (run/'gateway.log').is_file():
            match=re.search(r'\[([\d.]+)\].*Flight phase: takeoff',(run/'gateway.log').read_text())
            if match:
                takeoff_ns=int(float(match[1])*1e9)
                result['airborne_collisions']=[v for v in collisions if v['collision']['time_stamp']>=takeoff_ns]
                result['spawn_contacts_before_takeoff']=[v for v in collisions if v['collision']['time_stamp']<takeoff_ns]
                result['takeoff_stamp_ns']=takeoff_ns
        if a.case=='raw_side':
            guard_states=[(row.get('mission') or {}).get('safety_data',{}).get('lidar_emergency',{}) for row in rows]
            active=[state for state in guard_states if state.get('active')]
            result['lidar_emergency']=dict(
                active_samples=len(active),
                nearest_return_m=min((state['nearest_m'] for state in active),default=None),
                actions=sorted({state['action'] for state in active if 'action' in state}),
            )
            if not error and (not active or result['airborne_collisions'] or not result['completed']):
                error='Expected LiDAR intervention, mission completion and zero airborne collisions'
                result['error']=error
        (out/'samples.json').write_text(json.dumps(rows),encoding='utf8')
        (out/'result.json').write_text(json.dumps(result,indent=2),encoding='utf8')
        print(json.dumps(result),flush=True)
    if error:raise SystemExit(1)


if __name__=='__main__':main()
