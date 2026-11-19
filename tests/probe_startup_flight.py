"""Explicit live startup probe; owns UE, flies the simulated ego, restores origin."""
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
from evtol_console.server import ConsoleBridge, ros_stop


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--actors',type=int,default=32)
    opts=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    os.environ['ROS_DOMAIN_ID']='221'
    rclpy.init(domain_id=221)
    engine=Engine(root,ros_stop=ros_stop)
    if opts.actors>len(engine.config['actors']['agents']):raise ValueError('Not enough configured actors')
    engine.config['actors']['count']=opts.actors
    engine.config['actors']['agents']=engine.config['actors']['agents'][:opts.actors]
    engine.config['actors']['pedestrians_enabled']=False
    engine.config['actors']['initialization']='first_waypoint'
    engine.config['ue']['visible']=False
    engine.config['run'].update(execute=True,actors=True,evaluate=True,new_session=True)
    origin=root/'data/session_origin.json'
    prior_origin=origin.read_bytes() if origin.exists() else None
    config=root/'config/console.local.json';config_hash=hashlib.sha256(config.read_bytes()).hexdigest()
    bridge=None;executor=None;spin=None;finished=threading.Event()
    report=dict(result='FAIL',actors=opts.actors,samples=[],initialization='first_waypoint')
    started=time.monotonic()
    try:
        bridge=ConsoleBridge(engine,'drone1',0)
        executor=SingleThreadedExecutor();executor.add_node(bridge)
        def run():
            while not finished.is_set():executor.spin_once(timeout_sec=.05)
        spin=threading.Thread(target=run,daemon=True);spin.start()
        def operation(action,timeout):
            response=engine.request(dict(action=action))
            if not response['ok']:raise RuntimeError(response)
            op_id=response['data']['operation_id'];deadline=time.monotonic()+timeout;logged=0.
            while time.monotonic()<deadline:
                status=engine.status()
                op=next(v for v in status['operations'] if v['id']==op_id)
                failure=status['telemetry'].get('startup',{})
                if failure.get('phase')=='failed':raise RuntimeError(failure['reason'])
                if op['state']=='done':return
                if op['state'] in ('failed','cancelled'):raise RuntimeError(op['message'])
                if time.monotonic()-logged>=5.:
                    logged=time.monotonic();states=status['telemetry'].get('actors',{}).get('actors',{})
                    ready=sum(bool(a.get('ready_for_experiment')) for a in states.values())
                    print(json.dumps(dict(operation=action,elapsed=round(time.monotonic()-started,1),ready=ready,
                        actors=opts.actors,flight=status['telemetry'].get('flight',{}))),flush=True)
                time.sleep(.2)
            raise TimeoutError(action+' timed out')
        operation('start_ue',140.)
        operation('start_ros',230.)
        deadline=time.monotonic()+90.;confirmed=None;logged=0.
        while time.monotonic()<deadline:
            status=engine.status();t=status['telemetry']
            actors=t.get('actors',{}).get('actors',{})
            failures={n:a for n,a in actors.items() if a['phase'] in ('failed','cleanup_failed','landing','stopped')}
            if failures:raise RuntimeError('Unexpected actor failure/landing: '+json.dumps(failures))
            if t.get('flight',{}).get('phase')=='failed':raise RuntimeError(str(t['flight']))
            position=t.get('position_enu_m',[0.,0.,0.])
            row=dict(elapsed_s=round(time.monotonic()-started,2),position_enu_m=position,
                     flight=t.get('flight',{}),mission=t.get('mission',{}),
                     actor_phases={n:a['phase'] for n,a in actors.items()})
            report['samples'].append(row)
            good=(t.get('flight',{}).get('phase')=='flying' and position[2]>2.
                  and len(actors)==opts.actors and all(a.get('ready_for_experiment') for a in actors.values())
                  and sum(a.get('waypoint_index',0)>=2 for a in actors.values())>=min(2,opts.actors))
            if good:
                if confirmed is None:confirmed=time.monotonic()
                if time.monotonic()-confirmed>=10.:
                    report.update(result='PASS',run=status['active_run'],elapsed_s=round(time.monotonic()-started,2),
                                  max_height_m=max(s['position_enu_m'][2] for s in report['samples']),
                                  all_actors_ready=True,unexpected_landing=False)
                    print(json.dumps({k:v for k,v in report.items() if k!='samples'}),flush=True)
                    break
            else:confirmed=None
            if time.monotonic()-logged>=5.:
                logged=time.monotonic();print(json.dumps(row),flush=True)
            time.sleep(.25)
        else:raise TimeoutError('Ego/actors did not confirm flight; inspect run logs')
    except BaseException as exc:
        report['error']=str(exc);report['run']=str(engine.active)
        raise
    finally:
        engine.close();finished.set()
        if spin:spin.join(timeout=3)
        if executor:executor.shutdown()
        if bridge:bridge.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        if prior_origin is None:
            if origin.exists():origin.unlink()
        else:origin.write_bytes(prior_origin)
        report['user_config_unchanged']=hashlib.sha256(config.read_bytes()).hexdigest()==config_hash
        report['remaining_owned_processes']=list(engine.processes)
        (root/f'reports/startup_flight_{opts.actors}.json').write_text(json.dumps(report,indent=2),encoding='utf8')


if __name__=='__main__':main()
