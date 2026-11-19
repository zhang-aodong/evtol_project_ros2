"""Explicit UE flight regression. Never imported by unittest discovery."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import statistics
import threading
import time
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from std_msgs.msg import String
from evtol_perception_interfaces.msg import TargetArray
from evtol_console.engine import Engine
from evtol_console.server import ConsoleBridge,ros_stop
from evtol_targeting.common import LATEST,ns
from evtol_scenario.flight import PositionFlight
from evtol_simulation.coordinates import NED_ENU


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--mode',choices=['32','side'],default='32')
    parser.add_argument('--root',required=True);opts=parser.parse_args()
    root=Path(opts.root);os.environ['ROS_DOMAIN_ID']='222';rclpy.init(domain_id=222)
    engine=Engine(root,ros_stop=ros_stop)
    engine.config['ue']['visible']=False
    engine.config['actors'].update(initialization='first_waypoint',pedestrians_enabled=False)
    engine.config['run'].update(execute=True,actors=True,evaluate=True,new_session=True)
    if opts.mode=='side':
        engine.config['actors'].update(count=1,agents=[dict(name='Drone2',speed_mps=2.,
            spawn_ned_m=[0.,20.,0.],waypoints_enu_m=[[20.,0.,10.],[-20.,0.,10.]],repeat=1)])
        engine.config['mission'].update(mode='waypoints',waypoints_enu_m=[[0.,0.,10.]],land_at_last=False)
        # For this controlled test, hold the obstacle until the ego has reached
        # a stable hover. No second controller commands the same vehicle.
        engine.config['run']['actors']=False
    config=root/'config/console.local.json';before_config=hashlib.sha256(config.read_bytes()).hexdigest()
    origin=root/'data/session_origin.json';before_origin=origin.read_bytes() if origin.exists() else None
    report=dict(result='FAIL',mode=opts.mode,samples=[]);started=time.monotonic();finished=threading.Event()
    bridge=None;observer=None;executor=None;spin=None
    raw_age=[];realtime_age=[];fused_age=[];lidar_age=[];side_positions=[]
    manual_stop=threading.Event();crossing=threading.Event();actor_thread=None;actor_error=[]
    latest_lidar={};latest_truth=[];matched_side_warning=threading.Event();latest_risks=[]
    try:
        bridge=ConsoleBridge(engine,'drone1',0)
        observer=Node('perception_live_probe');observer.set_parameters([Parameter('use_sim_time',value=True)])
        def age(storage,msg):
            value=(observer.get_clock().now().nanoseconds-ns(msg.header.stamp))*1e-6
            if value>=0:storage.append(value)
        observer.create_subscription(Image,'/drone1/camera/rgb/image_raw',lambda m:age(raw_age,m),LATEST)
        observer.create_subscription(Image,'/drone1/camera/rgb/image_realtime',lambda m:age(realtime_age,m),LATEST)
        observer.create_subscription(TargetArray,'/drone1/perception/targets_measurements',lambda m:age(fused_age,m),LATEST)
        def receive_lidar(msg):
            age(lidar_age,msg)
            latest_lidar.clear();latest_lidar.update({int(t.track_id):np.array([t.position.x,t.position.y,t.position.z]) for t in msg.targets})
        def receive_truth(msg):
            if msg.targets:
                p=msg.targets[0].position;side_positions.append(float(p.x));latest_truth[:]=[np.array([p.x,p.y,p.z])]
        def receive_risks(msg):
            latest_risks[:]=sorted(json.loads(msg.data).get('targets',[]),key=lambda r:r.get('cpa_distance_m',0)-r.get('envelope_m',0))[:3]
            if not crossing.is_set() or not latest_truth:return
            for risk in json.loads(msg.data).get('targets',[]):
                p=latest_lidar.get(int(risk['track_id']))
                if risk.get('warning') and risk.get('source')=='lidar_only' and p is not None and np.linalg.norm(p-latest_truth[0])<2.:
                    matched_side_warning.set()
        observer.create_subscription(TargetArray,'/drone1/perception/lidar_obstacles',receive_lidar,LATEST)
        observer.create_subscription(TargetArray,'/simulation/truth/targets',receive_truth,LATEST)
        observer.create_subscription(String,'/drone1/safety/risks',receive_risks,LATEST)
        executor=SingleThreadedExecutor();executor.add_node(bridge);executor.add_node(observer)
        def run():
            while not finished.is_set():executor.spin_once(timeout_sec=.02)
        spin=threading.Thread(target=run,daemon=True);spin.start()
        def action(name,timeout):
            response=engine.request(dict(action=name))
            if not response['ok']:raise RuntimeError(response)
            identifier=response['data']['operation_id'];end=time.monotonic()+timeout;logged=0.
            while time.monotonic()<end:
                s=engine.status();op=next(o for o in s['operations'] if o['id']==identifier)
                if op['state']=='done':return
                if op['state'] in ('failed','cancelled'):raise RuntimeError(op['message'])
                fail=s['telemetry'].get('startup',{})
                if fail.get('phase')=='failed':raise RuntimeError(fail['reason'])
                if time.monotonic()-logged>5:
                    logged=time.monotonic();actors=s['telemetry'].get('actors',{}).get('actors',{})
                    print(json.dumps(dict(action=name,elapsed_s=round(time.monotonic()-started,1),
                        actors_ready=sum(bool(a.get('ready_for_experiment')) for a in actors.values()))),flush=True)
                time.sleep(.2)
            raise TimeoutError(name)
        action('start_ue',140);action('start_ros',240)
        if opts.mode=='side':
            data=json.loads(origin.read_text(encoding='utf8'))
            offset=np.array([0.,20.,0.]);origin_enu=np.array(data['position'])
            first=NED_ENU@(origin_enu+[20.,0.,10.])-offset
            end=NED_ENU@(origin_enu+[-20.,0.,10.])-offset
            def manual_actor():
                rpc=engine.rpc(5.)
                try:
                    rpc.call('enableApiControl',True,'Drone2')
                    flight=PositionFlight(rpc,'Drone2',manual_stop)
                    if not flight.place(first):return
                    while not manual_stop.is_set() and not crossing.is_set():flight.step(first,1.);manual_stop.wait(.08)
                    if manual_stop.is_set():return
                    flight.move(end,2.,40.)
                    while not manual_stop.is_set():flight.step(end,1.);manual_stop.wait(.08)
                except Exception as exc:actor_error.append(str(exc))
                finally:rpc.close()
            actor_thread=threading.Thread(target=manual_actor,daemon=True);actor_thread.start()
        flight_start=None;side_warning=False;max_recall=0.;max_fov=0.;max_lidar=0;logged=0.
        deadline=time.monotonic()+130.
        while time.monotonic()<deadline:
            s=engine.status();t=s['telemetry'];flight=t.get('flight',{});metric=t.get('detection_metrics',{})
            if flight.get('phase')=='failed':raise RuntimeError(str(flight))
            if actor_error:raise RuntimeError(actor_error[0])
            if flight.get('phase')=='flying' and flight_start is None:flight_start=time.monotonic()
            for name,a in t.get('actors',{}).get('actors',{}).items():
                if a['phase'] in ('failed','landing','cleanup_failed'):raise RuntimeError(name+': '+str(a))
            diagnostics=t.get('diagnostics',{});li=diagnostics.get('/drone1/lidar_obstacles',{}).get('values',{})
            max_lidar=max(max_lidar,int(li.get('active_tracks',0)))
            max_recall=max(max_recall,metric.get('recall') or 0.)
            max_fov=max(max_fov,metric.get('fov_recall') or 0.)
            reason=t.get('mission',{}).get('reason','')
            if opts.mode=='side':
                if not crossing.is_set() and t.get('mission',{}).get('phase')=='complete' and t.get('position_enu_m',[0,0,0])[2]>9.:
                    report['crossing_start_ego_position']=t['position_enu_m'];crossing.set()
                side_warning=matched_side_warning.is_set()
            row=dict(elapsed_s=round(time.monotonic()-started,2),position=t.get('position_enu_m'),
                flight=flight.get('phase'),reason=reason,pipeline=metric.get('pipeline'),
                truth_count=metric.get('truth_count'),detected_count=metric.get('detected_count'),
                fov_recall=metric.get('fov_recall'),lidar_tracks=int(li.get('active_tracks',0)),
                min_distance=metric.get('minimum_center_distance_m'),collision_count=metric.get('collision_count'),
                side_east=side_positions[-1] if side_positions else None,
                safety=diagnostics.get('/drone1/safety',{}).get('values',{}),risks=latest_risks.copy())
            report['samples'].append(row)
            if time.monotonic()-logged>5:
                logged=time.monotonic();print(json.dumps(row),flush=True)
            if flight_start is not None and time.monotonic()-flight_start>40:
                if opts.mode=='32' and max_recall>0 and max_lidar>0:
                    report['result']='PASS';break
                if opts.mode=='side' and side_positions and side_positions[-1]<-10:
                    minimum=metric.get('minimum_center_distance_m')
                    if side_warning and max_lidar>0 and minimum is not None and minimum>2 and metric.get('collision_count')==0:
                        report['result']='PASS';break
                    raise RuntimeError('Side crossing failed: '+json.dumps(row))
            time.sleep(.25)
        if report['result']!='PASS':raise TimeoutError('No validated timely detection / side avoidance')
        report.update(max_recall=max_recall,max_fov_recall=max_fov,max_lidar_tracks=max_lidar,side_warning=side_warning)
    except BaseException as exc:
        report['error']=str(exc);raise
    finally:
        report['run']=str(engine.active);report['elapsed_s']=round(time.monotonic()-started,2)
        def stats(values):
            if not values:return None
            return dict(samples=len(values),p50_ms=round(statistics.median(values),2),p90_ms=round(float(np.percentile(values,90)),2),max_ms=round(max(values),2))
        report['ages']={name:stats(values) for name,values in [('raw_rgb',raw_age),('realtime_rgb',realtime_age),('fused',fused_age),('lidar',lidar_age)]}
        manual_stop.set()
        if actor_thread:actor_thread.join(timeout=6.)
        engine.close();finished.set()
        if spin:spin.join(timeout=3)
        if executor:executor.shutdown()
        if observer:observer.destroy_node()
        if bridge:bridge.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        if before_origin is None:
            if origin.exists():origin.unlink()
        else:origin.write_bytes(before_origin)
        report['user_config_unchanged']=hashlib.sha256(config.read_bytes()).hexdigest()==before_config
        report['remaining_owned_processes']=list(engine.processes)
        (root/f'reports/perception_live_{opts.mode}.json').write_text(json.dumps(report,indent=2),encoding='utf8')
        print(json.dumps({k:v for k,v in report.items() if k!='samples'}),flush=True)


if __name__=='__main__':main()
