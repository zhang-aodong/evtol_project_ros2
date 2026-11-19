"""Real UE/ROS regression: full source scans, ROS/preview delivery and takeoff.

Run after scripts/activate.ps1. Starts/stops only its own processes; the user's
console draft and session origin are restored. A real AirSim project is required.
"""
import argparse
from collections import Counter, deque
import hashlib
import json
from pathlib import Path
import threading
import time
import sys

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from evtol_perception_interfaces.msg import VelocityCommand, TargetArray
from evtol_console.engine import Engine
from evtol_console.server import ConsoleBridge, ros_stop
from evtol_console.processes import OwnedProcess
from evtol_simulation.lidar_ring import LidarSharedMemory


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1])
    parser.add_argument('--label',default='flight_lidar_fixed')
    parser.add_argument('--seconds',type=float,default=65.)
    parser.add_argument('--headless',action='store_true')
    a=parser.parse_args();root=a.root.resolve()
    config=root/'config/console.local.json';original_hash=hashlib.sha256(config.read_bytes()).hexdigest()
    origin=root/'data/session_origin.json';original_origin=origin.read_bytes() if origin.exists() else None
    rclpy.init(domain_id=223);engine=Engine(root,ros_stop=ros_stop)
    engine.config['ue']['visible']=not a.headless
    engine.config['run'].update(execute=True,actors=True,evaluate=True,new_session=True)
    done=threading.Event();source_done=threading.Event();executor=None;bridge=None;thread=None;source_thread=None;preview=None
    streams={name:deque(maxlen=12000) for name in ('lidar','navigation','preview')}
    source={name:[] for name in ('Lidar1','NavigationLidar')};commands=[];samples=[];risks=[];tracks=[]
    report=dict(result='FAIL',label=a.label,actors=engine.config['actors']['count'],visible=not a.headless)
    def cloud(name,msg):
        fields={f.name:f.offset for f in msg.fields}
        dtype=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[fields[k] for k in ('x','y','z')],'itemsize':msg.point_step})
        data=np.frombuffer(msg.data,dtype=dtype,count=msg.width*msg.height)
        angles=np.mod(np.degrees(np.arctan2(data['y'],data['x'])),360.)
        mask=np.hypot(data['x'],data['y'])>.5;angles=np.sort(angles[mask])
        span=float(360.-np.max(np.diff(np.r_[angles,angles[0]+360.]))) if len(angles)>1 else 0.
        streams[name].append(dict(wall=time.time(),stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
            points=msg.width*msg.height,span_deg=span,sha256=hashlib.sha256(msg.data).hexdigest()))
    def command(msg):commands.append(dict(wall=time.time(),reason=msg.reason,v=[msg.velocity.x,msg.velocity.y,msg.velocity.z]))
    def targets(name,msg):
        tracks.append(dict(wall=time.time(),source=name,stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
            targets=[dict(id=int(t.track_id),cls=t.class_id,state=t.state,position=[t.position.x,t.position.y,t.position.z],
                velocity=[t.velocity.x,t.velocity.y,t.velocity.z],size=[t.size.x,t.size.y,t.size.z],covariance=list(t.covariance)) for t in msg.targets]))
    def sample():
        t=engine.status()['telemetry'];row=dict(wall=time.time(),position=t.get('position_enu_m'),flight=t.get('flight',{}),
            mission=t.get('mission',{}),actors_ready=sum(bool(v.get('ready_for_experiment')) for v in t.get('actors',{}).get('actors',{}).values()))
        samples.append(row);return row
    def log():
        row=sample();mission=row['mission']
        print(json.dumps(dict(wall=row['wall'],position=row['position'],flight=row['flight'].get('phase'),
            mission=mission.get('phase'),reason=mission.get('reason'),actors_ready=row['actors_ready'],
            safety=mission.get('safety_data')),ensure_ascii=False),flush=True)
    def acquire_source():
        readers={name:LidarSharedMemory('AirSimLidar_'+name,65536,8) for name in source};last={}
        try:
            while not source_done.wait(.005):
                for name,reader in readers.items():
                    frame=reader.read_latest(last.get(name,0))
                    if frame is None:continue
                    last[name]=frame.frame_id
                    source[name].append(dict(wall=time.time(),id=frame.frame_id,stamp_ns=frame.timestamp_ns,
                        points=frame.point_count,complete=frame.scan_complete))
        finally:
            for reader in readers.values():reader.close()
    def operation(action,timeout):
        result=engine.request(dict(action=action));assert result['ok'],result
        identifier=result['data']['operation_id'];deadline=time.monotonic()+timeout;next_log=0.
        while time.monotonic()<deadline:
            item=next(v for v in engine.status()['operations'] if v['id']==identifier)
            if item['state']=='done':return
            if item['state'] in ('failed','cancelled'):raise RuntimeError(item['message'])
            if time.monotonic()>=next_log:log();next_log=time.monotonic()+5.
            time.sleep(.2)
        raise TimeoutError(action)
    try:
        bridge=ConsoleBridge(engine,'drone1',0)
        for name,topic in [('lidar','lidar/points'),('navigation','navigation/lidar/points'),('preview','preview/lidar/points')]:
            bridge.create_subscription(PointCloud2,'/drone1/'+topic,lambda m,n=name:cloud(n,m),qos_profile_sensor_data)
        bridge.create_subscription(VelocityCommand,'/drone1/control/safe_velocity',command,qos_profile_sensor_data)
        for name,topic in [('fusion','perception/tracks'),('lidar','perception/lidar_obstacles')]:
            bridge.create_subscription(TargetArray,'/drone1/'+topic,lambda m,n=name:targets(n,m),qos_profile_sensor_data)
        bridge.create_subscription(String,'/drone1/safety/risks',lambda m:risks.append(dict(wall=time.time(),data=json.loads(m.data))),10)
        executor=SingleThreadedExecutor();executor.add_node(bridge)
        def spin():
            while not done.is_set():executor.spin_once(timeout_sec=.02)
        thread=threading.Thread(target=spin,daemon=True);thread.start()
        operation('start_ue',140.)
        source_thread=threading.Thread(target=acquire_source,daemon=True);source_thread.start()
        preview=OwnedProcess('preview',[sys.executable,'-u','-m','evtol_visualization.preview'],root,root/'reports'/('preview_'+a.label+'.log'))
        operation('start_ros',240.)
        started=time.time();deadline=time.monotonic()+a.seconds;next_log=0.
        while time.monotonic()<deadline:
            if time.monotonic()>=next_log:log();next_log=time.monotonic()+2.
            else:sample()
            time.sleep(.25)
        flight=[s for s in samples if s['wall']>=started and s['position']]
        assert any(s['position'][2]>=8. and s['flight'].get('phase')=='flying' for s in flight),'Never reached 8 m in confirmed flight'
        first=next(s['wall'] for s in flight if s['position'][2]>2.)
        assert flight[-1]['wall']-first>=10.,'Insufficient sustained-flight observation'
        assert not any(s['position'][2]<.3 and s['flight'].get('phase') not in ('landing','landed') for s in flight if s['wall']>first),'Unexpected return to ground'
        for name,rows in source.items():
            assert len(rows)>400,(name,'insufficient frames')
            assert all(r['complete'] for r in rows),(name,'partial source scan')
            assert all(r['points']>0 for r in rows),(name,'empty source frame')
        for name,rows in streams.items():assert len(rows)>50,(name,'missing ROS/preview stream')
        assert not any(c['reason']=='Waiting for complete navigation LiDAR sweep' for c in commands if c['wall']>started),'False sweep hold remains'
        report['result']='PASS'
    except BaseException as exc:
        report['error']=str(exc);raise
    finally:
        source_done.set()
        if source_thread:source_thread.join(timeout=3.)
        engine.close();done.set()
        if preview:preview.stop()
        if thread:thread.join(timeout=3.)
        if executor:executor.shutdown()
        if bridge:bridge.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        if original_origin is None:origin.unlink(missing_ok=True)
        else:origin.write_bytes(original_origin)
        report.update(source=source,streams={k:list(v) for k,v in streams.items()},samples=samples,commands=commands,risks=risks,tracks=tracks,
            run=str(engine.active),config_unchanged=hashlib.sha256(config.read_bytes()).hexdigest()==original_hash,
            remaining_owned_processes=list(engine.processes))
        (root/'reports'/('probe_'+a.label+'.json')).write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
        print(json.dumps({k:v for k,v in report.items() if k not in ('source','streams','samples','commands','risks','tracks')},ensure_ascii=False),flush=True)

if __name__=='__main__':main()
