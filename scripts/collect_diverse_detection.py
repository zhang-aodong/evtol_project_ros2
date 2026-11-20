"""Independent UE sessions; scripted acquisition poses, never used for flight acceptance."""
import argparse,copy,hashlib,json,math,os,threading,time
from pathlib import Path
import numpy as np
import cv2
import rclpy
from rclpy.executors import SingleThreadedExecutor
from evtol_console.engine import Engine
from evtol_console.server import ConsoleBridge,ros_stop
from evtol_collection.dataset import labels_from_mask

def main():
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--counts',default='650,140,140');a=p.parse_args();root=a.root.resolve();out=a.output.resolve();out.mkdir(parents=True,exist_ok=True)
    os.environ['ROS_DOMAIN_ID']='225';rclpy.init(domain_id=225)
    cfg=root/'config/console.local.json';before=hashlib.sha256(cfg.read_bytes()).hexdigest();origin=root/'data/session_origin.json';prior=origin.read_bytes() if origin.exists() else None
    import evtol_console.engine as em
    original=em.artifacts
    def artifacts(*args,**kwargs):
        result=original(*args,**kwargs);result[1]['surround']['record_truth']=True
        from evtol_targeting.surround_geometry import camera_settings
        ego=result[1]['airsim']['vehicle'];s=result[1]['surround']
        for view in s['cameras']:result[3]['Vehicles'][ego]['Cameras'][view['name']]=camera_settings(s,view)
        return result
    em.artifacts=artifacts
    from evtol_simulation.camera_ring import SharedMemoryCameraReader
    from evtol_simulation.semantic import SemanticDecoder
    import yaml
    decoder=SemanticDecoder(yaml.safe_load((root/'src/evtol_simulation/config/source_parameters.yaml').read_text(encoding='utf8'))['semantic_segmentation']['classes'])
    if (out/'collection_report.json').exists() and json.loads((out/'collection_report.json').read_text()).get('complete'):
        raise FileExistsError('Completed dataset exists; choose a new output directory')
    report=dict(sessions=[],truth='semantic connected regions, not instance IDs',pose_mode='scripted acquisition poses; not closed-loop flight validation',complete=False,
        geometry=dict(width=1536,height=1536,hfov_deg=70,yaw_deg=[-15,15]),yolo_to_source={'0':6})
    try:
        for si,(split,count) in enumerate(zip(('train','val','test'),map(int,a.counts.split(',')))):
            engine=Engine(root,ros_stop=ros_stop);done=threading.Event();readers=[];rpc=None
            bridge=ConsoleBridge(engine,'drone1',0);executor=SingleThreadedExecutor();executor.add_node(bridge)
            def spin():
                while not done.is_set():executor.spin_once(timeout_sec=.03)
            thread=threading.Thread(target=spin,daemon=True);thread.start()
            try:
                agents=[dict(name=f'Drone{i+2}',speed_mps=1.,spawn_ned_m=[0.,0.,0.],waypoints_enu_m=[[0.,10.+i*5,12.]],repeat=0) for i in range(8)]
                engine.config['actors'].update(count=len(agents),agents=agents,pedestrians_enabled=False)
                engine.config['mission'].update(waypoints_enu_m=[[0,0,12]],land_at_last=False)
                engine.config['run'].update(execute=False,actors=False,evaluate=False,new_session=True)
                engine.config['ue']['visible']=False
                engine.config['sensors'].update(width=1536,height=1536,imgsz=1536,horizontal_fov_deg=70.)
                def operation(action,timeout=250):
                    r=engine.request(dict(action=action));assert r['ok'],r;deadline=time.monotonic()+timeout
                    while time.monotonic()<deadline:
                        op=next(v for v in engine.status()['operations'] if v['id']==r['data']['operation_id'])
                        if op['state']=='done':return
                        if op['state'] in ('failed','cancelled'):raise RuntimeError(op['message'])
                        time.sleep(.2)
                    raise TimeoutError(action)
                print('START '+split,flush=True);operation('start_ue');operation('start_ros')
                rpc=engine.rpc(3.)
                for name in ['Drone1']+[v['name'] for v in agents]:rpc.call('enableApiControl',True,name)
                # Freeze physics during pose changes; resume for captures so UE truth/RGB share timestamps.
                for name in ('front_left','front_right'):
                    prefix='Local\\AirSim_Drone1_'+name+'_1536x1536_'
                    readers.append((name,SharedMemoryCameraReader(prefix+'0',1536,1536,8),SharedMemoryCameraReader(prefix+'5',1536,1536,8)))
                for d in ('images','labels','masks','raw_masks'):(out/d/split).mkdir(parents=True,exist_ok=True)
                existing=list((out/'labels'/split).glob('*.txt'))
                saved=len(existing);resume_count=saved;objects=sum(len(v.read_text().splitlines()) for v in existing);negative=sum(not v.read_text().strip() for v in existing)
                rng=np.random.default_rng([181120,291121,401123][si]+saved);last={};records=[]
                frames=(out/(split+'.jsonl')).open('a',encoding='utf8')
                deadline=time.monotonic()+max(240,count*1.7)
                while saved<count and time.monotonic()<deadline:
                    # New independent scene every capture pair: angles, ranges, orientations and backgrounds.
                    base=np.array([rng.uniform(-15,15),rng.uniform(-10,35),-rng.uniform(7,30)])
                    heading=float(rng.uniform(-math.pi,math.pi));poses=[];is_negative=rng.random()<.12
                    def place(name,position,yaw,roll=0.,pitch=0.):
                        cr,sr=math.cos(roll/2),math.sin(roll/2);cp,sp=math.cos(pitch/2),math.sin(pitch/2);cy,sy=math.cos(yaw/2),math.sin(yaw/2)
                        zero=dict(x_val=0.,y_val=0.,z_val=0.)
                        rpc.call('simSetKinematics',dict(position=dict(zip(('x_val','y_val','z_val'),map(float,position))),orientation=dict(w_val=cr*cp*cy+sr*sp*sy,x_val=sr*cp*cy-cr*sp*sy,y_val=cr*sp*cy+sr*cp*sy,z_val=cr*cp*sy-sr*sp*cy),linear_velocity=zero,angular_velocity=zero,linear_acceleration=zero,angular_acceleration=zero),True,name)
                    rpc.call('simPause',True)
                    try:
                        place('Drone1',base,heading)
                        for i,agent in enumerate(agents):
                            distance=float(rng.choice([3.,5.,7.,9.,12.,17.,24.,35.,48.])*rng.uniform(.88,1.12))
                            bearing=float(rng.uniform(-58,58));elevation=float(rng.uniform(-24,24))
                            if is_negative:distance=150.;bearing=150.
                            angle=heading+math.radians(bearing)
                            pos=base+np.array([distance*math.cos(angle),distance*math.sin(angle),-distance*math.tan(math.radians(elevation))])
                            pos[2]=min(-1.5,pos[2]);yaw=float(rng.uniform(-math.pi,math.pi))
                            place(agent['name'],pos,yaw,float(rng.uniform(-.2,.2)),float(rng.uniform(-.2,.2)))
                            poses.append(dict(name=agent['name'],ned=pos.tolist(),distance_m=distance,bearing_deg=bearing,yaw_rad=yaw))
                    finally:rpc.call('simPause',False)
                    time.sleep(.18)
                    for name,rgb,sem in readers:
                        try:f=rgb.read_latest()
                        except RuntimeError:continue
                        if f.timestamp_ns==last.get(name):continue
                        sf=None
                        for j in range(sem.latest_frame_id(),max(0,sem.latest_frame_id()-8),-1):
                            try:v=sem._try_read_frame(j)
                            except RuntimeError:continue
                            if v and v.timestamp_ns==f.timestamp_ns:sf=v;break
                        if sf is None:continue
                        raw=np.frombuffer(sf.bgr_bytes,np.uint8).reshape(1536,1536,3);mask=decoder(raw)
                        if np.mean(mask==255)>.01:continue
                        im=np.frombuffer(f.bgr_bytes,np.uint8).reshape(1536,1536,3)
                        boxes,_,detail=labels_from_mask(mask,dict(class_ids=[6],min_area_px=8,polygon_epsilon_px=1.))
                        stem=f'{split}_{name}_{f.timestamp_ns}';last[name]=f.timestamp_ns
                        cv2.imwrite(str(out/'images'/split/(stem+'.jpg')),im,[cv2.IMWRITE_JPEG_QUALITY,95])
                        cv2.imwrite(str(out/'masks'/split/(stem+'.png')),mask);cv2.imwrite(str(out/'raw_masks'/split/(stem+'.png')),raw)
                        (out/'labels'/split/(stem+'.txt')).write_text('\n'.join(boxes),encoding='utf8')
                        frames.write(json.dumps(dict(stem=stem,stamp_ns=f.timestamp_ns,view=name,ego_ned=base.tolist(),ego_yaw_rad=heading,targets=poses,**detail))+'\n');frames.flush()
                        saved+=1;objects+=len(boxes);negative+=int(not boxes)
                    if saved%40<2:print(json.dumps(dict(split=split,frames=saved,objects=objects,negative=negative)),flush=True)
                frames.close()
                assert saved>=count and objects>0,(split,saved,objects)
                report['sessions'].append(dict(split=split,seed=[181120,291121,401123][si]+resume_count,resumed_initial_frames=resume_count,frames=saved,objects=objects,negative=negative,run=str(engine.active)))
                (out/'collection_report.json').write_text(json.dumps(report,indent=2),encoding='utf8')
            finally:
                for _,r,s in readers:r.close();s.close()
                if rpc:
                    try:rpc.call('simPause',False)
                    except Exception:pass
                    rpc.close()
                engine.close();done.set();thread.join(3);executor.shutdown();bridge.destroy_node()
        (out/'dataset.yaml').write_text(yaml.safe_dump(dict(path=str(out),train='images/train',val='images/val',test='images/test',names={0:'drone'})),encoding='utf8')
        report['complete']=True
    finally:
        if prior is None:origin.unlink(missing_ok=True)
        else:origin.write_bytes(prior)
        report['user_config_unchanged']=hashlib.sha256(cfg.read_bytes()).hexdigest()==before
        (out/'collection_report.json').write_text(json.dumps(report,indent=2),encoding='utf8')
        if rclpy.ok():rclpy.shutdown()
    print(json.dumps(report),flush=True)

if __name__=='__main__':main()
