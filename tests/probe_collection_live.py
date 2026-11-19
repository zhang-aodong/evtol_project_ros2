"""Opt-in real UE collection acceptance; owns and cleans only its own stack."""
import argparse
import hashlib
import json
from pathlib import Path
import threading
import time
import cv2
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from evtol_console.engine import Engine
from evtol_console.server import ConsoleBridge,ros_stop

def main():
    p=argparse.ArgumentParser();p.add_argument('--root',required=True);a=p.parse_args();root=Path(a.root)
    rclpy.init(domain_id=223);engine=Engine(root,ros_stop=ros_stop);bridge=None;executor=None;thread=None;finished=threading.Event()
    user=root/'config/console.local.json';before=hashlib.sha256(user.read_bytes()).hexdigest()
    origin=root/'data/session_origin.json';prior=origin.read_bytes() if origin.exists() else None
    report=dict(result='FAIL');folders=[]
    try:
        engine.config['ue']['visible']=False
        engine.config['actors'].update(count=2,initialization='first_waypoint',pedestrians_enabled=False,agents=[
            dict(name='Drone2',speed_mps=.6,spawn_ned_m=[15.,-3.,0.],waypoints_enu_m=[[-3.,15.,3.],[3.,15.,3.]],repeat=0),
            dict(name='Drone3',speed_mps=.6,spawn_ned_m=[22.,4.,0.],waypoints_enu_m=[[4.,22.,4.],[-4.,22.,4.]],repeat=0)])
        engine.config['run'].update(execute=False,actors=True,evaluate=True,new_session=True)
        bridge=ConsoleBridge(engine,'drone1',0);executor=SingleThreadedExecutor();executor.add_node(bridge)
        def spin():
            while not finished.is_set():executor.spin_once(timeout_sec=.05)
        thread=threading.Thread(target=spin,daemon=True);thread.start()
        def operation(action,payload=None,timeout=150):
            response=engine.request(dict(action=action,payload=payload or {}));assert response['ok'],response
            identifier=response['data']['operation_id'];end=time.monotonic()+timeout
            while time.monotonic()<end:
                status=engine.status();op=next(o for o in status['operations'] if o['id']==identifier)
                if op['state']=='done':return
                if op['state'] in ('failed','cancelled'):raise RuntimeError(action+': '+op['message'])
                time.sleep(.2)
            raise TimeoutError(action)
        operation('start_ue');operation('start_ros',timeout=240)
        time.sleep(3)
        options=engine.collection_config
        options['output_dir']='data/collections_acceptance'
        options['dataset'].update(class_ids=[6],rate_hz=3.)
        operation('start_collection',dict(mode='dataset',config=options),timeout=40)
        time.sleep(7);operation('stop_collection',timeout=40)
        folder=engine.collection_folder;folders.append(str(folder));value=json.loads((folder/'collection.json').read_text())
        assert value['complete'] and value['dataset_frames']>=8,value
        assert value['dataset_objects']>0,'UE truth contained no drone labels'
        images=list((folder/'semantic/images/train').glob('*.jpg'));masks=list((folder/'semantic/masks/train').glob('*.png'))
        assert len(images)==len(masks)==value['dataset_frames']
        first=next((p for p in masks if np.count_nonzero(cv2.imdecode(np.frombuffer(p.read_bytes(),np.uint8),0)==6)>0),None)
        assert first is not None
        rgb=cv2.imdecode(np.frombuffer((folder/'semantic/images/train'/(first.stem+'.jpg')).read_bytes(),np.uint8),1)
        mask=cv2.imdecode(np.frombuffer(first.read_bytes(),np.uint8),0)
        preview=rgb.copy();preview[mask==6]=(preview[mask==6]*.4+np.array([0,0,255])*.6).astype(np.uint8)
        labels=(folder/'detection/labels/train'/(first.stem+'.txt')).read_text().splitlines()
        for line in labels:
            _,cx,cy,w,h=map(float,line.split());ih,iw=mask.shape
            cv2.rectangle(preview,(int((cx-w/2)*iw),int((cy-h/2)*ih)),(int((cx+w/2)*iw),int((cy+h/2)*ih)),(0,255,0),2)
        cv2.imencode('.jpg',preview)[1].tofile(str(root/'reports/collection_truth_preview.jpg'))
        report['dataset']=value
        options['bag'].update(topics=['/clock','/tf_static','/drone1/imu/data','/drone1/camera/rgb/image_realtime'],split_size_mb=16)
        operation('start_collection',dict(mode='bag',config=options),timeout=40)
        time.sleep(4);operation('stop_collection',timeout=40)
        folder=engine.collection_folder;folders.append(str(folder));value=json.loads((folder/'collection.json').read_text())
        assert value['complete'] and all(value['bag_messages'].get(t,0)>0 for t in options['bag']['topics']),value
        import rosbag2_py
        reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(folder/'rosbag2'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
        assert {t.name for t in reader.get_all_topics_and_types()}==set(options['bag']['topics'])
        total=0
        while reader.has_next():reader.read_next();total+=1
        del reader
        assert total==sum(value['bag_messages'].values()),(total,value['bag_messages'])
        report.update(bag=value,readback_messages=total,result='PASS',folders=folders)
    except BaseException as exc:report['error']=str(exc);raise
    finally:
        engine.close();finished.set()
        if thread:thread.join(timeout=3)
        if executor:executor.shutdown()
        if bridge:bridge.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        if prior is None:origin.unlink(missing_ok=True)
        else:origin.write_bytes(prior)
        report.update(user_config_unchanged=hashlib.sha256(user.read_bytes()).hexdigest()==before,remaining_owned_processes=list(engine.processes))
        (root/'reports/collection_live_verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
        print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)

if __name__=='__main__':main()
