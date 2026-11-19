"""Independent target truth publication and explicitly geometric error audit."""
import argparse
from collections import defaultdict,deque
import json
import threading
import time
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from evtol_simulation.rpc import AirSimRPC
from evtol_simulation.coordinates import NED_ENU,vector
from evtol_perception_interfaces.msg import TargetArray,Target,ReferenceOrigin
from .common import args_config,project_path,ns,header,stamp,xyz,put,LATCHED,Health


class TargetTruth(Node):
    def __init__(self,c):
        super().__init__('evaluation_target_truth');self.c=c;self.origin=None;self.stop=threading.Event()
        self.health=Health(self,'/evaluation/target_truth');self.lock=threading.Lock()
        actors=yaml.safe_load(project_path(c,c['actors_config']).read_text(encoding='utf-8-sig'))
        self.agents=actors['interference_drones']['agents']
        self.pub=self.create_publisher(TargetArray,'/simulation/truth/targets',qos_profile_sensor_data)
        self.create_subscription(ReferenceOrigin,'/simulation/reference_origin',self.receive_origin,LATCHED)
        self.worker=threading.Thread(target=self.run,daemon=True);self.worker.start()

    def receive_origin(self,msg):
        with self.lock:self.origin=msg

    def run(self):
        c=self.c['airsim'];rpc=AirSimRPC(c['host'],c['port'],2.)
        try:
            while not self.stop.is_set():
                begun=time.perf_counter()
                with self.lock:origin=self.origin
                if origin:
                    result=TargetArray(session_id=origin.session_id,status='TRUTH_VEHICLE_ORIGIN')
                    for index,a in enumerate(self.agents,1):
                        try:
                            name=a['vehicle_name']
                            before=int(rpc.call('getMultirotorState',name)['timestamp'])
                            kin=rpc.call('simGetGroundTruthKinematics',name)
                            after=int(rpc.call('getMultirotorState',name)['timestamp'])
                            if after<before or after-before>50_000_000:raise ValueError('Wide truth bracket')
                            t=(before+after)//2
                            offset=np.asarray(a['spawn_offset_ned_m']) if self.c['evaluation']['vehicle_truth_frame']=='vehicle_local' else np.zeros(3)
                            position=NED_ENU@(vector(kin['position'])+offset)-xyz(origin.source_origin_enu)
                            target=Target(track_id=index,class_id=6,reference_kind='vehicle_origin:'+name,
                                state='truth',measured=True,last_measurement_stamp=stamp(t))
                            put(target.position,position);put(target.velocity,NED_ENU@vector(kin['linear_velocity']))
                            result.targets.append(target);result.header=header(t)
                        except Exception as exc:self.health.error(str(a['vehicle_name'])+': '+str(exc))
                    if result.targets:self.pub.publish(result);self.health.ok()
                self.stop.wait(max(0.,1./self.c['evaluation']['truth_hz']-(time.perf_counter()-begun)))
        finally:rpc.close()

    def close(self):self.stop.set();self.worker.join(timeout=5.)


class Evaluator(Node):
    def __init__(self,c):
        super().__init__('target_evaluator');self.c=c;self.history=defaultdict(lambda:deque(maxlen=200));self.session=''
        output=project_path(c,c['evaluation']['output_dir'])/time.strftime('%Y%m%d_%H%M%S')
        output.mkdir(parents=True,exist_ok=True);self.file=(output/'target_errors.jsonl').open('w',encoding='utf8')
        self.health=Health(self,'/evaluation/geometric_errors');self.pending=deque(maxlen=100)
        self.create_subscription(TargetArray,'/simulation/truth/targets',self.truth,qos_profile_sensor_data)
        self.create_subscription(TargetArray,'/'+c['namespace']+'/perception/targets_measurements',self.receive,qos_profile_sensor_data)
        self.create_timer(.05,self.process)

    def truth(self,msg):
        if self.session!=msg.session_id:self.history.clear();self.pending.clear();self.session=msg.session_id
        for target in msg.targets:self.history[target.track_id].append((ns(target.last_measurement_stamp),xyz(target.position)))

    def receive(self,msg):
        if len(self.pending)==self.pending.maxlen:self.health.dropped+=1
        self.pending.append((time.perf_counter(),msg))

    def process(self):
        if not self.pending:return
        begun,msg=self.pending[0];t=ns(msg.lidar_stamp)
        truth={}
        if msg.session_id==self.session:
            for key,hist in self.history.items():
                for (ta,pa),(tb,pb) in zip(hist,list(hist)[1:]):
                    if ta<=t<=tb and tb-ta<=150_000_000:
                        truth[key]=pa+(pb-pa)*(t-ta)/max(1,tb-ta);break
        if not truth and time.perf_counter()-begun<.3:return
        self.pending.popleft()
        from scipy.optimize import linear_sum_assignment
        keys=list(truth);matches={}
        if msg.targets and keys:
            distances=np.linalg.norm(np.asarray([xyz(d.position) for d in msg.targets])[:,None,:]-np.asarray([truth[k] for k in keys])[None,:,:],axis=2)
            rows,cols=linear_sum_assignment(distances)
            matches={int(i):(keys[j],float(distances[i,j])) for i,j in zip(rows,cols)}
        record=dict(stamp_ns=t,session_id=msg.session_id,status=msg.status,
            evaluation_basis='geometric_only_surface_to_vehicle_origin_not_identity_verified',
            truth_available=len(truth),detections=[],unmatched_truth=list(keys))
        for i,detection in enumerate(msg.targets):
            candidate=matches.get(i)
            matched=candidate is not None and candidate[1]<=self.c['evaluation']['association_gate_m']
            record['detections'].append(dict(track_id=detection.track_id,class_id=detection.class_id,
                position_enu_m=xyz(detection.position).tolist(),reference_kind=detection.reference_kind,
                matched=matched,candidate_truth_id=candidate[0] if candidate else None,
                candidate_error_m=candidate[1] if candidate else None))
            if matched:record['unmatched_truth'].remove(candidate[0])
        self.file.write(json.dumps(record,ensure_ascii=False)+'\n');self.file.flush();self.health.ok()

    def close(self):self.file.close()


def main(args=None):
    p=argparse.ArgumentParser(add_help=False);p.add_argument('--truth',action='store_true');opts,remaining=p.parse_known_args(args)
    c,ros=args_config(remaining);rclpy.init(args=ros);node=TargetTruth(c) if opts.truth else Evaluator(c)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.close();node.destroy_node();rclpy.shutdown()
