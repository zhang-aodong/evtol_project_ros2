"""Independent target truth publication and explicitly geometric error audit."""
import argparse
from collections import defaultdict,deque
from concurrent.futures import ThreadPoolExecutor
from std_msgs.msg import String
import json
import threading
import time
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from evtol_simulation.rpc import AirSimRPC,image_request
from evtol_simulation.coordinates import NED_ENU,vector
from evtol_perception_interfaces.msg import TargetArray,Target,ReferenceOrigin
from .common import args_config,project_path,ns,header,stamp,xyz,put,LATCHED,Health


class TargetTruth(Node):
    def __init__(self,c):
        super().__init__('evaluation_target_truth');self.c=c;self.origin=None;self.stop=threading.Event()
        self.health=Health(self,'/evaluation/target_truth');self.lock=threading.Lock()
        actors=yaml.safe_load(project_path(c,c['actors_config']).read_text(encoding='utf-8-sig'))
        self.agents=actors['interference_drones']['agents'] if actors['interference_drones']['enabled'] else []
        self.status_pub=self.create_publisher(String,'/simulation/truth/status',10)
        self.pub=self.create_publisher(TargetArray,'/simulation/truth/targets',qos_profile_sensor_data)
        self.create_subscription(ReferenceOrigin,'/simulation/reference_origin',self.receive_origin,LATCHED)
        self.worker=threading.Thread(target=self.run,daemon=True);self.worker.start()

    def receive_origin(self,msg):
        with self.lock:self.origin=msg

    def run(self):
        c=self.c['airsim'];local=threading.local();clients=[];client_lock=threading.Lock()
        def sample(item):
            index,a,origin=item
            if not hasattr(local,'rpc'):
                local.rpc=AirSimRPC(c['host'],c['port'],.3)
                with client_lock:clients.append(local.rpc)
            rpc=local.rpc;name=a['vehicle_name']
            try:
                kin=rpc.call('simGetGroundTruthKinematics',name)
                offset=np.asarray(a['spawn_offset_ned_m']) if self.c['evaluation']['vehicle_truth_frame']=='vehicle_local' else np.zeros(3)
                position=NED_ENU@(vector(kin['position'])+offset)-xyz(origin.source_origin_enu)
                velocity=NED_ENU@vector(kin['linear_velocity'])
                if not np.isfinite(position).all() or not np.isfinite(velocity).all():raise ValueError('Nonfinite target truth')
                target=Target(track_id=index,class_id=6,reference_kind='vehicle_origin:'+name,
                    state='truth',measured=True)
                put(target.position,position);put(target.velocity,velocity)
                return target,dict(name=name,valid=True,
                    orientation_ned_frd=kin.get('orientation'))
            except Exception as exc:return None,dict(name=name,valid=False,reason=str(exc))
        pool=ThreadPoolExecutor(max_workers=min(8,max(1,len(self.agents))))
        collision_rpc=AirSimRPC(c['host'],c['port'],.3);collision_state=[None]
        def collisions():
            rpc=AirSimRPC(c['host'],c['port'],.3)
            try:
                while not self.stop.wait(.2):
                    try:collision_state[0]=dict(valid=True,**rpc.call('simGetCollisionInfo',c['vehicle']))
                    except Exception as exc:collision_state[0]=dict(valid=False,reason=str(exc))
            finally:rpc.close()
        collision_worker=threading.Thread(target=collisions,daemon=True);collision_worker.start()
        try:
            while not self.stop.is_set():
                begun=time.perf_counter()
                with self.lock:origin=self.origin
                if origin:
                    result=TargetArray(session_id=origin.session_id,status='TRUTH_PER_TARGET_TIMESTAMPS')
                    statuses=[]
                    try:
                        if self.c['evaluation'].get('batched_truth',False):
                            responses=collision_rpc.call('simGetImages',[image_request('evaluation_truth:'+a['vehicle_name'],0) for a in self.agents],c['vehicle'],False) if self.agents else []
                            if len(responses)!=len(self.agents):raise ValueError('Incomplete native truth batch')
                            for index,(a,r) in enumerate(zip(self.agents,responses),1):
                                parts=r.get('message','').split('|')
                                if len(parts)!=3 or parts[0]!='evaluation_truth_v1':raise ValueError('Native truth batch plugin missing')
                                before,after=map(int,parts[1:]);values=np.asarray(r['image_data_float'],float)
                                if values.shape!=(13,) or not np.isfinite(values).all() or not 0<=after-before<=50_000_000:
                                    raise ValueError('Invalid native target truth sample')
                                at=int(r['time_stamp']);offset=np.asarray(a['spawn_offset_ned_m']) if self.c['evaluation']['vehicle_truth_frame']=='vehicle_local' else np.zeros(3)
                                target=Target(track_id=index,class_id=6,reference_kind='vehicle_origin:'+a['vehicle_name'],state='truth',measured=True,last_measurement_stamp=stamp(at))
                                put(target.position,NED_ENU@(values[:3]+offset)-xyz(origin.source_origin_enu));put(target.velocity,NED_ENU@values[7:10])
                                result.targets.append(target);statuses.append(dict(name=a['vehicle_name'],stamp_ns=at,valid=True,bracket_ms=(after-before)*1e-6,time_basis='native_per_vehicle_bracket',orientation_ned_frd=values[3:7].tolist()))
                        else:
                            self.sample_legacy_batch(pool,sample,origin,collision_rpc,result,statuses)
                    except Exception as exc:
                        result.targets=[];statuses=[dict(name=a['vehicle_name'],valid=False,reason=str(exc)) for a in self.agents]
                    times=[ns(t.last_measurement_stamp) for t in result.targets]
                    result.header=header(max(times) if times else self.get_clock().now().nanoseconds)
                    self.pub.publish(result)
                    self.status_pub.publish(String(data=json.dumps(dict(schema_version=1,session_id=origin.session_id,
                        expected=len(self.agents),known=len(times),valid=len(times)==len(self.agents),targets=statuses,
                        ego_collision=collision_state[0],
                        sample_span_ms=(max(times)-min(times))*1e-6 if times else None,
                        cycle_wall_ms=(time.perf_counter()-begun)*1000))))
                    if len(times)==len(self.agents):self.health.ok()
                    else:self.health.error('Incomplete target truth')
                sample_hz=self.c['evaluation'].get('truth_sample_hz',self.c['evaluation']['truth_hz'])
                self.stop.wait(max(0.,1./sample_hz-(time.perf_counter()-begun)))
        finally:
            self.stop.set();collision_worker.join(timeout=1.)
            pool.shutdown(wait=True,cancel_futures=True)
            collision_rpc.close()
            for rpc in clients:rpc.close()

    def sample_legacy_batch(self,pool,sample,origin,collision_rpc,result,statuses):
        c=self.c['airsim']
        before=int(collision_rpc.call('getMultirotorState',c['vehicle'])['timestamp'])
        sampled=list(pool.map(sample,[(i,a,origin) for i,a in enumerate(self.agents,1)]))
        after=int(collision_rpc.call('getMultirotorState',c['vehicle'])['timestamp'])
        if after<before or after-before>50_000_000:raise ValueError('Wide shared truth bracket')
        t=(before+after)//2
        for target,status in sampled:
            status.update(stamp_ns=t,bracket_ms=(after-before)*1e-6,time_basis='shared_bracket_midpoint')
            statuses.append(status)
            if target is not None:
                target.last_measurement_stamp=stamp(t);result.targets.append(target)

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
            from .experiment import gated_assignment
            pairs=gated_assignment([dict(position=xyz(d.position),class_id=6) for d in msg.targets],
                [dict(position=truth[k],class_id=6) for k in keys],self.c['evaluation']['association_gate_m'])
            matches={i:(keys[j],error) for i,(j,error) in pairs.items()}
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
