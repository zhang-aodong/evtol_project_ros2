"""Fixed-rate independent evaluation, JSON v1 metrics, markers and durable logs."""
from collections import deque
import json
from pathlib import Path
import time
import uuid
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Point
from evtol_perception_interfaces.msg import ReferenceOrigin,TargetArray,Risk
from evtol_simulation.coordinates import extrinsic,quaternion_matrix
from .common import args_config,project_path,ns,xyz,header,put,LATCHED,Health
from .experiment import RangeAudit,DEFAULTS
from .warning_audit import WarningAudit
import yaml


class ExperimentEvaluator(Node):
    def __init__(self,c):
        super().__init__('range_experiment_evaluator');self.c=c;self.o={**DEFAULTS,**c.get('experiment',{})}
        actors=yaml.safe_load(project_path(c,c['actors_config']).read_text(encoding='utf-8-sig'))
        self.expected=[i+1 for i,_ in enumerate(actors['interference_drones']['agents'])] if actors['interference_drones']['enabled'] else []
        self.names={i+1:a['vehicle_name'] for i,a in enumerate(actors['interference_drones']['agents'])}
        self.audit=RangeAudit(self.expected,self.o);self.alarm_audit=RangeAudit(self.expected,self.o)
        self.warning_pending=deque(maxlen=128);self.lidar_warning=None;self.fusion_warning=None;self.hemisphere_warning=None
        self.origin=None;self.pending=deque(maxlen=128);self.poses=deque(maxlen=400)
        self.last_clock=None;self.last_tick=None;self.last_publish=0.;self.last_event=0;self.last=None;self.segment=0
        self.camera=None;self.actors=None;self.warning=None;self.latest_risks=None;self.latest_risks_wall=0.
        self.flight=None;self.flight_wall=0.
        self.truth_status=None;self.truth_status_wall=0.;self.collisions=set();self.scoring_started=None
        self.warning_audit=WarningAudit(self.o['warning_events'])
        self.risk_history=deque(maxlen=128)
        self.pipeline=dict(frames=0,late_frames=0,matched_frame_targets=0,last_latency_ms=None)
        self.health=Health(self,'/evaluation/range');self.wall_clock=Clock(clock_type=ClockType.STEADY_TIME)
        output=project_path(c,c['evaluation']['output_dir'])/('range_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
        output.mkdir(parents=True,exist_ok=True);self.output=output
        self.files={k:(output/(k+'.jsonl')).open('a',encoding='utf8') for k in ('metrics','matches','encounters','warnings')}
        self.pub=self.create_publisher(String,'/evaluation/detection_metrics',10)
        self.matches_pub=self.create_publisher(String,'/evaluation/target_matches',10)
        self.events_pub=self.create_publisher(String,'/evaluation/encounter_events',10)
        self.warning_pub=self.create_publisher(String,'/evaluation/warning_metrics',10)
        self.markers_pub=self.create_publisher(MarkerArray,'/evaluation/markers',qos_profile_sensor_data)
        root='/'+c['namespace']
        self.create_subscription(ReferenceOrigin,'/simulation/reference_origin',self.receive_origin,LATCHED)
        # Preserve actual interpolation samples across short evaluator stalls.
        # This does not extrapolate truth, widen the gap gate or retain evidence.
        truth_qos=QoSProfile(depth=64,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(TargetArray,'/simulation/truth/targets',self.receive_truth,truth_qos)
        self.create_subscription(Odometry,root+'/state/odometry',self.receive_ego,truth_qos)
        self.create_subscription(TargetArray,root+'/perception/targets_measurements',self.receive_measurements,qos_profile_sensor_data)
        self.create_subscription(CameraInfo,root+'/camera/rgb/camera_info',lambda m:setattr(self,'camera',m),qos_profile_sensor_data)
        self.create_subscription(String,'/scenario/status',lambda m:setattr(self,'actors',json.loads(m.data)),LATCHED)
        self.create_subscription(String,root+'/safety/risks',self.receive_risks,10)
        self.create_subscription(String,root+'/warning/fusion',self.receive_fusion_warning,10)
        self.create_subscription(String,root+'/warning/lidar',lambda m:setattr(self,'lidar_warning',json.loads(m.data)),10)
        self.create_subscription(String,root+'/control/flight_status',self.receive_flight,LATCHED)
        self.create_subscription(String,'/simulation/truth/status',self.receive_truth_status,10)
        self.create_timer(.025,self.tick,clock=self.wall_clock)
        (output/'configuration.json').write_text(json.dumps(dict(experiment=self.o,expected=self.names),ensure_ascii=False,indent=2),encoding='utf8')

    def receive_risks(self,msg):
        try:
            value=json.loads(msg.data)
            if not self.origin or value.get('session_id')!=self.origin.session_id:return
            self.latest_risks=value;self.latest_risks_wall=time.monotonic();self.risk_history.append(value)
        except ValueError:pass

    def receive_fusion_warning(self,msg):
        try:
            value=json.loads(msg.data)
            if self.origin and value.get('session_id')==self.origin.session_id:
                self.fusion_warning=value
                self.warning_pending.append((self.get_clock().now().nanoseconds*1e-9,value))
        except (ValueError,KeyError):pass

    def receive_flight(self,msg):
        try:
            value=json.loads(msg.data)
            if self.origin and value.get('session_id')==self.origin.session_id:
                self.flight=value;self.flight_wall=time.monotonic()
        except ValueError:pass

    def receive_truth_status(self,msg):
        try:
            value=json.loads(msg.data)
            if self.origin and value.get('session_id')==self.origin.session_id:
                self.truth_status=value;self.truth_status_wall=time.monotonic()
        except ValueError:pass

    def reset(self,session):
        self.segment+=1
        self.audit=RangeAudit(self.expected,self.o);self.audit.session=session
        self.alarm_audit=RangeAudit(self.expected,self.o);self.alarm_audit.session=session
        self.warning_pending.clear();self.lidar_warning=None;self.fusion_warning=None;self.hemisphere_warning=None
        self.pending.clear();self.poses.clear();self.last_tick=None;self.last_event=0;self.last=None
        self.latest_risks=None;self.flight=None;self.flight_wall=0.
        self.truth_status=None;self.collisions=set();self.scoring_started=None
        self.warning_audit=WarningAudit(self.o['warning_events'])
        self.risk_history.clear()
        self.pipeline=dict(frames=0,late_frames=0,matched_frame_targets=0,last_latency_ms=None)

    def receive_origin(self,msg):
        if self.origin is None or msg.session_id!=self.origin.session_id:self.reset(msg.session_id)
        self.origin=msg

    def receive_ego(self,msg):
        if self.origin is None or msg.header.frame_id!='map_enu':return
        t=ns(msg.header.stamp)*1e-9
        q=msg.pose.pose.orientation;r=quaternion_matrix([q.x,q.y,q.z,q.w])
        # Odometry twist is expressed in the child frame.
        for audit in (self.audit,self.alarm_audit):
            audit.add_ego(t,xyz(msg.pose.pose.position),r@xyz(msg.twist.twist.linear),r[:,0])
        self.poses.append((t,xyz(msg.pose.pose.position),r))

    def receive_truth(self,msg):
        if self.origin is None or msg.session_id!=self.origin.session_id or msg.header.frame_id!='map_enu':return
        for target in msg.targets:
            for audit in (self.audit,self.alarm_audit):
                audit.add_truth(int(target.track_id),ns(target.last_measurement_stamp)*1e-9,xyz(target.position),xyz(target.velocity))

    def receive_measurements(self,msg):
        if self.origin is None or msg.session_id!=self.origin.session_id or msg.header.frame_id!='map_enu':return
        if len(self.pending)==self.pending.maxlen:self.health.dropped+=1
        self.pending.append((self.get_clock().now().nanoseconds*1e-9,msg))

    def visible(self,identity,position):
        if self.c.get('surround',{}).get('enabled',False) and self.poses:
            from .surround_geometry import optical_pose,intrinsics
            s=self.c['surround'];t=self.audit.last_t;pose=min(self.poses,key=lambda p:abs(p[0]-t))
            if abs(pose[0]-t)>self.o['truth_max_gap_s']:return False
            body=(position-pose[1])@pose[2];k=intrinsics(s)
            for view in s['cameras']:
                offset,q=optical_pose(view);optical=(body-offset)@quaternion_matrix(q)
                if optical[2]<=0:continue
                uv=k@optical;u,v=uv[:2]/uv[2]
                if 0<=u<s['width'] and 0<=v<s['height']:return True
            return False
        if not self.camera or not self.poses:return False
        t=self.audit.last_t;pose=min(self.poses,key=lambda p:abs(p[0]-t))
        if abs(pose[0]-t)>self.o['truth_max_gap_s']:return False
        cc=self.c['extrinsics']['camera'];tc,qc=extrinsic(cc['xyz_frd_m'],cc['rpy_deg'])
        optical=quaternion_matrix(qc)@quaternion_matrix([-.5,.5,-.5,.5])
        body=(position-pose[1])@pose[2]
        p=(body-tc)@optical
        if p[2]<=0:return False
        uv=np.asarray(self.camera.k).reshape(3,3)@p;u,v=uv[:2]/uv[2]
        lc=self.c['extrinsics']['lidar'];tl,ql=extrinsic(lc['xyz_frd_m'],lc['rpy_deg'])
        pl=(body-tl)@quaternion_matrix(ql)
        yaw=abs(np.degrees(np.arctan2(pl[1],pl[0])));pitch=abs(np.degrees(np.arctan2(pl[2],np.hypot(pl[0],pl[1]))))
        return bool(0<=u<self.camera.width and 0<=v<self.camera.height and yaw<=90 and pitch<=15 and np.linalg.norm(pl)<=100)

    def emit(self,key,value,pub):
        data=json.dumps(value,ensure_ascii=False,allow_nan=False,separators=(',',':'))
        self.files[key].write(data+'\n');pub.publish(String(data=data))

    def tick(self):
        if self.origin is None:return
        now=self.get_clock().now().nanoseconds*1e-9
        if self.last_clock is not None and now<self.last_clock:self.reset(self.origin.session_id)
        self.last_clock=now
        # Process each independent scan once, wait boundedly for its truth bracket.
        for _ in range(len(self.pending)):
            available,msg=self.pending.popleft();t=ns(msg.lidar_stamp)*1e-9
            detections=[dict(track_id=int(d.track_id),class_id=int(d.class_id),position=xyz(d.position)) for d in msg.targets if d.measured]
            row=self.audit.measure(t,available,detections)
            if row is None:
                if now-t<=self.o['buffer_s']:self.pending.append((available,msg))
                else:self.health.dropped+=1
            else:
                self.pipeline['frames']+=1;self.pipeline['late_frames']+=int(row['late'])
                self.pipeline['matched_frame_targets']+=row['frame_tp']
                self.pipeline['last_latency_ms']=round((available-t)*1000,2)
                self.emit('matches',dict(session_id=self.audit.session,**row),self.matches_pub)
        t=float(np.floor((now-self.o['buffer_s'])*self.o['truth_hz'])/self.o['truth_hz'])
        if t<0 or (self.last_tick is not None and t<=self.last_tick):return
        skipped=max(0,round((t-self.last_tick)*self.o['truth_hz'])-1) if self.last_tick is not None else 0
        self.last_tick=t
        # No unbounded catch-up after stalls; missed epochs are explicit.
        scoring=not self.o['score_only_flying'] or bool(self.flight and self.flight.get('phase')=='flying' and time.monotonic()-self.flight_wall<2.)
        if scoring and self.scoring_started is None:self.scoring_started=t
        waiting=[]
        for available,warning in self.warning_pending:
            wt=warning['stamp_ns']*1e-9
            detections=[dict(track_id=d['track_id'],position=np.asarray(d['position_enu_m']),class_id=6)
                        for d in warning['targets'] if warning['valid'] and d['warning']]
            row=self.alarm_audit.measure(wt,available,detections)
            if row is None and now-wt<=self.o['buffer_s']:waiting.append((available,warning))
        self.warning_pending.clear();self.warning_pending.extend(waiting)
        self.hemisphere_warning=self.alarm_audit.snapshot(t,score=scoring)
        value=self.audit.snapshot(t,self.visible,score=scoring)
        if value is None:return
        value['evaluation_delay_s']=now-t;value['output_dir']=str(self.output)
        value['hemisphere_warning']=self.hemisphere_warning
        value['evaluation_segment']=self.segment
        value['pipeline']=dict(self.pipeline)
        value['skipped_epochs']=skipped
        value['actor_status']=self.actors
        value['flight_phase']=self.flight.get('phase') if self.flight else None
        collision=(self.truth_status or {}).get('ego_collision') or {}
        collision_fresh=time.monotonic()-self.truth_status_wall<1. and collision.get('valid',False)
        collision_stamp=int(collision.get('time_stamp',0))
        if scoring and collision_fresh and collision.get('has_collided') and collision_stamp*1e-9>=self.scoring_started:
            self.collisions.add(collision_stamp)
        value['collision_status_valid']=bool(collision_fresh)
        value['collision_count']=len(self.collisions)
        for row in value.get('targets',[]):row['name']=self.names.get(row['identity'],str(row['identity']))
        self.last=value
        if value['valid']:self.health.ok()
        else:self.health.error('Truth incomplete: '+str(value.get('missing')))
        for event in self.audit.events:
            if event.get('_emitted'):continue
            self.emit('encounters',dict(session_id=self.audit.session,**event),self.events_pub);event['_emitted']=True
        # Persist every evaluated epoch; only render at the configured display rate.
        self.files['metrics'].write(json.dumps(value,ensure_ascii=False,allow_nan=False)+'\n')
        if time.monotonic()-self.last_publish<1/self.o['publish_hz']:return
        self.last_publish=time.monotonic();self.pub.publish(String(data=json.dumps(value,ensure_ascii=False,allow_nan=False)))
        for f in self.files.values():f.flush()
        self.draw(value)
        self.publish_warning(value)
        summary=dict(schema_version=1,session_id=self.audit.session,last=value,hemisphere_warning=self.hemisphere_warning,ended=False)
        (self.output/'run_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf8')

    def publish_warning(self,value):
        # Diagnostic truth-CPA counts only. No circular claim of contract alarm accuracy.
        risks=[];ego=self.audit.at(self.audit.ego,value['stamp_s'])
        if value['valid'] and ego:
            for identity in self.expected:
                state=self.audit.at(self.audit.truth[identity],value['stamp_s'])
                r=state[0]-ego[0];v=state[1]-ego[1]
                tc=float(np.clip(-r@v/max(v@v,1e-12),0,self.o['risk_horizon_s']))
                distance=float(np.linalg.norm(r+v*tc))
                margin=self.o['ego_radius_m']+.5*np.linalg.norm(self.o['target_size_m'])+self.o['risk_clearance_m']
                if distance<margin:risks.append(dict(identity=identity,cpa_time_s=tc,cpa_center_distance_m=distance))
        algorithm=self.latest_risks if time.monotonic()-self.latest_risks_wall<1. else None
        labels=None
        if self.o['warning_events'] and self.scoring_started is not None:
            while self.risk_history:
                sample=self.risk_history.popleft();t=sample['stamp_ns']*1e-9
                mapping={}
                for row in sorted(self.audit.observations,key=lambda r:r['stamp_s']):
                    if row['late'] or not 0<=t-row['stamp_s']<=self.o['evidence_s']:continue
                    for identity,match in row['matches'].items():mapping[match['track_id']]=identity
                    for fp in row['false_positives']:mapping.pop(fp['track_id'],None)
                labels=self.warning_audit.observe(t-self.scoring_started,mapping,sample.get('targets',[]),sample['stamp_ns'],
                    valid=value['valid'] and sample.get('valid',False))
            if labels is None:
                labels=self.warning_audit.observe(value['stamp_s']-self.scoring_started,{},[],None,
                    valid=value['valid'] and algorithm is not None and algorithm.get('valid',False))
        self.emit('warnings',dict(schema_version=1,session_id=self.audit.session,stamp_s=value['stamp_s'],
            valid=value['valid'],basis='instantaneous_truth_cpa_diagnostic_not_counterfactual_acceptance',
            truth_risk_count=len(risks) if value['valid'] else None,truth_risks=risks,algorithm=algorithm,
            labeled_event_metrics=labels,
            contract_precision=labels['precision'] if labels else None,
            contract_false_alarm_rate=labels['false_alarm_rate'] if labels else None,
            hemisphere=self.hemisphere_warning,lidar=self.lidar_warning,fusion=self.fusion_warning,
            primary_metric='timely_warning_per_entry_into_forward_hemisphere',target_success_rate=self.c.get('warnings',{}).get('target_success_rate',.9),
            hemisphere_miss_rate=(1-self.hemisphere_warning['timely_success_rate']) if self.hemisphere_warning and self.hemisphere_warning.get('timely_success_rate') is not None else None),self.warning_pub)

    def draw(self,value):
        t=int(value['stamp_s']*1e9);markers=[Marker(action=Marker.DELETEALL)]
        ego=self.audit.at(self.audit.ego,value['stamp_s'])
        if ego:
            m=Marker(header=header(t),ns='evaluation_range',id=0,type=Marker.SPHERE,action=Marker.ADD)
            put(m.pose.position,ego[0]);m.pose.orientation.w=1.
            m.scale.x=m.scale.y=m.scale.z=2*self.o['radius_m'];m.color.b=1.;m.color.g=.5;m.color.a=.06
            if self.o['region']=='front_hemisphere' and value.get('forward_enu'):
                m.type=Marker.LINE_LIST;m.scale.x=.04;m.color.a=.6
                f=np.asarray(value['forward_enu']);axis=np.array([0.,0.,1.]) if abs(f[2])<.9 else np.array([0.,1.,0.])
                side=np.cross(f,axis);side/=np.linalg.norm(side);up=np.cross(side,f)
                def local(theta,phi):
                    return self.o['radius_m']*(np.cos(theta)*f+np.sin(theta)*(np.cos(phi)*side+np.sin(phi)*up))
                paths=[[(th,ph) for th in np.linspace(0,np.pi/2,19)] for ph in np.linspace(0,2*np.pi,13)]
                paths += [[(th,ph) for ph in np.linspace(0,2*np.pi,73)] for th in np.linspace(0,np.pi/2,7)]
                for path in paths:
                    for a,b in zip(path,path[1:]):
                        for theta,phi in (a,b):
                            point=Point();put(point,local(theta,phi));m.points.append(point)
            markers.append(m)
        for row in value.get('targets',[]):
            m=Marker(header=header(t),ns='evaluation_truth',id=int(row['identity']),type=Marker.CUBE,action=Marker.ADD)
            put(m.pose.position,row['position_enu_m']);m.pose.orientation.w=1.;put(m.scale,self.o['target_size_m'])
            m.color.g=1. if row['detected'] else .1;m.color.r=.1 if row['detected'] else 1.;m.color.a=.4 if row['in_range'] else .1
            m.lifetime.sec=1;markers.append(m)
        self.markers_pub.publish(MarkerArray(markers=markers))

    def close(self):
        summary=dict(schema_version=1,session_id=self.audit.session,last=self.last,hemisphere_warning=self.hemisphere_warning,ended=True,
            end_reason='node_shutdown',pending_encounters=sum(e['result']=='pending' for e in self.audit.active.values()))
        (self.output/'run_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf8')
        for f in self.files.values():f.close()


def main(args=None):
    c,ros=args_config(args);rclpy.init(args=ros);node=ExperimentEvaluator(c)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.close();node.destroy_node();rclpy.shutdown()
