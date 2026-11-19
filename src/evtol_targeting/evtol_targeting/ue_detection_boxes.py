"""Detection-derived AABB overlays in UE via AirSim's world plotting API.

Target positions come exclusively from detection/tracking messages.
The ego actor is read only to calibrate the plotting coordinate origin.
Input positions/sizes are ENU; simPlotLineList expects global NED metres.
"""
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock as RosClock, ClockType
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from evtol_perception_interfaces.msg import ReferenceOrigin, TargetArray
from evtol_simulation.rpc import AirSimRPC
from evtol_simulation.runtime import SensorLease
from .common import LATCHED, args_config, header, ns, xyz
from .ue_box_geometry import options, Box, box_edges_ned, to_global_ned, wire_point


class UeDetectionBoxes(Node):
    def __init__(self,c,start_worker=True):
        super().__init__('ue_detection_boxes')
        self.c=c;self.o=options(c);self.root='/'+c['namespace'].strip('/')
        self.lock=threading.Lock();self.stop=threading.Event();self.worker=None
        self.origin=None;self.session='';self.origin_stamp=0;self.epoch=0
        self.sim_ns=0;self.clock_wall=None;self.message_wall=None;self.message_ns=0;self.boxes=()
        self.draw_calls=0;self.last_box_count=0;self.errors=0;self.rejected=0
        self.last_error='';self.last_draw=None;self.spawn=None
        self.diagnostics=self.create_publisher(DiagnosticArray,'/diagnostics',10)
        self.create_subscription(ReferenceOrigin,'/simulation/reference_origin',self.receive_origin,LATCHED)
        self.create_subscription(Clock,'/clock',self.receive_clock,qos_profile_sensor_data)
        topic=self.o['input_topic']
        self.input_topic=topic if topic.startswith('/') else self.root+'/'+topic
        self.create_subscription(TargetArray,self.input_topic,self.receive_targets,qos_profile_sensor_data)
        self.steady=RosClock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.,self.report,clock=self.steady)
        if start_worker:
            self.worker=threading.Thread(target=self.run,name='ue_box_renderer',daemon=True);self.worker.start()
        self.get_logger().info('UE detection boxes: '+self.input_topic+' -> global NED plot, classes '+str(self.o['class_ids']))

    def receive_origin(self,msg):
        p=xyz(msg.source_origin_enu)
        if not msg.session_id or msg.header.frame_id!='map_enu' or not np.isfinite(p).all():return
        with self.lock:
            if msg.session_id!=self.session:
                self.epoch+=1;self.boxes=();self.message_wall=None;self.message_ns=0;self.spawn=None
            self.session=msg.session_id;self.origin=tuple(p);self.origin_stamp=ns(msg.header.stamp)

    def receive_clock(self,msg):
        value=ns(msg.clock)
        if value<=0:return
        with self.lock:
            if self.sim_ns and value<self.sim_ns:
                self.epoch+=1;self.boxes=();self.message_wall=None;self.message_ns=0;self.spawn=None
                self.session='';self.origin=None
            if value!=self.sim_ns:self.clock_wall=time.perf_counter()
            self.sim_ns=value

    def receive_targets(self,msg):
        t=ns(msg.header.stamp)
        with self.lock:
            if self.origin is None or msg.header.frame_id!='map_enu' or msg.session_id!=self.session or t<self.origin_stamp or t<=self.message_ns:
                self.rejected+=1;return
            selected=[]
            for target in msg.targets:
                if target.class_id not in self.o['class_ids'] or target.state not in ('tentative','confirmed','predicted'):continue
                p=xyz(target.position);size=xyz(target.size);measured=ns(target.last_measurement_stamp)
                if not np.isfinite(p).all() or not np.isfinite(size).all() or np.any(size<0) or not 0<measured<=t:
                    self.rejected+=1;continue
                selected.append(Box(target.track_id,tuple(p),tuple(np.maximum(size,self.o['min_size_enu_m'])+2*self.o['padding_m']),measured,target.state))
                if len(selected)>=self.o['max_boxes']:break
            self.boxes=tuple(selected);self.message_ns=t;self.message_wall=time.perf_counter()

    def snapshot(self,wall):
        with self.lock:
            if (self.origin is None or self.message_wall is None or self.clock_wall is None
                or wall-self.message_wall>self.o['message_timeout_s']
                or wall-self.clock_wall>self.o['clock_timeout_s']):return None
            age=(self.sim_ns-self.message_ns)*1e-9
            if age < -.1 or age>self.o['message_timeout_s']:return None
            boxes=tuple(b for b in self.boxes if 0<=self.sim_ns-b.last_measurement_ns<=self.o['measurement_timeout_s']*1e9)
            return self.epoch,self.origin,boxes

    def load_spawn(self,rpc):
        if self.o['source_coordinates']=='global_ned':return (0.,0.,0.)
        if self.o['global_ned_offset_m'] is not None:return tuple(self.o['global_ned_offset_m'])
        vehicle=self.c['airsim']['vehicle']
        actor=self.o['ego_actor_name'] or vehicle
        def position(pose):
            p=pose['position'];a=np.asarray([p[k] for k in ('x_val','y_val','z_val')],dtype=float)
            if not np.isfinite(a).all():raise ValueError('Cannot calibrate ego actor '+actor+'; set ego_actor_name or global_ned_offset_m')
            return a
        # Vehicle-local NED includes a pawn ground offset, absent from settings XYZ.
        # Bracket the global sample to bound motion error to <= 2.5 cm.
        for _ in range(3):
            if self.stop.is_set():raise RuntimeError('UE box renderer stopping')
            before=position(rpc.call('simGetVehiclePose',vehicle))
            world=position(rpc.call('simGetObjectPose',actor))
            after=position(rpc.call('simGetVehiclePose',vehicle))
            if np.linalg.norm(after-before)<=.05:
                return tuple(world-(before+after)/2)
        raise ValueError('Ego moved during plot-origin calibration; retrying (or configure global_ned_offset_m)')

    def render_once(self,rpc,wall=None):
        snapshot=self.snapshot(time.perf_counter() if wall is None else wall)
        if snapshot is None or not snapshot[2]:
            with self.lock:self.last_box_count=0
            return
        epoch,origin,boxes=snapshot
        if self.spawn is None:
            offset=self.load_spawn(rpc)
            with self.lock:
                if self.epoch!=epoch:return
                self.spawn=offset
            # Calibration RPC may outlive the input snapshot. Recheck freshness.
            snapshot=self.snapshot(time.perf_counter() if wall is None else wall)
            if snapshot is None or not snapshot[2]:return
            epoch,origin,boxes=snapshot
        with self.lock:
            if self.epoch!=epoch or self.spawn is None:return
            offset=self.spawn
        points=[];labels=[];positions=[]
        for box in boxes:
            points.extend(wire_point(p) for p in box_edges_ned(box.center,box.size,origin,offset))
            top=np.asarray(box.center)+[0.,0.,box.size[2]/2+.25]
            positions.append(wire_point(to_global_ned(top,origin,offset)))
            labels.append(f'DET UAV #{box.track_id} {box.state}')
        with self.lock:
            if self.epoch!=epoch:return
        # Bounded lifetime avoids clearing other tools' persistent markers.
        rpc.call('simPlotLineList',points,self.o['color_rgba'],float(self.o['line_thickness']),
                 float(self.o['line_lifetime_s']),False)
        if self.o['show_labels']:
            rpc.call('simPlotStrings',labels,positions,float(self.o['label_scale']),
                     self.o['color_rgba'],float(self.o['line_lifetime_s']))
        with self.lock:
            self.draw_calls+=1;self.last_box_count=len(boxes);self.last_draw=time.perf_counter();self.last_error=''

    def run(self):
        connection=self.c['airsim']
        rpc=AirSimRPC(connection['host'],connection['port'],self.o['rpc_timeout_s'])
        try:
            while not self.stop.is_set():
                begun=time.perf_counter()
                try:
                    # Calibrate as soon as the session exists, before first detections/flight.
                    with self.lock:
                        calibrate=self.spawn is None and self.origin is not None
                        epoch=self.epoch
                    if calibrate:
                        offset=self.load_spawn(rpc)
                        with self.lock:
                            if self.epoch==epoch:self.spawn=offset
                    self.render_once(rpc)
                except Exception as exc:
                    with self.lock:
                        self.errors+=1;self.last_error=str(exc);self.last_box_count=0;self.spawn=None
                    if self.errors==1 or self.errors%20==0:self.get_logger().warning('UE boxes: '+str(exc))
                self.stop.wait(max(.001,1./self.o['rate_hz']-(time.perf_counter()-begun)))
        finally:rpc.close()

    def report(self):
        wall=time.perf_counter();snapshot=self.snapshot(wall)
        with self.lock:
            values=dict(draw_calls=self.draw_calls,boxes=self.last_box_count,errors=self.errors,
                rejected_messages=self.rejected,session_id=self.session,input_topic=self.input_topic,
                last_error=self.last_error,draw_age_s=wall-self.last_draw if self.last_draw else -1.)
            state='rpc_error' if self.last_error else ('drawing' if snapshot and snapshot[2] else 'waiting_or_no_fresh_detection')
        status=DiagnosticStatus(name=self.root+'/ue_detection_boxes',hardware_id=self.c['airsim']['vehicle'],
            level=DiagnosticStatus.WARN if self.last_error else DiagnosticStatus.OK,message=state,
            values=[KeyValue(key=k,value=str(v)) for k,v in values.items()])
        self.diagnostics.publish(DiagnosticArray(header=header(self.get_clock().now().nanoseconds),status=[status]))

    def close(self):
        self.stop.set()
        if self.worker:self.worker.join(timeout=3*self.o['rpc_timeout_s']+1.)


def main(args=None):
    c,ros=args_config(args);o=options(c)
    if not o['enabled']:return
    lease=SensorLease(dict(backend='airsim',airsim=c['airsim']),'ue_detection_boxes')
    node=None
    rclpy.init(args=ros)
    try:
        node=UeDetectionBoxes(c)
        rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node:node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        lease.close()
