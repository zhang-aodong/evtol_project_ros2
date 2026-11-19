"""Independent configurable pedestrians and obstacle aircraft; never commands ego."""
import math
import threading
import time
import yaml
import json
from concurrent.futures import ThreadPoolExecutor
import numpy as np
import rclpy
from rclpy.node import Node
from evtol_simulation.rpc import AirSimRPC
from evtol_simulation.runtime import SensorLease
from evtol_simulation.coordinates import quaternion_matrix, vector, FRD_FLU
from evtol_perception_interfaces.msg import ReferenceOrigin
from evtol_targeting.common import args_config, project_path, xyz, LATCHED, Health
from .paths import validate,vehicle_path,pedestrian_path,advance,NED_ENU
from .flight import PositionFlight
from std_srvs.srv import Trigger
from std_msgs.msg import String
from rclpy.clock import Clock,ClockType


class Actors(Node):
    def __init__(self,c):
        super().__init__('scenario_actors');self.c=c
        self.settings=yaml.safe_load(project_path(c,c['actors_config']).read_text(encoding='utf-8-sig'))
        validate(self.settings,c['airsim']['vehicle'])
        self.health=Health(self,'/'+c['namespace']+'/actors');self.origin=None;self.started=False
        self.stop=threading.Event();self.threads=[];self.controlled=set();self.lock=threading.Lock();self.leases=[]
        self.closed=False
        self.motion=threading.Event()
        if not self.settings['interference_drones'].get('wait_for_ego_flying',False):self.motion.set()
        self.actor_states={}
        self.status_pub=self.create_publisher(String,'/scenario/status',LATCHED)
        self.status_clock=Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.,self.publish_states,clock=self.status_clock)
        self.create_subscription(ReferenceOrigin,'/simulation/reference_origin',self.receive,LATCHED)
        self.create_subscription(String,'/'+c['namespace']+'/control/flight_status',self.receive_flight,LATCHED)
        self.create_service(Trigger,'/scenario/stop',self.stop_service)

    def receive_flight(self,msg):
        try:status=json.loads(msg.data)
        except (ValueError,TypeError):return
        if self.origin and status.get('session_id')==self.origin.session_id and status.get('phase')=='flying':
            if not self.motion.is_set():self.get_logger().info('本机已起飞，释放障碍机航线执行')
            self.motion.set()

    def stop_service(self,request,response):
        self.close();response.success=self.cleanup_ok
        response.message='Actors stopped and cleanup completed' if self.cleanup_ok else 'Cleanup failed; inspect /diagnostics before ending the process'
        return response

    def publish_states(self):
        with self.lock:states=dict(self.actor_states)
        self.status_pub.publish(String(data=json.dumps(dict(session_id=self.origin.session_id if self.origin else '',
            actors=states,stopping=self.stop.is_set()),ensure_ascii=False)))

    def actor_state(self,name,phase,**details):
        with self.lock:
            old=self.actor_states.get(name,{})
            state=dict(old,name=name,phase=phase,updated_wall=time.time(),reason='')
            state.update(details);self.actor_states[name]=state
        self.get_logger().info(name+': '+phase+' '+json.dumps(details,ensure_ascii=False))
        self.publish_states()

    def client(self,timeout=3.):
        c=self.c['airsim'];return AirSimRPC(c['host'],c['port'],timeout)

    def receive(self,msg):
        if self.origin and self.origin.session_id!=msg.session_id:
            self.stop.set();self.health.error('Origin session changed: restart actors after cleanup')
            threading.Thread(target=self.close,daemon=True).start();return
        self.origin=msg
        if self.started:return
        self.started=True
        # Source truth is ego-local NED. Spawn offsets below are relative to
        # the ego spawn, matching the existing project configuration contract.
        self.ego_origin=NED_ENU@xyz(msg.source_origin_enu)
        q=msg.initial_orientation
        full_rotation=NED_ENU@quaternion_matrix([q.x,q.y,q.z,q.w])@FRD_FLU
        yaw=math.atan2(full_rotation[1,0],full_rotation[0,0])
        self.ego_rotation=np.array([[math.cos(yaw),-math.sin(yaw),0.],
            [math.sin(yaw),math.cos(yaw),0.],[0.,0.,1.]])
        settings=self.settings
        if settings['interference_drones']['enabled']:
            for index,agent in enumerate(settings['interference_drones']['agents']):self.spawn(self.vehicle,agent,index)
        if settings['pedestrians']['enabled']:
            for agent in settings['pedestrians']['agents']:self.spawn(self.pedestrian,agent)

    def spawn(self,fn,*args):
        def run():
            try:fn(*args)
            except Exception as exc:
                self.actor_state(args[0].get('vehicle_name',args[0].get('name','actor')),'failed',reason=str(exc))
                self.health.error(exc)
        thread=threading.Thread(target=run,daemon=True);self.threads.append(thread);thread.start()

    def vehicle(self,agent,index):
        s=self.settings['interference_drones'];name=agent['vehicle_name']
        self.actor_state(name,'waiting',reason='等待错峰启动')
        if self.stop.wait(agent.get('start_delay_s',index*s['takeoff_stagger_s'])):return
        lc=dict(backend='airsim',airsim=dict(self.c['airsim'],vehicle=name))
        lease=SensorLease(lc,'vehicle_control')
        with self.lock:self.leases.append(lease)
        rpc=self.client(s['rpc_timeout_s'])
        try:
            initial=vector(rpc.call('simGetGroundTruthKinematics',name)['position'])
            points=vehicle_path(agent,self.ego_origin,self.ego_rotation,initial)
            if self.stop.is_set():return
            rpc.call('enableApiControl',True,name)
            with self.lock:self.controlled.add(name)
            flight=PositionFlight(rpc,name,self.stop,agent.get('waypoint_tolerance_m',s.get('waypoint_tolerance_m',.5)))
            if s.get('initialization','ground')=='first_waypoint':
                self.actor_state(name,'initializing',reason='实验开始前在首航点布设，清零速度；飞行中不重定位',
                    target_local_ned_m=points[0].tolist())
                if not flight.place(points[0]):return
            else:
                self.actor_state(name,'arming')
                position,velocity=flight.state()
                if np.linalg.norm(velocity)>.25:
                    collision=rpc.call('simGetCollisionInfo',name)
                    raise RuntimeError(f'{name}: ground spawn is moving at {np.linalg.norm(velocity):.2f} m/s; '
                        f'collision={collision.get("object_name", "")}; use a clear ground spawn or first_waypoint initialization')
                if not rpc.call('armDisarm',True,name):raise RuntimeError(name+': arming failed')
                if self.stop.is_set():return
                self.actor_state(name,'takeoff')
                if not rpc.call('takeoff',float(s['takeoff_timeout_s']),name):
                    if self.stop.is_set():return
                    current,velocity=flight.state()
                    if position[2]-current[2]<.5 or np.linalg.norm(velocity)>.5:
                        raise RuntimeError(name+': takeoff failed without stable airborne state')
                    self.get_logger().warning(name+': takeoff task timed out; verified stable airborne state')
            iterations=0;repeat=int(agent.get('repeat',0))
            while not self.stop.is_set():
                for waypoint_index,point in enumerate(points):
                    if self.stop.is_set():return
                    self.actor_state(name,'flying',waypoint_index=waypoint_index+1,waypoint_count=len(points),
                        target_local_ned_m=point.tolist(),speed_mps=float(agent['speed_mps']),iteration=iterations+1)
                    arrival=flight.move(point,float(agent['speed_mps']),float(s['move_timeout_s']))
                    if arrival is None:return
                    actual,error=arrival
                    self.health.ok()
                    self.actor_state(name,'waypoint_reached',ready_for_experiment=True,measured_local_ned_m=actual.tolist(),waypoint_error_m=error)
                    if iterations==0 and waypoint_index==0 and not self.motion.is_set():
                        self.actor_state(name,'ready',reason='首航点已稳定，悬停等待本机起飞',ready_for_experiment=True)
                        while not self.stop.is_set() and not self.motion.is_set():
                            flight.step(point,1.)
                            self.health.ok();self.stop.wait(.08)
                        if self.stop.is_set():return
                    if self.stop.wait(agent.get('hold_s',0.)):return
                iterations+=1
                if len(points)==1 or repeat>0 and iterations>=repeat:
                    self.actor_state(name,'holding',reason='航点任务完成，悬停等待关闭')
                    while not self.stop.is_set():flight.step(points[-1],1.);self.health.ok();self.stop.wait(.08)
                    return
        finally:rpc.close()

    def pedestrian(self,agent):
        s=self.settings['pedestrians'];rpc=self.client();name=agent['name']
        self.actor_state(name,'waiting')
        if self.stop.wait(agent.get('start_delay_s',0.)):rpc.close();return
        try:
            pose=rpc.call('simGetObjectPose',name);initial=vector(pose['position'])
            if not np.isfinite(initial).all():raise ValueError('Pedestrian Actor missing: '+name)
            points=pedestrian_path(agent,initial,self.ego_origin);current=initial.copy();index=0;last=time.perf_counter()
            self.actor_state(name,'moving',waypoint_count=len(points))
            while not self.stop.is_set():
                tick=time.perf_counter();dt=min(.2,tick-last);last=tick
                delta=points[index]-current
                current,reached=advance(current,points[index],float(agent['speed_mps'])*dt)
                yaw=math.atan2(delta[1],delta[0])+math.radians(s['forward_yaw_offset_deg'])
                pose=dict(position=dict(zip(('x_val','y_val','z_val'),map(float,current))),
                    orientation=dict(x_val=0.,y_val=0.,z_val=math.sin(yaw/2),w_val=math.cos(yaw/2)))
                if not rpc.call('simSetObjectPose',name,pose,bool(s['teleport'])):raise RuntimeError('Cannot move '+name)
                self.health.ok()
                if reached:
                    index+=1
                    if index==len(points):
                        if not agent['loop']:break
                        index=0
                    if self.stop.wait(agent.get('hold_s',0.)):break
                self.stop.wait(max(0.,1./s['update_rate_hz']-(time.perf_counter()-tick)))
            self.actor_state(name,'stopped')
        finally:rpc.close()

    def close(self):
        if self.closed:return
        self.closed=True
        self.cleanup_ok=True
        self.stop.set()
        self.get_logger().info('开始场景清理：取消任务、等待工作线程、逐架降落并释放控制')
        with self.lock:names=list(self.controlled)
        # Cancel long movement RPCs before joining producer threads. Landing
        # uses separate clients and never blocks another actor's cancellation.
        for name in names:
            self.actor_state(name,'cancelling')
            rpc=self.client()
            try:rpc.call('cancelLastTask',name)
            except Exception as exc:self.cleanup_ok=False;self.health.error(exc)
            finally:rpc.close()
        join_deadline=time.monotonic()+4.
        for thread in self.threads:
            thread.join(timeout=max(0.,join_deadline-time.monotonic()))
            if thread.is_alive():self.cleanup_ok=False;self.health.error('Actor task did not stop after cancellation')
        with self.lock:names=list(self.controlled)
        def finish(name):
            rpc=self.client(65.)
            try:
                if self.settings['interference_drones']['land_on_stop']:
                    self.actor_state(name,'landing',reason='普通关闭：等待降落，最多60s')
                    deadline=time.perf_counter()+60.
                    if not rpc.call('land',60.,name):raise RuntimeError(name+': landing did not complete')
                    # SimpleFlight can update landed_state only after the
                    # disarm request. Require successful landing and settled
                    # motion first, then verify disarm and landed-state change.
                    while np.linalg.norm(vector(rpc.call('getMultirotorState',name)['kinematics_estimated']['linear_velocity']))>.2:
                        if time.perf_counter()>deadline:raise RuntimeError(name+': landing motion has not settled')
                        time.sleep(.1)
                    if not rpc.call('armDisarm',False,name):raise RuntimeError(name+': disarm failed')
                    while rpc.call('getMultirotorState',name)['landed_state']!=0:
                        if time.perf_counter()>deadline:raise RuntimeError(name+': landed state not confirmed after disarm')
                        time.sleep(.1)
                else:rpc.call('hover',name)
                rpc.call('enableApiControl',False,name)
                self.actor_state(name,'stopped',reason='已释放 API 控制')
            except Exception as exc:
                self.actor_state(name,'cleanup_failed',reason=str(exc));self.cleanup_ok=False;self.health.error(exc)
            finally:rpc.close()
        # A serial 32 x 60 s landing made failed startup look like a deadlock.
        if names:
            with ThreadPoolExecutor(max_workers=min(16,len(names))) as pool:list(pool.map(finish,names))
        for lease in self.leases:lease.close()


def main(args=None):
    from evtol_simulation.runtime import TimerResolution
    resolution=TimerResolution()
    c,ros=args_config(args);rclpy.init(args=ros);node=Actors(c)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.close();node.destroy_node();rclpy.shutdown();resolution.close()
