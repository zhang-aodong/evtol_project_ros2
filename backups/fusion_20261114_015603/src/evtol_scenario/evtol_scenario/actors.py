"""Independent configurable pedestrians and obstacle aircraft; never commands ego."""
import math
import threading
import time
import yaml
import json
import numpy as np
import rclpy
from rclpy.node import Node
from evtol_simulation.rpc import AirSimRPC
from evtol_simulation.runtime import SensorLease
from evtol_simulation.coordinates import quaternion_matrix, vector, FRD_FLU
from evtol_perception_interfaces.msg import ReferenceOrigin
from evtol_targeting.common import args_config, project_path, xyz, LATCHED, Health
from .paths import validate,vehicle_path,pedestrian_path,advance,NED_ENU
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
        self.actor_states={}
        self.status_pub=self.create_publisher(String,'/scenario/status',LATCHED)
        self.status_clock=Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.,self.publish_states,clock=self.status_clock)
        self.create_subscription(ReferenceOrigin,'/simulation/reference_origin',self.receive,LATCHED)
        self.create_service(Trigger,'/scenario/stop',self.stop_service)

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
            self.actor_state(name,'arming')
            with self.lock:self.controlled.add(name)
            if not rpc.call('armDisarm',True,name):raise RuntimeError(name+': arming failed')
            if self.stop.is_set():return
            state=rpc.call('getMultirotorState',name)
            self.actor_state(name,'takeoff')
            if state['landed_state']==0 and not rpc.call('takeoff',float(s['takeoff_timeout_s']),name):
                if self.stop.is_set():return
                state=rpc.call('getMultirotorState',name);kin=state['kinematics_estimated']
                # SimpleFlight can leave the ground and stabilize while its
                # takeoff position convergence task reports a timeout. Match
                # the old controller's airborne recovery, with physical checks.
                risen=initial[2]-vector(kin['position'])[2]
                if state['landed_state']==0 or risen<.5 or np.linalg.norm(vector(kin['linear_velocity']))>.5:
                    raise RuntimeError(name+': takeoff failed without stable airborne state')
                self.get_logger().warning(name+': takeoff task timed out but vehicle is airborne and stable; entering configured path')
            iterations=0;repeat=int(agent.get('repeat',0))
            while not self.stop.is_set():
                for waypoint_index,point in enumerate(points):
                    if self.stop.is_set():return
                    self.actor_state(name,'flying',waypoint_index=waypoint_index+1,waypoint_count=len(points),
                        target_local_ned_m=point.tolist(),speed_mps=float(agent['speed_mps']),iteration=iterations+1)
                    for attempt in range(3):
                        succeeded=rpc.call('moveToPosition',*map(float,point),float(agent['speed_mps']),
                            float(s['move_timeout_s']),0,dict(is_rate=True,yaw_or_rate=0.),-1.,1.,name)
                        if self.stop.is_set():return
                        actual=vector(rpc.call('simGetGroundTruthKinematics',name)['position'])
                        error=float(np.linalg.norm(actual-point))
                        if error<=agent.get('waypoint_tolerance_m',s.get('waypoint_tolerance_m',.5)):
                            if not succeeded:self.get_logger().warning(name+': task returned false but measured waypoint error is within tolerance')
                            break
                        if attempt==2:raise RuntimeError(f'{name}: waypoint failed after retries, measured error={error:.3f} m')
                        self.get_logger().warning(f'{name}: waypoint error={error:.3f} m, retrying')
                    self.health.ok()
                    self.actor_state(name,'waypoint_reached',measured_local_ned_m=actual.tolist(),waypoint_error_m=error)
                    if self.stop.wait(agent.get('hold_s',0.)):return
                iterations+=1
                if len(points)==1 or repeat>0 and iterations>=repeat:
                    rpc.call('hover',name);self.actor_state(name,'holding',reason='航点任务完成，悬停等待关闭');self.stop.wait();return
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
        for thread in self.threads:
            thread.join(timeout=4.)
            if thread.is_alive():self.cleanup_ok=False;self.health.error('Actor task did not stop after cancellation')
        with self.lock:names=list(self.controlled)
        for name in names:
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
        for lease in self.leases:lease.close()


def main(args=None):
    from evtol_simulation.runtime import TimerResolution
    resolution=TimerResolution()
    c,ros=args_config(args);rclpy.init(args=ros);node=Actors(c)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.close();node.destroy_node();rclpy.shutdown();resolution.close()
