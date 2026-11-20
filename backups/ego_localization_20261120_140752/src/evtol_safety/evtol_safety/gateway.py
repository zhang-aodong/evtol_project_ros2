"""Only this process commands Drone1. Shadow mode never opens an RPC client."""
import time
import threading
import json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.clock import Clock,ClockType
from std_msgs.msg import String
from std_srvs.srv import Trigger
from evtol_simulation.rpc import AirSimRPC
from evtol_simulation.runtime import SensorLease
from evtol_simulation.coordinates import NED_ENU,vector
from evtol_perception_interfaces.msg import VelocityCommand, ReferenceOrigin, StateStatus
from evtol_targeting.common import args_config, ns, xyz, Health, LATCHED


class Gateway(Node):
    def __init__(self,c,execute=False):
        super().__init__('airsim_control_gateway');self.c=c;self.execute=execute
        self.health=Health(self,'/'+c['namespace']+'/control_gateway')
        self.origin=None;self.command=None;self.received=0.;self.sequence=-1;self.state_ok=False
        self.state_wall=0.;self.lock=threading.Lock();self.stop=threading.Event();self.worker=None;self.lease=None
        self.bootstrapping=False
        self.phase='waiting' if execute else 'shadow';self.failure='';self.land_result=None;self.land_thread=None
        self.flight_pub=self.create_publisher(String,'/'+c['namespace']+'/control/flight_status',LATCHED)
        self.wall_clock=Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.2,self.publish_status,clock=self.wall_clock)
        self.create_service(Trigger,'/'+c['namespace']+'/control/stop',self.stop_service)
        self.create_subscription(ReferenceOrigin,'/simulation/reference_origin',self.receive_origin,LATCHED)
        self.create_subscription(VelocityCommand,'/'+c['namespace']+'/control/safe_velocity',self.receive,qos_profile_sensor_data)
        self.create_subscription(StateStatus,'/'+c['namespace']+'/state/status',self.status,10)
        if execute:
            lease_config=dict(backend='airsim',airsim=c['airsim'])
            self.lease=SensorLease(lease_config,'vehicle_control')
            self.worker=threading.Thread(target=self.run,daemon=True);self.worker.start()
        self.get_logger().info('EXECUTE: AirSim velocity gateway' if execute else 'SHADOW: no AirSim control connection')

    def publish_status(self):
        if self.origin:self.flight_pub.publish(String(data=json.dumps(dict(session_id=self.origin.session_id,
            phase=self.phase,execute=self.execute,reason=self.failure))))

    def transition(self,phase):
        if phase!=self.phase:self.get_logger().info('Flight phase: '+phase)
        self.phase=phase;self.publish_status()

    def stop_service(self,request,response):
        self.stop.set();response.success=True;response.message='Stopping velocity execution; gateway cancels task, requests hover and releases API control'
        return response

    def receive_origin(self,msg):
        with self.lock:
            if self.origin and self.origin.session_id!=msg.session_id:
                self.command=None;self.sequence=-1;self.state_ok=False
                if self.execute:self.stop.set();self.failure='Origin changed during execution; restart gateway'
            self.origin=msg

    def status(self,msg):
        with self.lock:
            self.state_ok=bool(self.origin and msg.session_id==self.origin.session_id and msg.valid)
            self.state_wall=time.perf_counter()

    def receive(self,msg):
        with self.lock:
            if self.origin is None or msg.session_id!=self.origin.session_id or msg.header.frame_id!='map_enu':return
            if msg.sequence<=self.sequence or not np.isfinite(xyz(msg.velocity)).all() or not np.isfinite(msg.yaw_rate):return
            if np.linalg.norm(xyz(msg.velocity))>self.c['control']['max_speed_mps']+1e-6:return
            self.sequence=msg.sequence;self.command=msg;self.received=time.perf_counter()
        if not self.execute:self.health.ok()

    def run(self):
        c=self.c['airsim'];rpc=AirSimRPC(c['host'],c['port'],self.c['control']['rpc_timeout_s']);vehicle=c['vehicle']
        acquired=False
        try:
            # A missing origin/state cannot trigger arming or movement.
            while not self.stop.wait(.02):
                now=self.get_clock().now().nanoseconds;wall=time.perf_counter()
                with self.lock:
                    ready=bool(self.origin and self.state_ok and wall-self.state_wall<self.c['state']['timeout_s']
                        and self.command and wall-self.received<self.c['control']['ttl_s']
                        and self.command.reason in ('mission_waiting','mission_arming','mission_takeoff','clear','avoidance')
                        and ns(self.command.valid_until)>now and -int(self.c['control'].get('clock_tolerance_s',.075)*1e9)<=now-ns(self.command.header.stamp)<=int(self.c['control']['ttl_s']*1e9))
                if ready:break
            if self.stop.is_set():return
            self.transition('arming');rpc.call('enableApiControl',True,vehicle);acquired=True
            gains=self.c['control'].get('velocity_gains')
            if gains:rpc.call('setVelocityControllerGains',gains['kp'],gains['ki'],gains['kd'],vehicle)
            controlled_takeoff=self.c.get('navigation_lidar',{}).get('enabled',False) and self.c['control']['takeoff_on_start']
            if controlled_takeoff:
                initial=rpc.call('getMultirotorState',vehicle)
                initial_z=vector(initial['kinematics_estimated']['position'])[2]
                if not rpc.call('armDisarm',True,vehicle):raise RuntimeError('Arming failed')
                self.transition('takeoff');takeoff_deadline=time.perf_counter()+self.c['control'].get('takeoff_timeout_s',60.)
            elif self.c['control']['takeoff_on_start']:
                self.bootstrapping=True
                self.transition('takeoff')
                initial=rpc.call('getMultirotorState',vehicle)
                initial_z=vector(initial['kinematics_estimated']['position'])[2]
                if not rpc.call('armDisarm',True,vehicle):raise RuntimeError('Arming failed')
                # Takeoff is explicitly configured and uses the existing AirSim
                # task semantics; no background scenario may also own Drone1.
                bootstrap=AirSimRPC(c['host'],c['port'],65.)
                try:
                    # SimpleFlight can report Flying while resting on the
                    # ground before arming. Explicit takeoff must not depend
                    # on that unreliable initial enum.
                    if not bootstrap.call('takeoff',20.,vehicle):
                        state=bootstrap.call('getMultirotorState',vehicle);kin=state['kinematics_estimated']
                        if self.stop.is_set():return
                        if state['landed_state']==0 or initial_z-vector(kin['position'])[2]<.5 or np.linalg.norm(vector(kin['linear_velocity']))>.5:
                            raise RuntimeError('Takeoff failed without stable airborne state')
                        self.get_logger().warning('Takeoff task timed out; verified stable airborne state, entering velocity control')
                finally:bootstrap.close();self.bootstrapping=False
            elif rpc.call('getMultirotorState',vehicle)['landed_state']==0:
                raise RuntimeError('Vehicle is landed: enable control.takeoff_on_start before executing a new mission')
            if not controlled_takeoff:self.transition('flying')
            while not self.stop.is_set():
                begun=time.perf_counter();now=self.get_clock().now().nanoseconds
                with self.lock:cmd=self.command;received=self.received;valid_state=self.state_ok and begun-self.state_wall<self.c['state']['timeout_s']
                valid=bool(cmd and valid_state and -int(self.c['control'].get('clock_tolerance_s',.075)*1e9)<=now-ns(cmd.header.stamp)<=int(self.c['control']['ttl_s']*1e9)
                    and ns(cmd.valid_until)>now and begun-received<self.c['control']['ttl_s'])
                if self.phase=='takeoff':
                    kin=rpc.call('getMultirotorState',vehicle)['kinematics_estimated']
                    risen=initial_z-vector(kin['position'])[2]
                    if risen>=self.c['mission'].get('takeoff_height_m',1.5)-.25:
                        self.transition('flying')
                    elif time.perf_counter()>takeoff_deadline:
                        raise RuntimeError('Safety-controlled takeoff timed out; last safety reason: '+(cmd.reason if cmd else 'no command'))
                if self.land_thread:
                    if self.land_thread.is_alive():
                        if not valid or cmd.reason!='mission_land':
                            rpc.call('cancelLastTask',vehicle)
                            self.land_thread.join(timeout=2.)
                            if self.land_thread.is_alive():raise RuntimeError('Cancelled landing task did not finish')
                            self.land_thread=None;self.land_result=None
                            self.failure='Landing paused: '+(cmd.reason if cmd else 'missing command')
                            self.transition('landing_paused')
                            # Retain API control and command zero velocity.
                            # Fresh, clear landing requests can retry; no RC
                            # handover or airborne disarm on a transient gap.
                            continue
                        self.stop.wait(.02);continue
                    if self.land_result is not True:raise RuntimeError('Landing task failed: '+str(self.land_result))
                    # AirSim can set Landed only after a successful disarm.
                    state=rpc.call('getMultirotorState',vehicle)
                    if np.linalg.norm(vector(state['kinematics_estimated']['linear_velocity']))>.2:
                        self.stop.wait(.02);continue
                    local=NED_ENU@vector(state['kinematics_estimated']['position'])-xyz(self.origin.source_origin_enu)
                    touchdown=np.asarray(self.c['mission']['waypoints_enu_m'][-1])
                    if abs(local[2]-touchdown[2])>.45 or np.linalg.norm(local[:2]-touchdown[:2])>self.c['mission']['tolerance_m']+.4:
                        raise RuntimeError('Landing returned before verified touchdown at final waypoint; refusing airborne disarm')
                    if not rpc.call('armDisarm',False,vehicle):raise RuntimeError('Disarm after landing failed')
                    deadline=time.perf_counter()+3.
                    while rpc.call('getMultirotorState',vehicle)['landed_state']!=0:
                        if time.perf_counter()>deadline:raise RuntimeError('Landed state not confirmed')
                        self.stop.wait(.1)
                    rpc.call('enableApiControl',False,vehicle);acquired=False
                    self.transition('landed');self.health.ok();return
                if valid and cmd.reason=='mission_land':
                    self.failure=''
                    self.transition('landing')
                    self.land_thread=threading.Thread(target=self.land,args=(vehicle,),daemon=True);self.land_thread.start();continue
                velocity=NED_ENU@xyz(cmd.velocity) if valid else np.zeros(3)
                # RPC waits for this short task. Timeouts cannot stack commands;
                # each request expires after duration even if this process dies.
                rpc.call('moveByVelocity',*map(float,velocity),self.c['control']['duration_s'],0,
                    dict(is_rate=True,yaw_or_rate=-float(np.degrees(cmd.yaw_rate)) if valid else 0.),vehicle)
                self.health.ok();self.stop.wait(max(0.,1./self.c['control']['rate_hz']-(time.perf_counter()-begun)))
        except Exception as exc:self.failure=str(exc);self.health.error(exc);self.transition('failed')
        finally:
            if acquired:
                try:
                    rpc.call('cancelLastTask',vehicle);rpc.call('hover',vehicle);rpc.call('enableApiControl',False,vehicle)
                except Exception as exc:self.health.error(exc)
            rpc.close()
            if self.stop.is_set():self.transition('stopped')

    def land(self,vehicle):
        c=self.c['airsim'];rpc=AirSimRPC(c['host'],c['port'],65.)
        try:self.land_result=bool(rpc.call('land',60.,vehicle))
        except Exception as exc:self.land_result=str(exc)
        finally:rpc.close()

    def close(self):
        self.stop.set()
        if self.execute:
            c=self.c['airsim'];rpc=AirSimRPC(c['host'],c['port'],1.)
            try:rpc.call('cancelLastTask',c['vehicle'])
            except Exception as exc:self.health.error(exc)
            finally:rpc.close()
        if self.worker:self.worker.join(timeout=4.)
        if self.lease:self.lease.close()


def main(args=None):
    from evtol_simulation.runtime import TimerResolution
    resolution=TimerResolution()
    import argparse
    p=argparse.ArgumentParser(add_help=False);p.add_argument('--execute',action='store_true');opts,remaining=p.parse_known_args(args)
    c,ros=args_config(remaining);rclpy.init(args=ros);node=Gateway(c,opts.execute)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.close();node.destroy_node();rclpy.shutdown();resolution.close()
