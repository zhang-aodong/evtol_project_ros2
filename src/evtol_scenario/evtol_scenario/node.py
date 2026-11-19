import argparse
import math
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from evtol_simulation.configuration import load
from evtol_simulation.rpc import AirSimRPC
from evtol_simulation.coordinates import vector, quaternion_matrix


def position(values):
    return dict(zip(('x_val','y_val','z_val'),map(float,values)))


class Scenario(Node):
    def __init__(self,c):
        super().__init__('simulation_scenario')
        self.c=c
        self.stop=threading.Event()
        self.threads=[]
        self.pub=self.create_publisher(String,'/'+c['namespace']+'/scenario/status',10)
        self.controlled=set()
        self.failures=[]
        self.lock=threading.Lock()
        conn=c['airsim']
        rpc=self.client()
        kin=rpc.call('simGetGroundTruthKinematics',conn['vehicle'])
        self.origin=vector(kin['position'])
        rot=quaternion_matrix([kin['orientation'][k] for k in ('x_val','y_val','z_val','w_val')])
        self.yaw=math.atan2(rot[1,0],rot[0,0])
        self.rotation=np.array([[math.cos(self.yaw),-math.sin(self.yaw),0],
                               [math.sin(self.yaw),math.cos(self.yaw),0],[0,0,1]])
        rpc.close()
        source=c['source']
        if source['interference_drones']['enabled']:
            for i,agent in enumerate(source['interference_drones']['agents']):
                self.spawn(self.obstacle,agent,i)
        if source['pedestrians']['enabled']:
            for agent in source['pedestrians']['agents']:
                self.spawn(self.pedestrian,agent)
        self.spawn(self.mission)
        self.create_timer(1.,self.status)

    def client(self,timeout=200.):
        c=self.c['airsim']
        return AirSimRPC(c['host'],c['port'],timeout)

    def spawn(self,fn,*args):
        def work():
            try:
                fn(*args)
            except Exception as exc:
                with self.lock: self.failures.append(f'{fn.__name__}: {exc}')
                self.get_logger().error(f'{fn.__name__}: {exc}')
        t=threading.Thread(target=work,daemon=True)
        self.threads.append(t)
        t.start()

    def status(self):
        with self.lock:
            state={'active_vehicles':sorted(self.controlled),'errors':list(self.failures)}
        import json
        self.pub.publish(String(data=json.dumps(state)))

    def acquire(self,rpc,vehicle):
        rpc.call('enableApiControl',True,vehicle)
        with self.lock: self.controlled.add(vehicle)
        if not rpc.call('armDisarm',True,vehicle):
            raise RuntimeError(f'{vehicle}: failed to arm')
        if not rpc.call('takeoff',20.,vehicle):
            raise RuntimeError(f'{vehicle}: takeoff failed')

    def mission(self):
        rpc=self.client()
        vehicle=self.c['airsim']['vehicle']
        try:
            self.acquire(rpc,vehicle)
            flight=self.c['source']['flight']
            if self.stop.is_set(): return
            yaw=dict(is_rate=False,yaw_or_rate=math.degrees(self.yaw))
            if not rpc.call('moveToZ',float(self.origin[2]-flight['climb_altitude_m']),
                            float(flight['climb_speed_mps']),60.,yaw,-1.,1.,vehicle):
                raise RuntimeError('Mission climb timed out')
            fixed=flight['fixed']
            path=[]
            for point in fixed['waypoints']:
                point=np.asarray(point,dtype=float)
                if fixed['coordinate_mode']=='relative_heading': point=self.origin+self.rotation@point
                elif fixed['coordinate_mode']=='relative_start': point=self.origin+point
                path.append(position(point))
            iteration=0
            while not self.stop.is_set() and (fixed['repeat']==0 or iteration<fixed['repeat']):
                iteration+=1
                if not rpc.call('moveOnPath',path,float(fixed['speed_mps']),float(fixed['timeout_s']),
                                1 if fixed['drivetrain']=='forward_only' else 0,
                                dict(is_rate=True,yaw_or_rate=0.),-1.,1.,vehicle):
                    raise RuntimeError('Fixed scenario path timed out')
            if not self.stop.is_set(): rpc.call('land',60.,vehicle)
        finally:
            rpc.close()

    def obstacle(self,agent,index):
        settings=self.c['source']['interference_drones']
        if self.stop.wait(index*settings['takeoff_stagger_s']): return
        rpc=self.client()
        vehicle=agent['vehicle_name']
        try:
            self.acquire(rpc,vehicle)
            offset=np.asarray(agent['spawn_offset_ned_m'])
            points=[self.origin+self.rotation@np.asarray(p)-offset for p in agent['waypoints_frd_m']]
            while not self.stop.is_set():
                for p in points:
                    if self.stop.is_set(): break
                    if not rpc.call('moveToPosition',*map(float,p),float(agent['speed_mps']),
                        float(settings['move_timeout_s']),0,dict(is_rate=True,yaw_or_rate=0.),-1.,1.,vehicle):
                        raise RuntimeError(f'{vehicle}: path timed out')
                if len(points)==1:
                    rpc.call('hover',vehicle)
                    self.stop.wait()
        finally:
            rpc.close()

    def pedestrian(self,agent):
        settings=self.c['source']['pedestrians']
        rpc=self.client(3.)
        try:
            pose=rpc.call('simGetObjectPose',agent['name'])
            initial=vector(pose['position'])
            if not np.isfinite(initial).all(): raise ValueError('Pedestrian actor not found')
            points=[np.asarray(p,dtype=float)+(initial if agent['coordinate_mode']=='relative_initial' else 0)
                    for p in agent['waypoints']]
            current=initial.copy()
            previous=time.monotonic()
            index=0
            while not self.stop.is_set():
                tick=time.monotonic()
                delta=points[index]-current
                distance=float(np.linalg.norm(delta))
                step=float(agent['speed_mps'])*(tick-previous)
                previous=tick
                if distance<=step:
                    current=points[index].copy()
                    index+=1
                    if index>=len(points):
                        if not agent['loop']: break
                        index=0
                elif distance>1e-9: current+=delta*(step/distance)
                yaw=math.atan2(delta[1],delta[0])+math.radians(settings['forward_yaw_offset_deg'])
                pose=dict(position=position(current),orientation=dict(x_val=0.,y_val=0.,
                    z_val=math.sin(yaw/2),w_val=math.cos(yaw/2)))
                if not rpc.call('simSetObjectPose',agent['name'],pose,settings['teleport']):
                    raise RuntimeError('Failed to move pedestrian')
                self.stop.wait(max(0.,1./settings['update_rate_hz']-(time.monotonic()-tick)))
        finally:
            rpc.close()

    def close(self):
        self.stop.set()
        # Independent connections cancel any long-running movement call first.
        for vehicle in list(self.controlled):
            rpc=self.client(65.)
            try:
                rpc.call('cancelLastTask',vehicle)
                rpc.call('land',60.,vehicle)
                rpc.call('armDisarm',False,vehicle)
                rpc.call('enableApiControl',False,vehicle)
            except Exception as exc:
                self.get_logger().error(f'{vehicle}: cleanup failed: {exc}')
            finally:
                rpc.close()
        for thread in self.threads: thread.join(timeout=2.)


def main(args=None):
    parser=argparse.ArgumentParser()
    parser.add_argument('--config',required=True)
    opts,ros_args=parser.parse_known_args(args)
    c=load(opts.config)
    if c['backend']=='mock': raise ValueError('Scenario requires real AirSim')
    rclpy.init(args=ros_args)
    node=Scenario(c)
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
