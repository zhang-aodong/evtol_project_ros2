"""ROS wiring only. Subscribes to sensor observations and frozen references.

No truth/state/TF subscription and no simulator RPC in this process.
"""
import json
import time
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import Imu,NavSatFix
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock as ClockMsg
from evtol_sim_interfaces.msg import FiveGObservations
from evtol_perception_interfaces.msg import ReferenceOrigin
from evtol_targeting.common import args_config,LATCHED,ns,header,xyz,put,clone,Health
from .options import options
from .stream import FusionStream
from .geodesy import GeoReference

SENSOR_QOS=QoSProfile(depth=128,reliability=ReliabilityPolicy.BEST_EFFORT)


class LocalizationNode(Node):
    def __init__(self,c):
        super().__init__('ego_localization');self.c=c;self.o=options(c.get('localization'));self.root='/'+c['namespace']
        self.stream=FusionStream(self.o);self.origin=None;self.datum=None;self.geo=None
        self.clock_ns=0;self.flight_ns=None;self.published_ns=0;self.path=deque(maxlen=20000);self.last_path_ns=0
        self.last_input_wall=None;self.publish_times=deque(maxlen=1000);self.ages=deque(maxlen=10000)
        self.health=Health(self,self.root+'/ego_localization')
        self.odom_pub=self.create_publisher(Odometry,self.root+'/localization/odometry',SENSOR_QOS)
        self.path_pub=self.create_publisher(Path,self.root+'/localization/path',LATCHED)
        self.status_pub=self.create_publisher(String,self.root+'/localization/status',10)
        self.create_subscription(ReferenceOrigin,'/simulation/reference_origin',self.reference,LATCHED)
        self.create_subscription(NavSatFix,'/simulation/geodetic_reference',self.geodetic,LATCHED)
        self.create_subscription(Imu,self.root+'/imu/data',self.imu,SENSOR_QOS)
        self.create_subscription(NavSatFix,self.root+'/gnss/fix',self.gnss,SENSOR_QOS)
        if self.o['mode']=='gnss_imu_5g':self.create_subscription(FiveGObservations,self.root+'/five_ga/observations',self.fiveg,SENSOR_QOS)
        self.create_subscription(ClockMsg,'/clock',self.clock,SENSOR_QOS)
        self.create_subscription(String,self.root+'/control/flight_status',self.flight,LATCHED)
        steady=Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.005,self.tick,clock=steady);self.create_timer(.5,self.report,clock=steady)

    def clear(self,reason):
        self.stream.reset(reason);self.published_ns=0;self.path.clear();self.last_path_ns=0;self.flight_ns=None
        self.publish_times.clear();self.ages.clear();self.last_input_wall=None
        self.path_pub.publish(Path(header=header(self.clock_ns)))

    def reference(self,msg):
        if self.origin is None or msg.session_id!=self.origin.session_id:
            self.clear('session_change');self.origin=msg;self.update_geo()

    def geodetic(self,msg):
        self.datum=msg;self.update_geo()

    def update_geo(self):
        self.geo=None
        if self.origin and self.datum and self.datum.header.frame_id==self.origin.source_frame:
            try:self.geo=GeoReference(self.datum.latitude,self.datum.longitude,self.datum.altitude,xyz(self.origin.source_origin_enu))
            except ValueError as exc:self.health.error(exc)

    def clock(self,msg):
        value=ns(msg.clock)
        if self.clock_ns and value<self.clock_ns:
            self.clear('clock_regression');self.origin=None;self.datum=None;self.geo=None
        self.clock_ns=value

    def flight(self,msg):
        try:
            v=json.loads(msg.data)
            if self.origin and v.get('session_id')==self.origin.session_id and v.get('phase') in ('takeoff','flying') and self.flight_ns is None:
                self.flight_ns=v.get('takeoff_stamp_ns') or v.get('stamp_ns') or self.clock_ns
        except ValueError:pass

    def push(self,kind,msg,data,key=None):
        t=ns(msg.header.stamp)
        if self.origin is None or self.geo is None:
            self.stream.reject(kind,'waiting_reference');return
        if t<ns(self.origin.header.stamp):self.stream.reject(kind,'previous_session');return
        if self.clock_ns and t>self.clock_ns+int(.5e9):self.stream.reject(kind,'future_clock');return
        if self.stream.push(kind,t,data,key):self.last_input_wall=time.monotonic()

    def imu(self,msg):
        q=msg.orientation;v=np.array([q.x,q.y,q.z,q.w]);norm=np.linalg.norm(v)
        valid=msg.orientation_covariance[0]>=0 and np.isfinite(v).all() and .5<norm<1.5
        self.push('imu',msg,dict(accel=xyz(msg.linear_acceleration),gyro=xyz(msg.angular_velocity),q=v/norm if valid else None))

    def gnss(self,msg):
        try:
            position=self.geo.position(msg.latitude,msg.longitude,msg.altitude) if self.geo else np.zeros(3)
            covariance=np.asarray(msg.position_covariance,float).reshape(3,3)
            # NavSatFix covariance is in local tangent ENU; within this scene its
            # axes agree with map ENU. Unknown/zero covariance uses configured floor.
            if msg.position_covariance_type==0:covariance=np.eye(3)*self.o['gnss_sigma_floor_m']**2
            self.push('gnss',msg,dict(position=position,covariance=covariance,valid=msg.status.status>=0))
        except (ValueError,TypeError):self.stream.reject('gnss','nonfinite_fix')

    def fiveg(self,msg):
        if not self.geo:return
        seen=set();links=[]
        for link in msg.observations:
            if link.gnb_id in seen:self.stream.reject('fiveg','duplicate_anchor',link.gnb_id);continue
            seen.add(link.gnb_id)
            links.append(dict(id=link.gnb_id,anchor=self.geo.anchor(xyz(link.base_station_enu)),
                valid=link.measurement_valid,range=link.measured_range_m,sigma=link.range_sigma_m,rsrp=link.prs_rsrp_dbm))
        self.push('fiveg',msg,links)

    def tick(self):
        if not self.clock_ns or not self.geo:return
        base=self.flight_ns if self.o['gnss_fault']['time_base']=='flight' else ns(self.origin.header.stamp)
        try:self.stream.drain(self.clock_ns,base)
        except Exception as exc:self.health.error(exc);return
        f=self.stream.filter
        if f.t is None or f.t<=self.published_ns or self.stream.status=='stale':return
        # Never publish a held state after the input stopped, including UE pause.
        if self.last_input_wall is None or time.monotonic()-self.last_input_wall>self.o['stale_s']:return
        msg=Odometry(header=header(f.t),child_frame_id=self.c['namespace']+'/base_link')
        put(msg.pose.pose.position,f.p);q=msg.pose.pose.orientation;q.x,q.y,q.z,q.w=map(float,f.q)
        velocity,angular,pc,tc=f.body_output(self.stream.last_imu[1]['gyro'])
        put(msg.twist.twist.linear,velocity);put(msg.twist.twist.angular,angular)
        msg.pose.covariance=pc.ravel().tolist();msg.twist.covariance=tc.ravel().tolist()
        self.odom_pub.publish(msg);self.published_ns=f.t;self.publish_times.append(time.perf_counter())
        self.ages.append((self.clock_ns-f.t)*1e-9);self.health.ok()
        if f.t-self.last_path_ns>=int(.2e9):
            self.path.append(PoseStamped(header=clone(msg.header),pose=clone(msg.pose.pose)))
            self.path_pub.publish(Path(header=clone(msg.header),poses=list(self.path)));self.last_path_ns=f.t

    def report(self):
        d=self.stream.diagnostic();f=self.stream.filter
        age=None if f.t is None else (self.clock_ns-f.t)*1e-9
        if not self.origin:d['status']='waiting_session_origin'
        elif not self.geo:d['status']='waiting_matching_geodetic_reference'
        elif self.last_input_wall is None or time.monotonic()-self.last_input_wall>self.o['stale_s']:d['status']='stale' if f.t else 'waiting_initialization'
        span=self.publish_times[-1]-self.publish_times[0] if len(self.publish_times)>1 else 0.
        hz=(len(self.publish_times)-1)/span if span>0 else 0.
        d.update(session_id=self.origin.session_id if self.origin else '',stamp_ns=self.clock_ns,state_stamp_ns=f.t,
            mode=self.o['mode'],state_age_s=age,publish_hz=hz,gnss_fault_time_base_ns=self.flight_ns if self.o['gnss_fault']['time_base']=='flight' else ns(self.origin.header.stamp) if self.origin else None,
            state_age_p95_s=float(np.percentile(self.ages,95)) if self.ages else None,
            state_age_max_s=max(self.ages) if self.ages else None,
            position_std_m=np.sqrt(np.diag(f.P)[:3]).tolist() if f.t else None)
        self.status_pub.publish(String(data=json.dumps(d,allow_nan=False)))
        self.health.details=dict(status=d['status'],mode=d['mode'],publish_hz=hz,state_age_s=age,last_rejection=d['last_rejection'])


def main(args=None):
    c,ros=args_config(args);rclpy.init(args=ros);node=LocalizationNode(c)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
