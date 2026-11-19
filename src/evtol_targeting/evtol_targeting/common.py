import argparse
import copy
from pathlib import Path
import time
import yaml
import numpy as np
from builtin_interfaces.msg import Time
from std_msgs.msg import Header
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
LATEST = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


def ns(value):
    return int(value.sec)*1_000_000_000 + int(value.nanosec)


def stamp(value):
    return Time(sec=int(value)//1_000_000_000, nanosec=int(value)%1_000_000_000)


def header(value, frame='map_enu'):
    return Header(stamp=stamp(value), frame_id=frame)


def xyz(value):
    return np.array([value.x, value.y, value.z], dtype=float)


def put(value, array):
    value.x, value.y, value.z = map(float, array)


def args_config(args=None):
    parser=argparse.ArgumentParser()
    parser.add_argument('--config',required=True)
    opts,remaining=parser.parse_known_args(args)
    path=Path(opts.config).resolve()
    c=yaml.safe_load(path.read_text(encoding='utf-8-sig'))
    from .configuration import validate_config
    validate_config(c)
    c['_root']=Path(c.get('project_root',path.parent.parent)).resolve()
    return c,remaining


def project_path(c, value):
    path=Path(value)
    return path if path.is_absolute() else c['_root']/path


class Health:
    def __init__(self,node,name):
        self.node,self.name=node,name
        self.pub=node.create_publisher(DiagnosticArray,'/diagnostics',10)
        self.count=self.errors=self.dropped=0
        self.last_error='';self.last=None;self.started=time.perf_counter()
        node.create_timer(1.,self.emit)

    def ok(self):
        self.count+=1;self.last=time.perf_counter();self.last_error=''

    def error(self,exc):
        self.errors+=1;self.last_error=str(exc)
        if self.errors==1 or self.errors%20==0:self.node.get_logger().error(str(exc))

    def emit(self):
        age=time.perf_counter()-(self.last or self.started)
        bad=bool(self.last_error) or age>2.
        status=DiagnosticStatus(name=self.name,hardware_id='simulation',
            level=DiagnosticStatus.WARN if bad else DiagnosticStatus.OK,
            message=self.last_error or ('waiting' if self.last is None else 'streaming'),
            values=[KeyValue(key=k,value=str(v)) for k,v in dict(samples=self.count,
                errors=self.errors,dropped=self.dropped,age_s=age,**getattr(self,'details',{})).items()])
        self.pub.publish(DiagnosticArray(status=[status]))


def odom_arrays(msg):
    from evtol_simulation.coordinates import quaternion_matrix
    q=msg.pose.pose.orientation
    r=quaternion_matrix([q.x,q.y,q.z,q.w])
    return xyz(msg.pose.pose.position),r,r@xyz(msg.twist.twist.linear)


def clone(msg):
    return copy.deepcopy(msg)
