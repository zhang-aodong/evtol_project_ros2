"""Shared acquisition-only argument, header and health utilities."""
import argparse
from pathlib import Path
import time
import yaml
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue
from rclpy.qos import QoSProfile,ReliabilityPolicy
LATEST=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)

def header(ns,frame):return Header(stamp=Time(sec=int(ns)//10**9,nanosec=int(ns)%10**9),frame_id=frame)

def args_config(args=None):
    p=argparse.ArgumentParser();p.add_argument('--config',required=True);a,ros=p.parse_known_args(args)
    return yaml.safe_load(Path(a.config).read_text(encoding='utf-8-sig')),ros

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


