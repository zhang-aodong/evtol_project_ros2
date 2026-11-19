"""Read-only checks for scenario origin, status and actual AirSim vehicles."""
import argparse
import json
import time
import rclpy
from rclpy.node import Node
from evtol_perception_interfaces.msg import ReferenceOrigin
from diagnostic_msgs.msg import DiagnosticArray
from rosgraph_msgs.msg import Clock
from evtol_targeting.common import LATCHED


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--seconds',type=float,default=8.)
    opts=parser.parse_args();rclpy.init();node=Node('scenario_read_only_diagnostics')
    report=dict(origins=[],diagnostics={},clock_samples=0)
    def origin(msg):
        report['origins'].append(dict(session_id=msg.session_id,stamp_sec=msg.header.stamp.sec))
    def diagnostic(msg):
        for status in msg.status:
            if status.name.endswith(('/actors','/state','/truth')):
                report['diagnostics'][status.name]=dict(message=status.message,values={v.key:v.value for v in status.values})
    def clock(msg):report['clock_samples']+=1
    subscriptions=[node.create_subscription(ReferenceOrigin,'/simulation/reference_origin',origin,LATCHED),
        node.create_subscription(DiagnosticArray,'/diagnostics',diagnostic,10),
        node.create_subscription(Clock,'/clock',clock,10)]
    try:
        begun=time.perf_counter()
        while time.perf_counter()-begun<opts.seconds:rclpy.spin_once(node,timeout_sec=.1)
        report['nodes']=[ns.rstrip('/')+'/'+name for name,ns in node.get_node_names_and_namespaces()]
        for label,method in [('origin_publishers',node.get_publishers_info_by_topic),('origin_subscribers',node.get_subscriptions_info_by_topic)]:
            report[label]=[dict(node=info.node_name,reliability=str(info.qos_profile.reliability),
                durability=str(info.qos_profile.durability)) for info in method('/simulation/reference_origin')]
        print(json.dumps(report,indent=2))
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
