"""Read-only live topic audit, with acquisition age and non-fabricated counts."""
import argparse
from collections import Counter
import json
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from diagnostic_msgs.msg import DiagnosticArray
from evtol_perception_interfaces.msg import SemanticFrame,TargetArray,VelocityCommand,Risk,ReferenceOrigin
from evtol_targeting.common import ns,LATCHED


def main():
    p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=30)
    p.add_argument('--output',default='reports/targeting_live.json');opts=p.parse_args()
    rclpy.init();node=Node('targeting_read_only_audit');node.set_parameters([rclpy.Parameter('use_sim_time',value=True)])
    stats={};diagnostics={};origins=[]
    def receive(topic,msg):
        row=stats.setdefault(topic,dict(arrivals=[],ages=[],counts=[],statuses=Counter(),inference_ms=[]))
        row['arrivals'].append(time.perf_counter())
        if hasattr(msg,'header'):
            age=(node.get_clock().now().nanoseconds-ns(msg.header.stamp))*1e-9
            if abs(age)<3600:row['ages'].append(age)
        if hasattr(msg,'targets'):row['counts'].append(len(msg.targets));row['statuses'][msg.status]+=1
        if hasattr(msg,'inference_ms'):row['inference_ms'].append(msg.inference_ms)
        if hasattr(msg,'reason'):row['statuses'][msg.reason]+=1
        if isinstance(msg,PointCloud2):row['counts'].append(msg.width*msg.height)
    topics=[('state/odometry',Odometry),('lidar/points',PointCloud2),('perception/semantic',SemanticFrame),
        ('perception/targets_measurements',TargetArray),('perception/tracks',TargetArray),
        ('control/safe_velocity',VelocityCommand),('safety/risk',Risk)]
    subscriptions=[node.create_subscription(cls,'/drone1/'+topic,lambda msg,t=topic:receive(t,msg),qos_profile_sensor_data) for topic,cls in topics]
    subscriptions.append(node.create_subscription(ReferenceOrigin,'/simulation/reference_origin',lambda msg:origins.append(msg.session_id),LATCHED))
    def diag(msg):
        for status in msg.status:
            level=status.level[0] if isinstance(status.level,bytes) else int(status.level)
            diagnostics[status.name]=dict(level=level,message=status.message,values={v.key:v.value for v in status.values})
    subscriptions.append(node.create_subscription(DiagnosticArray,'/diagnostics',diag,10))
    begun=time.perf_counter()
    try:
        while time.perf_counter()-begun<opts.seconds:rclpy.spin_once(node,timeout_sec=.1)
        output=dict(duration_s=time.perf_counter()-begun,session_ids=origins,topics={},diagnostics=diagnostics)
        for topic,row in stats.items():
            arrivals=row['arrivals'];ages=row['ages'];counts=row['counts'];ms=row['inference_ms']
            output['topics'][topic]=dict(samples=len(arrivals),hz=(len(arrivals)-1)/(arrivals[-1]-arrivals[0]) if len(arrivals)>1 else 0,
                max_gap_s=float(np.max(np.diff(arrivals))) if len(arrivals)>1 else None,
                acquisition_age_p50_s=float(np.median(ages)) if ages else None,
                acquisition_age_p95_s=float(np.percentile(ages,95)) if ages else None,
                nonempty_samples=sum(v>0 for v in counts),max_count=max(counts,default=0),statuses=dict(row['statuses']),
                inference_p50_ms=float(np.median(ms)) if ms else None)
        path=Path(opts.output);path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(output,indent=2),encoding='utf8')
        print(json.dumps(output,indent=2))
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
