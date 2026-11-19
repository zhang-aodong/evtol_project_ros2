"""Read-only check that mission and traveled Path messages match live state."""
import argparse
import json
from pathlib import Path
import time
import rclpy
from nav_msgs.msg import Path as NavPath,Odometry
from rclpy.qos import qos_profile_sensor_data
from evtol_targeting.common import LATCHED,ns


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--seconds',type=float,default=15.)
    parser.add_argument('--namespace',default='drone1')
    parser.add_argument('--output',default='reports/trajectory_live.json')
    args=parser.parse_args()
    rclpy.init();node=rclpy.create_node('trajectory_readonly_audit')
    root='/'+args.namespace.strip('/');paths={};poses={};samples={};checks=[]
    def receive_pose(msg):
        p=msg.pose.pose.position
        poses[ns(msg.header.stamp)]=(p.x,p.y,p.z)
        if len(poses)>2000:poses.pop(next(iter(poses)))
    def receive_path(key,msg):
        paths[key]=msg;samples[key]=samples.get(key,0)+1
        if key=='traveled' and msg.poses:
            p=msg.poses[-1];observed=poses.get(ns(p.header.stamp))
            if observed is not None:
                actual=(p.pose.position.x,p.pose.position.y,p.pose.position.z)
                checks.append(actual==observed)
    subscriptions=[node.create_subscription(Odometry,root+'/state/odometry',receive_pose,qos_profile_sensor_data)]
    subscriptions += [node.create_subscription(NavPath,root+'/navigation/'+suffix,
        lambda msg,key=key:receive_path(key,msg),LATCHED)
        for key,suffix in [('waypoints','waypoint_path'),('traveled','traveled_path')]]
    begin=time.perf_counter()
    try:
        while time.perf_counter()-begin<args.seconds:rclpy.spin_once(node,timeout_sec=.05)
        report=dict(duration_s=time.perf_counter()-begin,samples=samples,
            actual_endpoint_checks=len(checks),actual_endpoints_match=bool(checks) and all(checks),paths={})
        for key,msg in paths.items():
            report['paths'][key]=dict(frame=msg.header.frame_id,points=len(msg.poses),
                positions=[[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in msg.poses])
        report['passed']=(set(paths)=={'waypoints','traveled'} and bool(checks) and all(checks)
            and all(msg.header.frame_id=='map_enu' and msg.poses for msg in paths.values()))
        target=Path(args.output);target.parent.mkdir(parents=True,exist_ok=True)
        target.write_text(json.dumps(report,indent=2),encoding='utf8')
        print(json.dumps({k:v for k,v in report.items() if k!='paths'},indent=2))
        return 0 if report['passed'] else 1
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
