"""Read-only flight audit: actual waypoint visits, rates, reasons and collisions."""
import argparse
from collections import Counter
import json
from pathlib import Path
import time
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry,Path as NavPath
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from evtol_perception_interfaces.msg import VelocityCommand
from evtol_targeting.common import LATCHED,xyz
from evtol_simulation.rpc import AirSimRPC


def main():
    p=argparse.ArgumentParser();p.add_argument('--config',default='config/targeting.yaml')
    p.add_argument('--seconds',type=float,default=120);p.add_argument('--output',default='reports/navigation_live.json')
    a=p.parse_args();c=yaml.safe_load(Path(a.config).read_text(encoding='utf-8-sig'))
    rclpy.init();node=Node('navigation_read_only_audit');root='/'+c['namespace']
    points=np.asarray(c['mission']['waypoints_enu_m']);minimum=np.full(len(points),np.inf)
    rows=[];reasons=Counter();arrivals=[];counts=[];routes=Counter();mission={};flight={};events=[];last_print=0.
    rpc=AirSimRPC(c['airsim']['host'],c['airsim']['port'],2.)
    before=rpc.call('simGetCollisionInfo',c['airsim']['vehicle'])
    def pose(msg):
        nonlocal minimum,last_print
        position=xyz(msg.pose.pose.position)
        index=mission.get('waypoint_index')
        if index is not None:
            index=min(index,len(points)-1);minimum[index]=min(minimum[index],np.linalg.norm(points[index]-position))
        rows.append(dict(t=time.perf_counter(),position=position.tolist(),velocity_body_flu_mps=xyz(msg.twist.twist.linear).tolist(),
            waypoint_index=mission.get('waypoint_index'),phase=mission.get('phase'),reason=mission.get('reason')))
        if time.perf_counter()-last_print>3:
            print(json.dumps(rows[-1]),flush=True);last_print=time.perf_counter()
    def lidar(msg):arrivals.append(time.perf_counter());counts.append(msg.width*msg.height)
    def status(msg,target):
        value=json.loads(msg.data)
        if target is flight and value.get('phase')!=target.get('phase'):events.append(dict(t=time.perf_counter(),**value))
        target.update(value)
    subscriptions=[node.create_subscription(Odometry,root+'/state/odometry',pose,qos_profile_sensor_data),
        node.create_subscription(PointCloud2,root+'/navigation/lidar/points',lidar,qos_profile_sensor_data),
        node.create_subscription(String,root+'/mission/status',lambda m:status(m,mission),LATCHED),
        node.create_subscription(String,root+'/control/flight_status',lambda m:status(m,flight),LATCHED),
        node.create_subscription(VelocityCommand,root+'/control/safe_velocity',lambda m:reasons.update([m.reason]),qos_profile_sensor_data),
        node.create_subscription(NavPath,root+'/navigation/path',lambda m:routes.update([len(m.poses)]),qos_profile_sensor_data)]
    begin=time.perf_counter()
    try:
        while time.perf_counter()-begin<a.seconds:
            rclpy.spin_once(node,timeout_sec=.1)
            if flight.get('phase')=='landed' and mission.get('phase')=='complete':break
        after=rpc.call('simGetCollisionInfo',c['airsim']['vehicle'])
        report=dict(duration_s=time.perf_counter()-begin,mission=mission,flight=flight,flight_events=events,
            waypoint_min_distance_m=[float(v) if np.isfinite(v) else None for v in minimum],
            lidar_hz=(len(arrivals)-1)/(arrivals[-1]-arrivals[0]) if len(arrivals)>1 else 0.,
            lidar_max_gap_s=float(np.max(np.diff(arrivals))) if len(arrivals)>1 else None,
            lidar_min_points=min(counts,default=0),reasons=dict(reasons),route_sizes=dict(routes),
            collision_before=before,collision_after=after,trajectory=rows)
        path=Path(a.output);path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(report,indent=2),encoding='utf8')
        print(json.dumps({k:v for k,v in report.items() if k!='trajectory'},indent=2))
    finally:rpc.close();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
