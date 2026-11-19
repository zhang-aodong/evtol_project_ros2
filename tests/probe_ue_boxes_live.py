"""Explicit live UE overlay smoke test, isolated domain 224; no vehicle commands.

Publishes synthetic detections (NOT model output) for five seconds, checks real
UE plot RPC replies, then empties detections and verifies no further plot calls.
Run only with a visible UE game instance already running.
"""
import json
from pathlib import Path
import time
import numpy as np
import rclpy
import yaml
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from evtol_perception_interfaces.msg import ReferenceOrigin, TargetArray, Target
from evtol_simulation.coordinates import NED_ENU
from evtol_simulation.rpc import AirSimRPC
from evtol_targeting.common import LATCHED, header, put, stamp
from evtol_targeting.ue_detection_boxes import UeDetectionBoxes


def main():
    root=Path(__file__).resolve().parents[1]
    c=yaml.safe_load((root/'config/targeting.yaml').read_text(encoding='utf-8-sig'))
    rclpy.init(domain_id=224)
    # Drive real subscriptions, then invoke the same real renderer deterministically.
    overlay=UeDetectionBoxes(c,start_worker=False)
    source=rclpy.create_node('synthetic_ue_box_smoke_test')
    executor=SingleThreadedExecutor();executor.add_node(source);executor.add_node(overlay)
    origin_pub=source.create_publisher(ReferenceOrigin,'/simulation/reference_origin',LATCHED)
    clock_pub=source.create_publisher(Clock,'/clock',qos_profile_sensor_data)
    detections=source.create_publisher(TargetArray,'/drone1/perception/tracks',qos_profile_sensor_data)
    rpc=AirSimRPC(c['airsim']['host'],c['airsim']['port'],2.)
    calls=[]
    class CheckedRpc:
        def call(self,method,*args):
            assert method in ('simGetVehiclePose','simGetObjectPose','simPlotLineList','simPlotStrings'),method
            reply=rpc.call(method,*args);calls.append(method);return reply
    checked=CheckedRpc()
    try:
        def pos(method):
            p=rpc.call(method,'Drone1')['position']
            return np.array([p[k] for k in ('x_val','y_val','z_val')])
        local=pos('simGetVehiclePose')
        origin=ReferenceOrigin(header=header(1_000_000_000),session_id='synthetic-ue-box-smoke-test')
        put(origin.source_origin_enu,NED_ENU@local);origin_pub.publish(origin)
        # Ahead and above the stationary ego, suitable for the chase view.
        begun=time.perf_counter();next_draw=begun
        while time.perf_counter()-begun<5:
            now=time.perf_counter();t=1_000_000_000+int((now-begun)*1e9)
            clock_pub.publish(Clock(clock=stamp(t)))
            target=Target(track_id=999,class_id=6,state='confirmed',last_measurement_stamp=stamp(t))
            put(target.position,[0.,5.,1.]);put(target.size,[1.5,1.5,1.])
            detections.publish(TargetArray(header=header(t),session_id=origin.session_id,targets=[target]))
            executor.spin_once(timeout_sec=.01)
            if now>=next_draw:
                overlay.render_once(checked);next_draw=now+.1
        count=overlay.draw_calls
        assert count>=10,('Insufficient completed draw calls',count)
        offset=overlay.spawn
        clear_begin=time.perf_counter()
        while time.perf_counter()-clear_begin<.5:
            t+=10_000_000
            clock_pub.publish(Clock(clock=stamp(t)))
            detections.publish(TargetArray(header=header(t),session_id=origin.session_id))
            executor.spin_once(timeout_sec=.01)
        overlay.render_once(checked);assert overlay.last_box_count==0
        assert overlay.draw_calls==count
        report=dict(result='PASS_RPC_AND_ROS',fixture='synthetic detection, not actual model output',
            domain=224,draw_calls=count,global_ned_offset_m=list(offset),empty_detection_stops_drawing=True,
            methods=sorted(set(calls)),visual_window_check='Unavailable: computer-use native pipe missing')
        output=root/'reports/ue_boxes_live.json';output.write_text(json.dumps(report,indent=2),encoding='utf8')
        print(json.dumps(report,indent=2))
    finally:
        rpc.close();executor.shutdown();source.destroy_node();overlay.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
