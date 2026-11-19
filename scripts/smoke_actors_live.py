"""Explicit live simulation test: move only Drone2 and Pedestrian_2, then land."""
import json
import time
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger
from evtol_targeting.common import args_config,project_path
from evtol_simulation.rpc import AirSimRPC
from evtol_simulation.coordinates import vector
from evtol_simulation.runtime import TimerResolution
from evtol_scenario.actors import Actors


def main():
    c,ros=args_config();resolution=TimerResolution();rclpy.init(args=ros)
    settings=yaml.safe_load(project_path(c,c['actors_config']).read_text(encoding='utf8'))
    settings['interference_drones']['enabled']=True
    agent=next(a for a in settings['interference_drones']['agents'] if a['vehicle_name']=='Drone2')
    agent=dict(agent,coordinate_mode='relative_initial',speed_mps=1.,repeat=1,start_delay_s=0.,
        waypoints_frd_m=[[0.,0.,-3.],[0.,1.,-3.]])
    settings['interference_drones']['agents']=[agent]
    settings['interference_drones']['land_on_stop']=True
    settings['pedestrians']['enabled']=True
    settings['pedestrians']['agents']=[next(a for a in settings['pedestrians']['agents'] if a['name']=='Pedestrian_2')]
    fixture=project_path(c,'reports/actors_smoke_fixture.yaml');fixture.parent.mkdir(parents=True,exist_ok=True)
    fixture.write_text(yaml.safe_dump(settings),encoding='utf8');c['actors_config']=str(fixture)
    rpc=AirSimRPC(c['airsim']['host'],c['airsim']['port'],3.)
    p0=vector(rpc.call('simGetObjectPose','Pedestrian_2')['position'])
    d0=vector(rpc.call('simGetGroundTruthKinematics','Drone2')['position'])
    node=Actors(c);client_node=Node('actors_smoke_test_client');executor=SingleThreadedExecutor()
    executor.add_node(node);executor.add_node(client_node)
    try:
        begun=time.perf_counter();positions=[]
        while time.perf_counter()-begun<35.:
            executor.spin_once(timeout_sec=.05)
            if not positions or time.perf_counter()-positions[-1][0]>.5:
                positions.append((time.perf_counter(),vector(rpc.call('simGetGroundTruthKinematics','Drone2')['position']).tolist()))
        p1=vector(rpc.call('simGetObjectPose','Pedestrian_2')['position'])
        client=client_node.create_client(Trigger,'/scenario/stop')
        if not client.wait_for_service(timeout_sec=2.):raise RuntimeError('Stop service missing')
        future=client.call_async(Trigger.Request())
        while not future.done():executor.spin_once(timeout_sec=.1)
        response=future.result();landed=rpc.call('getMultirotorState','Drone2')['landed_state']
        report=dict(session_received=node.origin is not None,started=node.started,
            drone_initial_ned=d0.tolist(),drone_samples_ned=[p for _,p in positions],
            drone_max_displacement_m=float(max(np.linalg.norm(np.asarray(p)-d0) for _,p in positions)),
            drone_max_horizontal_displacement_m=float(max(np.linalg.norm((np.asarray(p)-d0)[:2]) for _,p in positions)),
            pedestrian_initial_ned=p0.tolist(),pedestrian_final_ned=p1.tolist(),pedestrian_displacement_m=float(np.linalg.norm(p1-p0)),
            cleanup_success=response.success,cleanup_message=response.message,landed_state=landed,
            note='Moves Drone2 and Pedestrian_2 only; Drone1 gateway remains shadow')
        path=project_path(c,'reports/actors_live_smoke.json');path.write_text(json.dumps(report,indent=2),encoding='utf8')
        print(json.dumps(report,indent=2))
        if not response.success or landed!=0 or report['drone_max_displacement_m']<1. or report['drone_max_horizontal_displacement_m']<.3 or report['pedestrian_displacement_m']<.1:
            raise RuntimeError('Live actor movement/cleanup checks failed')
    finally:
        node.close();executor.shutdown();node.destroy_node();client_node.destroy_node();rpc.close();rclpy.shutdown();resolution.close()

if __name__=='__main__':main()
