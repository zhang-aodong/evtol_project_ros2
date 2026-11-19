"""Read live SHM counters and publisher diagnostics without starting capture."""
import argparse
import json
from pathlib import Path
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from evtol_simulation.configuration import load
from evtol_simulation.camera_ring import SharedMemoryCameraReader
from evtol_simulation.lidar_ring import LidarSharedMemory


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--config',default='src/evtol_simulation/config/simulation.source_2k.yaml')
    parser.add_argument('--duration',type=float,default=10.)
    parser.add_argument('--output',default='reports/pipeline_probe.json')
    opts=parser.parse_args()
    if opts.duration<2:parser.error('duration must be at least 2 seconds')
    config=load(opts.config)
    rclpy.init();node=rclpy.create_node('pipeline_probe')
    readers={};health={}
    def callback(msg):
        wall=time.perf_counter()
        for status in msg.status:
            if status.name.startswith('/'+config['namespace']+'/') and any(v.key=='samples' for v in status.values):
                values={v.key:v.value for v in status.values}
                record=health.setdefault(status.name,{'first':values,'first_wall':wall})
                record.update(last=values,last_wall=wall)
    node.create_subscription(DiagnosticArray,'/diagnostics',callback,10)
    try:
        camera=config['camera']
        if camera['transport']=='shared_memory':
            stream=config['airsim']['vehicle']+':'+camera['name']
            key=''.join(c if c.isalnum() else '_' for c in stream)
            for image_type in (0,5):
                name=f"Local\\AirSim_{key}_{camera['width']}x{camera['height']}_{image_type}"
                readers['camera_'+str(image_type)]=SharedMemoryCameraReader(name,camera['width'],camera['height'],camera['slots'])
        if config['lidar']['transport']=='shared_memory':
            lidar=config['lidar']
            readers['lidar']=LidarSharedMemory(lidar['mapping_name'],lidar['max_points'],lidar['slots'])
        def ids():return {k:(r.latest_id() if k=='lidar' else r.latest_frame_id()) for k,r in readers.items()}
        first=previous=ids();backwards={k:0 for k in first};start=time.perf_counter()
        while time.perf_counter()-start<opts.duration:
            rclpy.spin_once(node,timeout_sec=.02)
            current=ids()
            for key in current:
                if current[key]<previous[key]:backwards[key]+=1
            previous=current
        duration=time.perf_counter()-start
        source={k:{'first_id':first[k],'last_id':previous[k],
                   'hz':(previous[k]-first[k])/duration,'counter_regressions':backwards[k]} for k in first}
        for record in health.values():
            dt=record.pop('last_wall')-record.pop('first_wall')
            record['window_hz']=(int(record['last']['samples'])-int(record['first']['samples']))/dt if dt else None
            record['new_errors']=int(record['last'].get('errors',0))-int(record['first'].get('errors',0))
        result={'duration_s':duration,'clock_publishers':len(node.get_publishers_info_by_topic('/clock')),
                'shared_memory_source':source,'publisher_diagnostics':health}
        target=Path(opts.output);target.parent.mkdir(parents=True,exist_ok=True)
        target.write_text(json.dumps(result,indent=2),encoding='utf8')
        print(json.dumps(result,indent=2))
    finally:
        for reader in readers.values():reader.close()
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
