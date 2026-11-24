"""Replay recorded observations through the same branch estimator as live ROS.

Receipt time controls availability, header time controls estimation, and truth
is consumed exclusively by ErrorAudit. No old filter or batch fitting is used.
"""
import argparse
import copy
import json
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np
import yaml
import rclpy
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from evtol_targeting.common import ns, xyz
from evtol_localization.absolute_node import AbsoluteNode
from evtol_localization.options import options, fault_at
from evtol_localization.evaluation import ErrorAudit


def replay(events, config, mode, outage, start, end):
    config = copy.deepcopy(config)
    o = options(config.get('localization'))
    o.update(enabled=True, mode=mode, estimator='absolute')
    o['gnss_fault'].update(enabled=outage, segments=[dict(
        start_s=start, end_s=end, mode='dropout', bias_enu_m=[0., 0., 0.])])
    config['localization'] = o
    audit = ErrorAudit()
    ages, stamps = [], []
    started = time.perf_counter()
    clock = now = 0
    session = ''
    rejected_references = 0
    rclpy.init(domain_id=225)
    node = AbsoluteNode(config)

    def capture(msg):
        t = ns(msg.header.stamp)
        q = msg.pose.pose.orientation
        audit.estimate(t, xyz(msg.pose.pose.position), np.array([q.x, q.y, q.z, q.w]))
        stamps.append(t)
        ages.append((now-t)*1e-9)

    def phase(t):
        base = node.fault_base()
        return fault_at(o['gnss_fault'], (t-base)*1e-9 if base is not None else None)[0]

    node.pub = SimpleNamespace(publish=capture)
    node.path_pub = SimpleNamespace(publish=lambda msg: None)
    statuses = []
    node.status_pub = SimpleNamespace(publish=lambda msg: statuses.append(json.loads(msg.data)))
    try:
        for topic, msg, receipt in events:
            now = receipt
            if topic == '/clock':
                now = ns(msg.clock)
                if clock and now < clock:
                    raise ValueError('Bag contains a clock reset; use one session per replay')
                clock = now
            node.sim_now = now
            if topic == '/simulation/reference_origin':
                if session and session != msg.session_id:
                    raise ValueError('Bag contains multiple sessions')
                session = msg.session_id
                node.on_origin(msg)
            elif topic == '/simulation/geodetic_reference' or topic.endswith('/gnss/home'):
                node.on_home(msg)
            elif topic.endswith('/control/flight_status'):
                node.on_flight(msg)
            if node.frame is None:
                rejected_references += 1
                continue
            if topic.endswith('/imu/data'):
                node.on_imu(msg)
            elif topic.endswith('/gnss/fix'):
                node.on_fix(msg)
            elif topic.endswith('/five_ga/observations') and mode == 'gnss_imu_5g':
                node.on_radio(msg)
            elif topic.endswith('/ground_truth/local_odometry'):
                q = msg.pose.pose.orientation
                audit.truth(ns(msg.header.stamp), xyz(msg.pose.pose.position), np.array([q.x,q.y,q.z,q.w]))
            audit.drain(now-int(.55e9), phase)
        if not session or node.frame is None:
            raise ValueError('Bag lacks matching recorded reference messages')
        base = node.fault_base()
        if base is None:
            raise ValueError('Bag lacks flight epoch; use session time base explicitly')
        node.report()
        rows = list(audit.rows)
        window = [r for r in rows if start <= (r['stamp_ns']-base)*1e-9 < end]
        return dict(mode=mode, outage=outage, estimator='origin/feat/ego-localization',
            source='recorded_airsim_sensor_bag', session_id=session, summary=audit.summary(),
            observations=statuses[-1], fault_base_ns=base,
            window=dict(start_s=start, end_s=end, samples=len(window),
                rmse_3d_m=float(np.sqrt(np.mean([r['error_3d_m']**2 for r in window]))) if window else None,
                end_error_3d_m=window[-1]['error_3d_m'] if window else None),
            state_hz=(len(stamps)-1)/((stamps[-1]-stamps[0])*1e-9) if len(stamps)>1 else 0.,
            state_age_p95_s=float(np.percentile(ages,95)) if ages else None,
            state_age_max_s=max(ages) if ages else None,
            seconds_compute=time.perf_counter()-started, rows=rows,
            waiting_reference_events=rejected_references, truth_used_for_estimation=False,
            alignment='session_enu_no_truth_fitting', options=o)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    p=argparse.ArgumentParser();p.add_argument('bag',type=Path);p.add_argument('--config',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--outage-start',type=float,default=40.);p.add_argument('--outage-end',type=float,default=60.)
    a=p.parse_args();c=yaml.safe_load(a.config.read_text(encoding='utf-8-sig'))
    reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(a.bag),storage_id=''),rosbag2_py.ConverterOptions('',''))
    types={t.name:get_message(t.type) for t in reader.get_all_topics_and_types()}
    events=[]
    suffixes=('/gnss/home','/imu/data','/gnss/fix','/five_ga/observations','/ground_truth/local_odometry','/control/flight_status')
    while reader.has_next():
        topic,data,t=reader.read_next()
        if topic in ('/clock','/simulation/reference_origin','/simulation/geodetic_reference') or topic.endswith(suffixes):events.append((topic,deserialize_message(data,types[topic]),t))
    if not events:raise ValueError('No localization sensor events')
    a.output.mkdir(parents=True,exist_ok=False);reports={}
    for mode in ('gnss_imu','gnss_imu_5g'):
        for outage in (False,True):
            name=mode+('_outage' if outage else '_normal');result=replay(events,c,mode,outage,a.outage_start,a.outage_end)
            rows=result.pop('rows');(a.output/(name+'_errors.jsonl')).write_text(''.join(json.dumps(r)+'\n' for r in rows),encoding='utf8')
            reports[name]=result;print(name,json.dumps({k:result[k] for k in ('summary','window','state_hz','state_age_p95_s')},ensure_ascii=False),flush=True)
    (a.output/'comparison.json').write_text(json.dumps(dict(bag=str(a.bag),events=len(events),runs=reports),indent=2,ensure_ascii=False),encoding='utf8')


if __name__=='__main__':main()
