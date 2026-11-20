"""Four causal replays of identical recorded inputs; never fits estimates to truth.

Uses the same FusionStream as the ROS node. Bag receipt time controls availability;
header stamps control fusion. This is not a whole-trajectory batch smoother.
"""
import argparse
import copy
import json
from pathlib import Path
import time
import numpy as np
import yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from evtol_targeting.common import ns,xyz
from evtol_localization.options import options,fault_at
from evtol_localization.stream import FusionStream
from evtol_localization.geodesy import GeoReference
from evtol_localization.evaluation import ErrorAudit


def replay(events,config,mode,outage,start,end):
    o=options(config.get('localization'));o['mode']=mode
    o['gnss_fault'].update(enabled=outage,segments=[dict(start_s=start,end_s=end,mode='dropout',bias_enu_m=[0.,0.,0.])])
    stream=FusionStream(o);audit=ErrorAudit();origin=datum=geo=None;clock=0;flight_base=None
    published=0;ages=[];stamps=[];started=time.perf_counter();session='';rejected_references=0
    def phase(t):return fault_at(o['gnss_fault'],None if flight_base is None else (t-flight_base)*1e-9)[0]
    for topic,msg,receipt in events:
        now=receipt
        if topic=='/clock':
            now=ns(msg.clock)
            if clock and now<clock:raise ValueError('Bag contains a clock reset; use one session per replay')
            clock=now
        elif topic=='/simulation/reference_origin':
            if session and session!=msg.session_id:raise ValueError('Bag contains multiple sessions')
            origin=msg;session=msg.session_id
        elif topic=='/simulation/geodetic_reference':datum=msg
        if geo is None and origin and datum:
            if origin.source_frame!=datum.header.frame_id:raise ValueError('Geographic and Cartesian references do not match')
            geo=GeoReference(datum.latitude,datum.longitude,datum.altitude,xyz(origin.source_origin_enu))
            if o['gnss_fault']['time_base']=='session':flight_base=ns(origin.header.stamp)
        if topic.endswith('/control/flight_status'):
            f=json.loads(msg.data)
            if f.get('takeoff_stamp_ns'):flight_base=f['takeoff_stamp_ns']
            elif flight_base is None and f.get('phase') in ('takeoff','flying'):flight_base=f.get('stamp_ns') or now
        if geo is None:rejected_references+=1;continue
        if topic.endswith('/imu/data'):
            q=msg.orientation;v=np.array([q.x,q.y,q.z,q.w]);norm=np.linalg.norm(v)
            valid=msg.orientation_covariance[0]>=0 and np.isfinite(v).all() and .5<norm<1.5
            stream.push('imu',ns(msg.header.stamp),dict(accel=xyz(msg.linear_acceleration),gyro=xyz(msg.angular_velocity),q=v/norm if valid else None))
        elif topic.endswith('/gnss/fix'):
            try:
                p=geo.position(msg.latitude,msg.longitude,msg.altitude)
                cov=np.asarray(msg.position_covariance).reshape(3,3) if msg.position_covariance_type else np.eye(3)*o['gnss_sigma_floor_m']**2
                stream.push('gnss',ns(msg.header.stamp),dict(position=p,covariance=cov,valid=msg.status.status>=0))
            except ValueError:stream.reject('gnss','nonfinite_fix')
        elif topic.endswith('/five_ga/observations') and mode=='gnss_imu_5g':
            links=[];seen=set()
            for link in msg.observations:
                if link.gnb_id in seen:continue
                seen.add(link.gnb_id);links.append(dict(id=link.gnb_id,anchor=geo.anchor(xyz(link.base_station_enu)),
                    range=link.measured_range_m,sigma=link.range_sigma_m,rsrp=link.prs_rsrp_dbm,valid=link.measurement_valid))
            stream.push('fiveg',ns(msg.header.stamp),links)
        elif topic.endswith('/ground_truth/local_odometry'):
            q=msg.pose.pose.orientation;audit.truth(ns(msg.header.stamp),xyz(msg.pose.pose.position),np.array([q.x,q.y,q.z,q.w]))
        # Match the node's available simulation time, not future sensor times.
        stream.drain(now,flight_base)
        f=stream.filter
        if f.t is not None and f.t>published and stream.status!='stale':
            audit.estimate(f.t,f.p.copy(),f.q.copy());published=f.t;stamps.append(f.t);ages.append((now-f.t)*1e-9)
        audit.drain(now-int(.55e9),phase)
    if not origin or not geo:raise ValueError('Bag lacks the matching recorded reference messages')
    if flight_base is None and o['gnss_fault']['time_base']=='flight':raise ValueError('Bag lacks flight epoch; use session time base explicitly')
    rows=list(audit.rows)
    window=[r for r in rows if start<=(r['stamp_ns']-flight_base)*1e-9<end] if flight_base else []
    result=dict(mode=mode,outage=outage,source='recorded_airsim_sensor_bag',session_id=session,
        summary=audit.summary(),observations=stream.diagnostic(),fault_base_ns=flight_base,
        window=dict(start_s=start,end_s=end,samples=len(window),
            rmse_3d_m=float(np.sqrt(np.mean([r['error_3d_m']**2 for r in window]))) if window else None,
            end_error_3d_m=window[-1]['error_3d_m'] if window else None),
        state_hz=(len(stamps)-1)/((stamps[-1]-stamps[0])*1e-9) if len(stamps)>1 else 0.,
        state_age_p95_s=float(np.percentile(ages,95)) if ages else None,state_age_max_s=max(ages) if ages else None,
        seconds_compute=time.perf_counter()-started,rows=rows,waiting_reference_events=rejected_references,
        truth_used_for_estimation=False,alignment='session_enu_no_truth_fitting',options=o)
    return result


def main():
    p=argparse.ArgumentParser();p.add_argument('bag',type=Path);p.add_argument('--config',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--outage-start',type=float,default=40.);p.add_argument('--outage-end',type=float,default=60.)
    a=p.parse_args();c=yaml.safe_load(a.config.read_text(encoding='utf-8-sig'))
    reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(a.bag),storage_id=''),rosbag2_py.ConverterOptions('',''))
    types={t.name:get_message(t.type) for t in reader.get_all_topics_and_types()}
    events=[]
    suffixes=('/imu/data','/gnss/fix','/five_ga/observations','/ground_truth/local_odometry','/control/flight_status')
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
