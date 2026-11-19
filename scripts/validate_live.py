"""Independent bulk-image and IMU observers; unchanged acceptance limits."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--config',required=True)
    parser.add_argument('--duration',type=float,default=60.)
    parser.add_argument('--warmup',type=float,default=10.)
    parser.add_argument('--allow-mock',action='store_true')
    parser.add_argument('--output',default='reports/live_acceptance.json')
    opts=parser.parse_args()
    target=Path(opts.output).resolve();target.parent.mkdir(parents=True,exist_ok=True)
    processes=[];logs=[]
    try:
        for scope in ('core','rgb','semantic','labels','imu'):
            output=target.with_name(target.stem+'_'+scope+'.json')
            log=output.with_suffix('.log').open('w',encoding='utf8');logs.append(log)
            command=[sys.executable,'-c','from evtol_simulation.validation import main; main()',
                '--config',str(Path(opts.config).resolve()),'--duration',str(opts.duration),
                '--warmup',str(opts.warmup),'--scope',scope,'--output',str(output)]
            if opts.allow_mock:command.append('--allow-mock')
            process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,
                creationflags=subprocess.CREATE_NO_WINDOW if os.name=='nt' else 0)
            processes.append((scope,process,output))
        for scope,process,output in processes:
            code=process.wait(timeout=opts.duration+opts.warmup+45)
            if code not in (0,1) or not output.exists():
                raise RuntimeError(f'{scope} monitor failed; see {output.with_suffix(".log")}')
        main_report=json.loads(processes[0][2].read_text(encoding='utf8'))
        reports=[json.loads(record[2].read_text(encoding='utf8')) for record in processes]
        pairs={}
        for report in reports:
            for topic,stream in report['streams'].items():
                # Keep the core observer's clock; every observer still checks it.
                if topic!='/clock':main_report['streams'][topic]=stream
            for topic,stamps in report.pop('pairing_stamps_ns',{}).items():
                if stamps:pairs[topic]=stamps
        main_report.pop('pairing_stamps_ns',None)
        main_report['publisher_diagnostics']['imu']=reports[-1]['publisher_diagnostics'].get('imu',{})
        problems=list(dict.fromkeys(p for report in reports for p in report['problems']))
        rgb=pairs.get('camera/rgb/image_raw',[]);sem=pairs.get('camera/semantic/image',[]);lidar=pairs.get('lidar/points',[])
        # Trim independent observer boundary differences before exact pairing.
        low=max(rgb[0],sem[0]) if rgb and sem else 0
        high=min(rgb[-1],sem[-1]) if rgb and sem else -1
        rgb_set={t for t in rgb if low<=t<=high};sem_set={t for t in sem if low<=t<=high}
        if not rgb_set or not sem_set or len(rgb_set&sem_set)<.95*min(len(rgb_set),len(sem_set)):
            problems.append('RGB/semantic exact timestamp pairing failed')
        interior=[t for t in rgb if lidar and lidar[0]<=t<=lidar[-1]]
        delta=max((min(abs(t-l) for l in lidar)*1e-6 for t in interior),default=float('inf'))
        from evtol_simulation.configuration import load
        if delta>load(Path(opts.config))['acceptance']['max_sync_delta_ms']:
            problems.append('Camera/LiDAR acquisition synchronization exceeds tolerance')
        main_report['max_camera_lidar_delta_ms']=delta if delta!=float('inf') else None
        main_report['problems']=problems
        main_report['passed']=all(report['passed'] for report in reports) and not problems
        main_report['frequency_passed']=all(s['rate_passed'] for s in main_report['streams'].values())
        main_report['result']=('PASS_MOCK' if opts.allow_mock else 'PASS_AIRSIM') if main_report['passed'] else 'FAIL'
        main_report['observer']='independent core/RGB/semantic/labels/IMU processes; original acceptance limits'
        target.write_text(json.dumps(main_report,indent=2,ensure_ascii=False),encoding='utf8')
        print(json.dumps(main_report,indent=2,ensure_ascii=False))
        return 0 if main_report['passed'] else 1
    finally:
        for _,process,_ in processes:
            if process.poll() is None:process.terminate();process.wait(timeout=10)
        for log in logs:log.close()


if __name__=='__main__':raise SystemExit(main())
