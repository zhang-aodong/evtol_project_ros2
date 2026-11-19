"""Start six ROS processes and measure DDS delivery end-to-end.

Run from an activated project shell: python scripts/smoke.py --duration 60
The default backend is mock. Use --backend airsim for live UE acceptance.
"""
import argparse
import os
from pathlib import Path
import subprocess


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--duration',type=float,default=60.)
    parser.add_argument('--backend',choices=('mock','airsim'),default='mock')
    parser.add_argument('--config',type=Path)
    parser.add_argument('--report-prefix',help='Distinct filename prefix for a separate configuration')
    opts=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    reports=root/'reports'
    reports.mkdir(exist_ok=True)
    config=opts.config.resolve() if opts.config else root/'src'/'evtol_simulation'/'config'/'simulation.yaml'
    label=opts.report_prefix or opts.backend
    # Isolate smoke tests from other running project graphs.
    env=dict(os.environ,ROS_DOMAIN_ID='73',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    ros=Path(env['CONDA_PREFIX']).parents[2]/'ros2-windows'/'Scripts'/'ros2.exe'
    if not ros.is_file(): raise RuntimeError('Activate scripts/activate.ps1 first')
    with (reports/(label+'_launch.log')).open('w',encoding='utf8') as log:
        process=subprocess.Popen([str(ros),'launch','evtol_bringup','simulation.launch.py',
            'backend:='+opts.backend,'rviz:=false','config:='+str(config)],env=env,
            stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
        try:
            command=[str(ros),'run','evtol_simulation','validate_sensors',
                '--config',str(config),'--warmup','12','--duration',str(opts.duration),
                '--output',str(reports/(label+'_acceptance.json'))]
            if opts.backend=='mock': command.append('--allow-mock')
            result=subprocess.run(command,env=env,
                timeout=opts.duration+50)
            return result.returncode
        finally:
            # Only terminate the process tree created by this test harness.
            if process.poll() is None:
                subprocess.run(['taskkill','/PID',str(process.pid),'/T','/F'],
                    stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,check=False)
                process.wait(timeout=10)


if __name__=='__main__': raise SystemExit(main())
