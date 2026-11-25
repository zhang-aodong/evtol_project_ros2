"""Explicitly start a new origin session before takeoff; never erase live state."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import yaml


p=argparse.ArgumentParser();p.add_argument('--config',required=True);p.add_argument('--new-session',action='store_true');opts=p.parse_args()
config=Path(opts.config).resolve();c=yaml.safe_load(config.read_text(encoding='utf-8-sig'))
if opts.new_session:
    script="Get-CimInstance Win32_Process -Filter \"Name='python.exe'\" | Select-Object ProcessId,CommandLine | ConvertTo-Json -Compress"
    result=subprocess.run(['powershell.exe','-NoProfile','-Command',script],capture_output=True,text=True,check=True)
    if result.stdout.strip():
        records=json.loads(result.stdout);records=records if isinstance(records,list) else [records]
        if any('from evtol_targeting.state_node import main' in str(r.get('CommandLine','')) for r in records):
            raise RuntimeError('Stop the active state adapter before resetting the origin')
    origin=Path(c['state']['origin_file'])
    if not origin.is_absolute():origin=Path(c.get('project_root',config.parent.parent))/origin
    if origin.exists():
        archive=origin.with_name(origin.stem+'.previous.json');origin.replace(archive)
        print('Previous origin archived to',archive)
    print('Next truth sample will define the new initial origin; do this before takeoff.')
