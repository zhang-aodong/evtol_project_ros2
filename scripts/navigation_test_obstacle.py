"""Explicit static test fixture; run before takeoff, remove after the test."""
import argparse
import json
from pathlib import Path
import numpy as np
import yaml
from evtol_simulation.rpc import AirSimRPC
from evtol_simulation.coordinates import vector

p=argparse.ArgumentParser();p.add_argument('--config',default='config/targeting.yaml');p.add_argument('--remove',action='store_true');a=p.parse_args()
c=yaml.safe_load(Path(a.config).read_text(encoding='utf-8-sig'));s=c['airsim'];name='EvtolNavigationTestCube'
r=AirSimRPC(s['host'],s['port'],3.)
try:
    if a.remove:print('removed',r.call('simDestroyObject',name))
    else:
        if r.call('isApiControlEnabled',s['vehicle']):raise RuntimeError('Stop the experiment and place the vehicle on the ground before spawning the fixture')
        kin=r.call('getMultirotorState',s['vehicle'])['kinematics_estimated']
        if np.linalg.norm(vector(kin['linear_velocity']))>.1:raise RuntimeError('Vehicle must be stationary on the ground')
        if name in r.call('simListSceneObjects',name):raise RuntimeError('Fixture already exists; remove it first')
        pos=kin['position'];pose=dict(position=dict(x_val=pos['x_val']+3.,y_val=pos['y_val'],z_val=pos['z_val']-3.),orientation=dict(x_val=0.,y_val=0.,z_val=0.,w_val=1.))
        print('spawned',r.call('simSpawnObject',name,'EditorCube',pose,dict(x_val=1.,y_val=1.,z_val=1.),False,False))
        print(json.dumps(pose))
finally:r.close()
