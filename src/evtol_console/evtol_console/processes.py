"""Own only processes created by this console; no broad name-based kills."""
import os
from pathlib import Path
import signal
import subprocess
import time
import threading


class OwnedProcess:
    def __init__(self,name,args,root,log,visible=False):
        self.name=name;self.log=Path(log);self.log.parent.mkdir(parents=True,exist_ok=True)
        self.stop_lock=threading.Lock()
        self.file=self.log.open('ab',buffering=0)
        kwargs={}
        if name!='ue':
            kwargs['env']=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
        if os.name=='nt':
            kwargs['creationflags']=subprocess.CREATE_NEW_PROCESS_GROUP|(0 if visible else subprocess.CREATE_NO_WINDOW)
            if not visible:
                startup=subprocess.STARTUPINFO();startup.dwFlags|=subprocess.STARTF_USESHOWWINDOW;startup.wShowWindow=0
                kwargs['startupinfo']=startup
        else:kwargs['start_new_session']=True
        try:self.process=subprocess.Popen(args,cwd=root,stdout=self.file,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,**kwargs)
        except BaseException:self.file.close();raise

    def state(self):
        code=self.process.poll()
        return dict(name=self.name,pid=self.process.pid,running=code is None,exit_code=code,log=str(self.log))

    def stop(self):
        with self.stop_lock:self._stop()

    def _stop(self):
        if self.process.poll() is None:
            if os.name=='nt':
                # Arguments are fixed and PID comes from our still-live Popen.
                result=subprocess.run(['taskkill.exe','/PID',str(self.process.pid),'/T','/F'],capture_output=True,timeout=5,
                    creationflags=subprocess.CREATE_NO_WINDOW)
                if result.returncode and self.process.poll() is None:raise RuntimeError('Cannot stop owned '+self.name)
            else:os.killpg(self.process.pid,signal.SIGTERM)
            self.process.wait(timeout=5)
        self.file.close()


def node_specs(c,config,simconfig,run):
    specs=[(k+'_sensor','evtol_simulation.sensor_node',['--config',str(simconfig),'--kind',k,'--backend','airsim'])
        for k in ('truth','camera','lidar','imu','gnss','radio')]
    algorithms=[('state','evtol_targeting.state_node',[]),('trajectory','evtol_targeting.trajectory_node',[]),
        ('vision','evtol_targeting.vision_node',[]),('localizer','evtol_targeting.localizer_node',[]),
        ('safety','evtol_safety.node',[]),('warnings','evtol_safety.warning_node',[]),('gateway','evtol_safety.gateway',['--execute'] if run['execute'] else [])]
    if c.get('surround',{}).get('enabled',False):
        detect=c['model'].get('task')=='detect'
        if detect or not c['surround'].get('front_diagnostic_inference',False):algorithms=[s for s in algorithms if s[0]!='vision']
        fusion='evtol_targeting.detection_fusion' if detect else 'evtol_targeting.surround_fusion'
        algorithms=[(name, fusion if name=='localizer' else module,extra) for name,module,extra in algorithms]
        algorithms.insert(0,('perception_lidar','evtol_simulation.perception_lidar',[]))
        if c['surround'].get('roi_lidar',False):algorithms.insert(0,('roi_lidar','evtol_simulation.roi_lidar',[]))
        algorithms.insert(next(i for i,s in enumerate(algorithms) if s[0]=='localizer'),('surround_vision','evtol_targeting.surround_vision',[]))
    if c.get('navigation_lidar',{}).get('enabled',False):algorithms.insert(0,('navigation_lidar','evtol_simulation.safety_lidar',[]))
    if c.get('navigation_lidar',{}).get('enabled',False) and c.get('lidar_obstacles',{}).get('enabled',True):
        algorithms.insert(next(i for i,s in enumerate(algorithms) if s[0]=='safety'),('lidar_obstacles','evtol_targeting.lidar_obstacles',[]))
    if c.get('ue_detection_boxes',{}).get('enabled',True):algorithms.append(('ue_boxes','evtol_targeting.ue_detection_boxes',[]))
    localization=c.get('localization',{})
    if localization.get('enabled',False) or c['state']['source']=='fused':
        algorithms.insert(0,('ego_localization','evtol_localization.node',[]))
        if localization.get('evaluate',True):algorithms.append(('ego_localization_evaluation','evtol_localization.evaluation_node',[]))
    if run['actors']:algorithms.insert(next(i for i,s in enumerate(algorithms) if s[0]=='gateway'),('actors','evtol_scenario.actors',[]))
    if run['evaluate']:algorithms.extend([('target_truth','evtol_targeting.evaluation',['--truth']),('evaluation','evtol_targeting.evaluation',[]),('range_evaluation','evtol_targeting.experiment_node',[])])
    for name,module,extra in algorithms:specs.append((name,module,['--config',str(config)]+extra+['--ros-args','-p','use_sim_time:=true']))
    return specs
