"""Single-worker orchestration; SDK/HTTP callbacks enqueue bounded operations."""
import copy
import hashlib
import json
import os
from pathlib import Path
import queue
import socket
import subprocess
import sys
import threading
import time
import uuid
from concurrent.futures import ThreadPoolExecutor,as_completed
import yaml
from evtol_simulation.rpc import AirSimRPC
from .configuration import defaults,validate,artifacts,atomic
from .processes import OwnedProcess,node_specs


class Engine:
    def __init__(self,root,ros_stop=None,process_factory=OwnedProcess):
        self.root=Path(root).resolve();self.file=self.root/'config/console.local.json'
        self.config=json.loads(self.file.read_text(encoding='utf-8-sig')) if self.file.exists() else defaults(self.root)
        validate(self.config,self.root)
        self.ros_stop=ros_stop;self.factory=process_factory;self.lock=threading.RLock();self.queue=queue.Queue(maxsize=1)
        self.closed=threading.Event();self.abort=threading.Event();self.operations=[];self.logs=[];self.processes={}
        self.force_event=threading.Event();self.force_running=False;self.force_thread=None
        self.log_file=self.root/'reports'/('console_manager_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]+'.jsonl')
        self.log_file.parent.mkdir(parents=True,exist_ok=True);self.log_sequence=0
        self.log_offsets={};self.log_pending={};self.observer_stop=threading.Event()
        self.active=None;self.ue_snapshot=None;self.ue_fingerprint=None;self.telemetry={};self.busy=False;self.last_health=0.;self.airsim_ready=False
        self.worker=threading.Thread(target=self.work,name='console_operations',daemon=True);self.worker.start()
        self.observer=threading.Thread(target=self.observe,name='console_logs',daemon=True);self.observer.start()

    def log(self,message,source='manager',level='INFO'):
        with self.lock:
            self.log_sequence+=1
            event=dict(id=self.log_sequence,time=time.strftime('%H:%M:%S'),wall=time.time(),source=source,level=level,message=str(message)[:4000])
            self.logs.append(event);self.logs=self.logs[-500:]
            try:
                if self.log_file.exists() and self.log_file.stat().st_size>20_000_000:self.log_file.replace(self.log_file.with_suffix('.previous.jsonl'))
                with self.log_file.open('a',encoding='utf8') as f:f.write(json.dumps(event,ensure_ascii=False)+'\n')
            except OSError:pass

    def observe(self):
        last_progress=0.
        while not self.observer_stop.wait(.5):
            try:
                self.read_process_logs()
                if time.monotonic()-last_progress>5.:
                    last_progress=time.monotonic()
                    with self.lock:running=[dict(op) for op in self.operations if op['state']=='running']
                    for op in running:self.log(f"执行中：{op['action']}，已耗时 {time.time()-op.get('started_wall',time.time()):.1f}s")
            except Exception as exc:self.log('Log collector: '+str(exc),level='WARN')

    def read_process_logs(self):
        with self.lock:owned=list(self.processes.items())
        for name,p in owned:
            path=Path(p.log);key=str(path)
            if not path.exists():continue
            offset=self.log_offsets.get(key,0);size=path.stat().st_size
            if size<offset:offset=0
            # Bound reads even if a noisy child generates megabytes per second.
            if size-offset>32768:offset=size-32768;self.log_pending.pop(key,None)
            with path.open('rb') as f:f.seek(offset);data=f.read(32768);self.log_offsets[key]=f.tell()
            if not data:continue
            chunks=(self.log_pending.pop(key,b'')+data).split(b'\n');self.log_pending[key]=chunks.pop()[-4000:]
            for raw in chunks[-80:]:
                line=raw.decode('utf8',errors='replace').strip()
                if line:self.log(line,name,'ERROR' if 'ERROR' in line or 'Traceback' in line else 'WARN' if 'WARN' in line else 'INFO')

    def status(self):
        with self.lock:
            states={name:p.state() for name,p in self.processes.items()}
            return dict(config=copy.deepcopy(self.config),processes=states,busy=self.busy,
                airsim_ready=self.airsim_ready,ue_owned='ue' in states and states['ue']['running'],
                ros_running=any(v['running'] for k,v in states.items() if k!='ue'),
                operations=copy.deepcopy(self.operations[-25:]),logs=copy.deepcopy(self.logs[-300:]),
                force_running=self.force_running,log_file=str(self.log_file),
                telemetry=copy.deepcopy(self.telemetry),active_run=str(self.active) if self.active else None,
                root=str(self.root),service='/simulation/console')

    def request(self,request):
        try:
            if not isinstance(request,dict):raise ValueError('Request must be an object')
            action=request.get('action');payload=request.get('payload',{})
            if action not in ('status','preview'):self.log('收到界面请求：'+str(action),source='button')
            if action=='status':return dict(ok=True,data=self.status())
            if action=='preview':
                d,route,compiled=validate(payload,self.root)
                return dict(ok=True,data=dict(waypoints_enu_m=route,profile=compiled))
            if action=='save':
                with self.lock:
                    if self.busy or self.force_running or any(p.state()['running'] for name,p in self.processes.items() if name!='ue'):
                        raise ValueError('Stop ROS before applying startup settings; edits remain in the panel draft')
                    d,base,scenario,ue,compiled=artifacts(payload,self.root)
                    if self.processes.get('ue') and self.processes['ue'].state()['running']:
                        if self.fingerprint(ue)!=self.ue_fingerprint or d['ue']!=self.config['ue']:
                            raise ValueError('UE paths, window mode, obstacle count or spawn changed; close UE before saving these changes')
                    atomic(self.file,d);self.config=d
                self.log('Configuration saved. Base targeting.yaml and actors.yaml were not overwritten.')
                return dict(ok=True,data=self.status())
            if action=='cancel':
                self.abort.set();self.log('取消请求已发送');return dict(ok=True,data={'message':'Cancellation requested; owned processes will be cleaned up'})
            if action=='force_stop_all':return self.request_force()
            if action not in ('start_ue','stop_ue','start_ros','stop_ros','stop_all'):raise ValueError('Unsupported action')
            with self.lock:
                if self.busy or self.force_running:raise ValueError('Another operation is in progress')
                self.busy=True;self.abort.clear();self.force_event.clear();op=dict(id=uuid.uuid4().hex,action=action,state='queued',message='',started_wall=time.time())
                self.operations.append(op);self.queue.put_nowait(op)
            return dict(ok=True,data=dict(operation_id=op['id']))
        except Exception as exc:
            self.log('请求失败：'+str(exc),level='ERROR');return dict(ok=False,error=str(exc))

    def request_force(self):
        with self.lock:
            if self.force_running:raise ValueError('强制结束正在执行')
            self.force_running=True;self.force_event.set();self.abort.set()
            op=dict(id=uuid.uuid4().hex,action='force_stop_all',state='running',message='并行结束自有 ROS / UE 进程树',started_wall=time.time())
            self.operations.append(op)
            self.force_thread=threading.Thread(target=self.force_stop,args=(op,),name='console_force_stop',daemon=True)
            self.force_thread.start()
        return dict(ok=True,data=dict(operation_id=op['id']))

    def terminate(self,owned):
        errors=[]
        with ThreadPoolExecutor(max_workers=max(1,min(32,len(owned)))) as pool:
            futures={pool.submit(p.stop):(name,p) for name,p in owned}
            for future in as_completed(futures):
                name,p=futures[future]
                try:
                    future.result()
                    with self.lock:
                        if self.processes.get(name) is p:self.processes.pop(name,None)
                    self.log('进程树已结束：'+name+' PID '+str(p.process.pid))
                except Exception as exc:
                    errors.append(name+': '+str(exc));self.log(errors[-1],level='ERROR')
        if errors:raise RuntimeError('; '.join(errors))

    def force_stop(self,op):
        self.log('强制结束开始：跳过悬停、降落和 ROS 清理服务',level='WARN')
        try:
            with self.lock:owned=list(self.processes.items())
            self.terminate(owned)
            with self.lock:
                self.airsim_ready=False;self.ue_fingerprint=None
                op.update(state='done',message='自有 ROS / UE 进程树已结束')
            self.log(op['message'])
        except Exception as exc:
            with self.lock:op.update(state='failed',message=str(exc))
            self.log(str(exc),level='ERROR')
        finally:
            with self.lock:self.force_running=False;op['finished_wall']=time.time()

    def work(self):
        while not self.closed.is_set():
            try:op=self.queue.get(timeout=.2)
            except queue.Empty:
                self.monitor();continue
            with self.lock:op['state']='running'
            self.log('开始执行：'+op['action'])
            try:
                if self.force_event.is_set():raise RuntimeError('操作被强制结束中断')
                getattr(self,op['action'])()
                if self.force_event.is_set():raise RuntimeError('普通操作已被强制结束接管')
                with self.lock:op['state']='done';op['message']='Completed'
                self.log(op['action']+' completed')
            except Exception as exc:
                with self.lock:op['state']='cancelled' if self.abort.is_set() else 'failed';op['message']=str(exc)
                self.log(op['action']+': '+str(exc))
            finally:
                with self.lock:self.busy=False;op['finished_wall']=time.time()

    def monitor(self):
        if time.monotonic()-self.last_health<2:return
        self.last_health=time.monotonic()
        rpc=self.rpc(.3)
        try:ready=bool(rpc.call('ping'))
        except Exception:ready=False
        finally:rpc.close()
        with self.lock:self.airsim_ready=ready
        with self.lock:owned=list(self.processes.items())
        failed=[k for k,p in owned if k!='ue' and p.process.poll() is not None]
        if failed and any(p.process.poll() is None for k,p in owned if k!='ue'):
            self.log('A ROS child exited: '+', '.join(failed)+'; stopping the owned stack')
            with self.lock:self.busy=True
            try:self.stop_ros()
            except Exception as exc:self.log(exc)
            finally:
                with self.lock:self.busy=False

    def rpc(self,timeout=1.):
        from .configuration import read_yaml
        connection=read_yaml(self.root/'config/targeting.yaml')['airsim']
        return AirSimRPC(connection['host'],connection['port'],timeout)

    def launch(self,name,args,folder,visible=False):
        with self.lock:
            self.check_abort()
            old=self.processes.get(name)
            if old and old.process.poll() is None:raise ValueError(name+' is already running')
            if old:old.stop()
            p=self.factory(name,args,self.root,folder/(name+'.log'),visible)
            self.processes[name]=p
        self.log('Started '+name+' PID '+str(p.process.pid));return p

    def check_abort(self):
        if self.closed.is_set() or self.abort.is_set() or self.force_event.is_set():raise RuntimeError('Operation cancelled')

    def fingerprint(self,settings):
        return hashlib.sha256(json.dumps(settings,sort_keys=True).encode()).hexdigest()

    def start_ue(self):
        if any(p.state()['running'] for p in self.processes.values()):raise ValueError('Stop owned processes before starting UE')
        d,base,scenario,settings,compiled=artifacts(self.config,self.root)
        editor=Path(d['ue']['editor']);project=Path(d['ue']['project'])
        if not editor.is_file() or editor.suffix.lower()!='.exe':raise ValueError('Choose a valid UE Editor .exe')
        if not project.is_file() or project.suffix.lower()!='.uproject':raise ValueError('Choose a valid .uproject')
        probe=self.rpc(.4)
        try:
            try:connected=bool(probe.call('ping'))
            except Exception:connected=False
            if connected:raise ValueError('AirSim is already running externally; stop it or use Start ROS to attach')
        finally:probe.close()
        folder=self.root/'data/console_runs'/('ue_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
        path=folder/'airsim.json';atomic(path,settings)
        args=[str(editor),str(project),'-game','-windowed','-nosound',f'-settings={path}',f'-abslog={folder / "unreal.log"}']
        if d['ue']['visible']:args += [f'-ResX={d["ue"]["width"]}',f'-ResY={d["ue"]["height"]}']
        else:args+=['-RenderOffscreen','-ResX=640','-ResY=480','-unattended']
        p=self.launch('ue',args,folder,d['ue']['visible'])
        try:
            subprocess.run([sys.executable,str(self.root/'scripts/configure_ue_process.py'),'--pid',str(p.process.pid)],
                capture_output=True,timeout=10,check=True,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
            deadline=time.monotonic()+120
            progress=0.
            while time.monotonic()<deadline:
                self.check_abort()
                if time.monotonic()-progress>=5.:
                    progress=time.monotonic();self.log('UE 进程已启动，正在等待 AirSim RPC 就绪；日志：'+str(folder/'unreal.log'))
                if p.process.poll() is not None:raise RuntimeError('UE exited; inspect '+str(folder/'unreal.log'))
                rpc=self.rpc(.5)
                try:
                    if rpc.call('ping'):
                        actual=json.loads(rpc.call('getSettingsString'))
                        if set(actual['Vehicles'])!=set(settings['Vehicles']):raise ValueError('UE loaded unexpected vehicle settings')
                        self.ue_snapshot=path;self.ue_fingerprint=self.fingerprint(settings);self.airsim_ready=True
                        self.log('AirSim ready; ROS is not started');return
                except (OSError,ConnectionError,TimeoutError):pass
                finally:rpc.close()
                self.closed.wait(.3)
            raise TimeoutError('AirSim did not become ready within 120 seconds')
        except BaseException:
            p.stop();self.airsim_ready=False;raise

    def start_ros(self):
        if any(p.state()['running'] for k,p in self.processes.items() if k!='ue'):raise ValueError('ROS stack is already running')
        d,c,scenario,settings,compiled=artifacts(self.config,self.root)
        probe=self.rpc(2.)
        try:
            if not probe.call('ping'):raise ValueError('Start UE first')
            actual=json.loads(probe.call('getSettingsString'))
            if self.ue_fingerprint and self.ue_fingerprint!=self.fingerprint(settings):raise ValueError('UE settings changed; restart UE before ROS')
            missing=set(settings['Vehicles'])-set(actual['Vehicles'])
            if missing:raise ValueError('UE is missing configured vehicles: '+', '.join(sorted(missing)))
            for agent in d['actors']['agents']:
                v=actual['Vehicles'][agent['name']]
                if any(abs(float(v.get(k,0.))-x)>.01 for k,x in zip(('X','Y','Z'),agent['spawn_ned_m'])):
                    raise ValueError('UE spawn does not match '+agent['name']+'; restart UE with console settings')
        finally:probe.close()
        # Endpoint leases detect external stacks before changing the origin.
        from evtol_simulation.runtime import SensorLease
        lease=SensorLease(dict(backend='airsim',airsim=c['airsim']),'state_adapter');lease.close()
        folder=self.root/'data/console_runs'/('ros_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]);folder.mkdir(parents=True)
        if d['run']['new_session']:
            origin=self.root/c['state']['origin_file']
            if origin.exists():origin.replace(folder/'previous_origin.json')
        c['project_root']=str(self.root);c['actors_config']=str(folder/'actors.yaml')
        (folder/'actors.yaml').write_text(yaml.safe_dump(scenario,allow_unicode=True,sort_keys=False),encoding='utf8')
        (folder/'targeting.yaml').write_text(yaml.safe_dump(c,allow_unicode=True,sort_keys=False),encoding='utf8')
        atomic(folder/'console.json',d)
        sim=self.root/'src/evtol_simulation/config/simulation.perception_2k.yaml'
        specs=node_specs(c,folder/'targeting.yaml',sim,d['run'])
        with self.lock:self.telemetry.pop('diagnostics',None)
        try:
            for name,module,args in specs:
                self.check_abort()
                if name=='gateway' and d['run']['execute']:
                    # Torch/CUDA initialization can stall UE while starting.
                    # Never arm during this cold-start load spike. Require
                    # actual successful inference, not the initial OK/waiting
                    # diagnostic that has not processed any images yet.
                    self.wait_for_vision(c)
                self.launch(name,[sys.executable,'-u','-X','faulthandler','-c',f'from {module} import main; main()']+args,folder)
                if self.closed.wait(.08):self.check_abort()
            deadline=time.monotonic()+8
            while time.monotonic()<deadline:
                self.check_abort()
                with self.lock:dead=[name for name,proc in self.processes.items() if name!='ue' and proc.process.poll() is not None]
                if dead:raise RuntimeError('Startup failed: '+', '.join(dead)+'; inspect run logs')
                self.closed.wait(.2)
            self.active=folder;self.log('ROS processes started; sensor/model readiness is reported separately in the monitor')
        except BaseException:
            self.stop_ros();raise

    def vision_ready(self,namespace):
        with self.lock:
            info=self.telemetry.get('diagnostics',{}).get('/'+namespace.strip('/')+'/vision',{})
            try:return info.get('level')==0 and time.time()-info.get('received',0.)<3. and int(info.get('values',{}).get('samples',0))>0
            except (ValueError,TypeError):return False

    def wait_for_vision(self,c):
        self.log('Waiting for successful visual inference before starting the executing gateway')
        deadline=time.monotonic()+c['model']['startup_timeout_s']+c['model']['timeout_s']+15.
        stable=None
        while time.monotonic()<deadline:
            self.check_abort()
            with self.lock:dead=[n for n,p in self.processes.items() if n!='ue' and p.process.poll() is not None]
            if dead:raise RuntimeError('Startup failed: '+', '.join(dead))
            if self.vision_ready(c['namespace']):
                if stable is None:stable=time.monotonic()
                if time.monotonic()-stable>=2.:return
            else:stable=None
            self.closed.wait(.2)
        raise TimeoutError('Visual inference did not become ready; the executing gateway was not started. Inspect vision.log')

    def stop_ros(self):
        with self.lock:owned=[(n,p) for n,p in self.processes.items() if n!='ue']
        if not owned:return
        # Request the existing control/actors cleanup services while the ROS
        # executor stays alive. Actors may need up to 60 s each to land.
        if self.ros_stop and not self.force_event.is_set():
            for service,timeout in [('/'+self._namespace()+'/control/stop',8.),('/scenario/stop',180.)]:
                if self.force_event.is_set():break
                try:
                    if service.endswith('/scenario/stop') and not any(n=='actors' for n,p in owned):continue
                    self.log(f'请求清理服务 {service}，最长等待 {timeout:g}s；可随时强制结束')
                    self.log(self.ros_stop(service,timeout,cancel=self.force_event))
                except Exception as exc:self.log('Cleanup service: '+str(exc))
        self.log('开始并行结束 ROS 进程树')
        self.terminate(owned)
        self.log('Owned ROS processes stopped. UE remains running.')

    def _namespace(self):
        from .configuration import read_yaml
        return read_yaml(self.root/'config/targeting.yaml')['namespace'].strip('/')

    def stop_ue(self):
        if any(p.state()['running'] for k,p in self.processes.items() if k!='ue'):raise ValueError('Stop ROS before closing UE')
        with self.lock:p=self.processes.get('ue')
        if p:
            self.log('结束自有 UE 进程树 PID '+str(p.process.pid))
            p.stop()
            with self.lock:
                if self.processes.get('ue') is p:self.processes.pop('ue',None)
            self.airsim_ready=False;self.ue_fingerprint=None
        else:self.log('No owned UE process. External UE is not terminated.')

    def stop_all(self):self.stop_ros();self.stop_ue()

    def close(self):
        self.log('管理程序退出：中断当前操作并清理自有进程',level='WARN')
        self.abort.set();self.closed.set();self.force_event.set()
        try:
            with self.lock:owned=list(self.processes.items())
            self.terminate(owned)
            self.worker.join(timeout=12.)
            if self.force_thread:self.force_thread.join(timeout=12.)
        finally:
            self.observer_stop.set();self.observer.join(timeout=2.)
