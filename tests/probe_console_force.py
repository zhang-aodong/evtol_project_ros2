"""Real ROS actor-status + Foxglove force-stop probe, using harmless owned Python trees.

Does not launch UE, arm vehicles, or stop any pre-existing process.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import yaml
import rclpy
from rclpy.executors import SingleThreadedExecutor
from evtol_console.engine import Engine
from evtol_console.processes import OwnedProcess
from evtol_console.server import ConsoleBridge,http_server
from evtol_scenario.actors import Actors


def main():
    root=Path(__file__).resolve().parents[1]
    os.environ['ROS_DOMAIN_ID']='224';rclpy.init(domain_id=224)
    def cleanup(service,timeout,cancel=None):
        while not cancel.wait(.1):pass
        raise RuntimeError('Probe cleanup interrupted by force request')
    engine=Engine(root,ros_stop=cleanup);finished=threading.Event()
    bridge=None;http=None;executor=None;actors=None;spin=None;child=None
    started=time.monotonic()
    report_path=root/'reports/console_force_probe.json'
    report_path.write_text(json.dumps(dict(result='RUNNING')),encoding='utf8')
    try:
        bridge=ConsoleBridge(engine,'drone1',0);http=http_server(engine,0)
        threading.Thread(target=http.serve_forever,daemon=True).start()
        c=yaml.safe_load((root/'config/targeting.yaml').read_text(encoding='utf8'));c['_root']=root
        actors=Actors(c)
        executor=SingleThreadedExecutor();executor.add_node(bridge);executor.add_node(actors)
        def run():
            while not finished.is_set():executor.spin_once(timeout_sec=.05)
        spin=threading.Thread(target=run,daemon=True);spin.start()
        actors.actor_state('ProbeDrone','landing',reason='验证状态传输，无实际飞行')
        deadline=time.monotonic()+5
        while time.monotonic()<deadline and 'actors' not in engine.status()['telemetry']:time.sleep(.05)
        assert engine.status()['telemetry']['actors']['actors']['ProbeDrone']['phase']=='landing'
        folder=root/'reports'/('console_force_probe_'+str(time.time_ns()));folder.mkdir(exist_ok=True)
        code="import subprocess,sys,time; p=subprocess.Popen([sys.executable,'-c','import time;time.sleep(120)']); print(p.pid,flush=True);time.sleep(120)"
        engine.launch('actors',[sys.executable,'-u','-c',code],folder)
        engine.launch('ue',[sys.executable,'-u','-c','import time;time.sleep(120)'],folder)
        deadline=time.monotonic()+5
        while time.monotonic()<deadline and not (folder/'actors.log').read_text().strip():time.sleep(.05)
        child=int((folder/'actors.log').read_text().splitlines()[0])
        child_handle=subprocess._winapi.OpenProcess(0x100000,False,child) if os.name=='nt' else None
        result=subprocess.run(['node',str(root/'foxglove/evtol-console/tests/transport.mjs'),str(http.server_port),str(bridge.server.port),'force'],
            capture_output=True,text=True,encoding='utf8',timeout=30,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        print(result.stdout,flush=True)
        if result.returncode:raise RuntimeError(result.stderr)
        if child_handle:
            try:assert subprocess._winapi.WaitForSingleObject(child_handle,3000)==0,'Grandchild still running'
            finally:subprocess._winapi.CloseHandle(child_handle)
        assert not engine.status()['processes']
        report=dict(result='PASS',real_ros_actor_status=True,real_foxglove_force_service=True,
            process_tree_terminated=True,simulation_started=False,output=result.stdout,manager_log=str(engine.log_file))
        report_path.write_text(json.dumps(report,indent=2),encoding='utf8')
    except Exception as exc:
        report_path.write_text(json.dumps(dict(result='FAIL',error=str(exc)),indent=2),encoding='utf8')
        raise
    finally:
        engine.close();finished.set()
        if spin:spin.join(timeout=3)
        if executor:executor.shutdown()
        if actors:actors.close();actors.destroy_node()
        if http:http.shutdown();http.server_close()
        if bridge:bridge.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
