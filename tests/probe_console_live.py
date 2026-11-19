"""Explicit transport/lifecycle integration probe. Never arm the ego or actors."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from evtol_console.engine import Engine
from evtol_console.server import ConsoleBridge,http_server,ros_stop


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--live',action='store_true')
    parser.add_argument('--background',action='store_true');opts=parser.parse_args()
    root=Path(__file__).resolve().parents[1];rclpy.init(domain_id=223)
    os.environ['ROS_DOMAIN_ID']='223'
    engine=Engine(root,ros_stop=ros_stop)
    engine.config['run'].update(execute=False,actors=False,evaluate=False,new_session=True)
    engine.config['actors']['pedestrians_enabled']=False
    engine.config['actors']['count']=1;engine.config['actors']['agents']=engine.config['actors']['agents'][:1]
    engine.config['ue']['visible']=not opts.background
    engine.config['mission']['mode']='profile'
    engine.config['mission']['max_speed_mps']=2.
    # Existing user session data is restored after the isolated probe.
    origin=root/'data/session_origin.json';saved_origin=origin.read_bytes() if origin.exists() else None
    bridge=None;http=None;executor=None;spin=None;finished=threading.Event()
    try:
        bridge=ConsoleBridge(engine,'drone1',0);http=http_server(engine,0)
        threading.Thread(target=http.serve_forever,daemon=True).start()
        executor=SingleThreadedExecutor();executor.add_node(bridge)
        def run():
            while not finished.is_set():executor.spin_once(timeout_sec=.05)
        spin=threading.Thread(target=run,daemon=True);spin.start()
        command=['node',str(root/'foxglove/evtol-console/tests/transport.mjs'),str(http.server_port),str(bridge.server.port)]
        if opts.live:command.append('live')
        result=subprocess.run(command,capture_output=True,text=True,encoding='utf8',timeout=240,
            creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        print(result.stdout,flush=True)
        if result.returncode:raise RuntimeError(result.stderr)
        if opts.live:
            assert not any(p.state()['running'] for p in engine.processes.values())
        report=dict(result='PASS',real_websocket_services=True,live_ue_ros=opts.live,
            ue_mode='background' if opts.background else 'visible',execute=False,actors=False,
            ui_visual_capture='Unavailable: native computer-use pipe missing',output=result.stdout)
        path=root/'reports'/('console_live_'+('background' if opts.background else 'visible')+'.json')
        path.write_text(json.dumps(report,indent=2),encoding='utf8')
    finally:
        engine.close();finished.set()
        if spin:spin.join(timeout=3)
        if executor:executor.shutdown()
        if http:http.shutdown();http.server_close()
        if bridge:bridge.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        if saved_origin is not None:origin.write_bytes(saved_origin)
        elif origin.exists():origin.unlink()


if __name__=='__main__':main()
