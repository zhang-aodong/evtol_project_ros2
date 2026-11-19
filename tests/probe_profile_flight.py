"""Explicit live flight probe: owns its UE and restores the prior ENU origin.

Run manually, not through unittest discovery. This arms the simulated ego.
"""
import json
import os
from pathlib import Path
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from evtol_console.configuration import validate
from evtol_console.engine import Engine
from evtol_console.server import ConsoleBridge, ros_stop


def main():
    root = Path(__file__).resolve().parents[1]
    os.environ['ROS_DOMAIN_ID'] = '223'
    rclpy.init(domain_id=223)
    engine = Engine(root, ros_stop=ros_stop)
    engine.config['run'].update(execute=True, actors=False, evaluate=False, new_session=True)
    engine.config['actors'].update(count=0, agents=[], pedestrians_enabled=False)
    engine.config['ue']['visible'] = False
    engine.config['mission'].update(mode='profile', max_speed_mps=2., land_at_last=True)
    engine.config['mission']['profile'] = dict(heading_deg=0., points=[
        dict(speed_mps=0., height_m=0., distance_m=100.),
        dict(speed_mps=1., height_m=3., distance_m=100.),
        dict(speed_mps=1., height_m=3., distance_m=4.),
        dict(speed_mps=0., height_m=0., distance_m=100.),
    ])
    validate(engine.config, root)
    origin = root/'data/session_origin.json'
    saved_origin = origin.read_bytes() if origin.exists() else None
    bridge = None
    executor = None
    spin = None
    finished = threading.Event()
    samples = []
    try:
        bridge = ConsoleBridge(engine, 'drone1', 0)
        executor = SingleThreadedExecutor()
        executor.add_node(bridge)
        def run():
            while not finished.is_set():
                executor.spin_once(timeout_sec=.05)
        spin = threading.Thread(target=run, daemon=True)
        spin.start()
        engine.start_ue()
        engine.start_ros()
        deadline = time.monotonic()+180.
        last_phase = None
        while time.monotonic() < deadline:
            status = engine.status()
            telemetry = status['telemetry']
            phase = telemetry.get('flight', {}).get('phase')
            if phase != last_phase:
                print('Flight phase:', phase, flush=True)
                last_phase = phase
            if telemetry.get('position_enu_m'):
                samples.append(dict(position=telemetry['position_enu_m'], speed=telemetry.get('speed_mps'), phase=phase))
            if phase == 'failed':
                raise RuntimeError(str(telemetry.get('flight')))
            if phase == 'landed':
                break
            time.sleep(.25)
        else:
            raise TimeoutError('Profile did not land: '+json.dumps(status['telemetry']))
        assert max(s['position'][2] for s in samples) > 2.5, 'Did not reach climb waypoint'
        assert max(s['position'][1] for s in samples) > 3.5, 'Did not reach horizontal waypoint'
        final = samples[-1]['position']
        assert abs(final[2]) < .45 and abs(final[1]-4.) < 1., final
        result = dict(result='PASS', execute=True, actors=False, final_enu_m=final,
            max_height_m=max(s['position'][2] for s in samples), samples=samples,
            note='Small clear-scene profile; no claim of full-speed or obstacle-course validation')
        (root/'reports/console_profile_flight.json').write_text(json.dumps(result, indent=2), encoding='utf8')
        print('PASS: real profile climb, horizontal flight, descent and verified landing', flush=True)
    finally:
        engine.close()
        finished.set()
        if spin: spin.join(timeout=3)
        if executor: executor.shutdown()
        if bridge: bridge.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        if saved_origin is not None: origin.write_bytes(saved_origin)
        elif origin.exists(): origin.unlink()


if __name__ == '__main__':
    main()
