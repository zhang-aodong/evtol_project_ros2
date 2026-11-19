"""Read-only checks against settings actually loaded by the running AirSim server."""
import argparse
import json
from .configuration import load
from .rpc import AirSimRPC


def inspect_settings(c, settings):
    problems = []
    try:
        vehicle = settings['Vehicles'][c['airsim']['vehicle']]
        camera = vehicle['Cameras'][c['camera']['name']]
        captures = {x['ImageType']:x for x in camera['CaptureSettings']}
        for image_type in (0,5):
            actual = captures[image_type]
            for key, expected in [('Width',c['camera']['width']),('Height',c['camera']['height']),
                                  ('FOV_Degrees',c['camera']['horizontal_fov_deg'])]:
                if actual[key]!=expected:
                    problems.append(f'ImageType={image_type} {key}: {actual[key]} != {expected}')
        sensors = vehicle['Sensors']
        for key in ('imu','gnss','lidar'):
            sensor = sensors[c[key]['name']]
            if not sensor.get('Enabled',True):
                problems.append(f'{key} disabled')
            if key=='lidar':
                for field,expected in [('DataFrame','SensorLocalFrame'),('Range',c['lidar']['range_m']),
                    ('UpdateFrequency',c['rates']['lidar']),('RotationsPerSecond',c['rates']['lidar']),
                    ('CompleteScanPerUpdate',True)]:
                    if sensor.get(field)!=expected:
                        problems.append(f'LiDAR {field}: {sensor.get(field)} != {expected}')
            expected_xyz = c[key]['xyz_frd_m']
            for field,expected in zip(('X','Y','Z'),expected_xyz):
                if sensor.get(field,0.)!=expected:
                    problems.append(f'{key} {field} extrinsic mismatch')
            for field,expected in zip(('Roll','Pitch','Yaw'),c[key]['rpy_deg']):
                if sensor.get(field,0.)!=expected:
                    problems.append(f'{key} {field} extrinsic mismatch')
        for field,expected in zip(('X','Y','Z'),c['camera']['xyz_frd_m']):
            if camera.get(field,0.)!=expected:
                problems.append(f'Camera {field} extrinsic mismatch')
        for field,expected in zip(('Roll','Pitch','Yaw'),c['camera']['rpy_deg']):
            if camera.get(field,0.)!=expected:
                problems.append(f'Camera {field} extrinsic mismatch')
        if settings.get('ClockSpeed',1.)!=1.:
            problems.append('ClockSpeed != 1; real-time acceptance requires 1')
    except (KeyError,TypeError) as exc:
        problems.append(f'Missing AirSim settings: {exc}')
    return problems


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--config',required=True)
    parser.add_argument('--settings-file',help='Offline check only; omit to query running server')
    opts = parser.parse_args(args)
    c = load(opts.config)
    try:
        if opts.settings_file:
            from pathlib import Path
            settings = json.loads(Path(opts.settings_file).read_text(encoding='utf-8-sig'))
        else:
            conn = c['airsim']
            rpc = AirSimRPC(conn['host'],conn['port'],conn['timeout_s'])
            try:
                settings = json.loads(rpc.call('getSettingsString'))
            finally:
                rpc.close()
        problems = inspect_settings(c,settings)
        if not opts.settings_file and c['imu'].get('transport','rpc')=='shared_memory':
            from .imu_ring import SharedMemoryImu
            reader=SharedMemoryImu('AirSimImu_'+c['airsim']['vehicle']+'_'+c['imu']['name'],c['rates']['imu'])
            reader.close()
        if not opts.settings_file:
            from .semantic import native_decoder
            if native_decoder() is None:problems.append('Missing evtol_sensor_native library; run scripts/build.ps1')
    except Exception as exc:
        problems = [f'{type(exc).__name__}: {exc}']
    print(json.dumps(dict(passed=not problems,offline=bool(opts.settings_file),problems=problems),indent=2))
    raise SystemExit(1 if problems else 0)
