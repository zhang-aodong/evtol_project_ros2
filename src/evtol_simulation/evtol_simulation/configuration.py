from pathlib import Path
import math
import yaml


KINDS = ('camera', 'lidar', 'imu', 'gnss', 'truth', 'radio')


def load(path):
    path = Path(path).resolve()
    with path.open(encoding='utf-8-sig') as stream:
        config = yaml.safe_load(stream)
    with (path.parent / config['source_parameters']).open(encoding='utf-8-sig') as stream:
        config['source'] = yaml.safe_load(stream)
    if config['backend'] not in ('airsim', 'mock'):
        raise ValueError('backend must be airsim or mock')
    if config['camera']['transport'] not in ('shared_memory', 'rpc'):
        raise ValueError('Unknown camera transport')
    if config['lidar']['transport'] not in ('shared_memory', 'rpc'):
        raise ValueError('Unknown lidar transport')
    if config['imu'].get('transport','rpc') not in ('shared_memory','rpc'):
        raise ValueError('Unknown IMU transport')
    for kind in KINDS:
        if float(config['rates'][kind]) <= 0:
            raise ValueError(f'Invalid {kind} rate')
    if config['camera']['width'] <= 0 or config['camera']['height'] <= 0:
        raise ValueError('Invalid image size')
    for section, field, default in [('imu','buffer_latency_s',.05),
                                     ('camera','reorder_window_s',.2)]:
        value = float(config[section].get(field, default))
        if not math.isfinite(value) or not 0. <= value <= .5:
            raise ValueError(f'{section}.{field} must be between 0 and 0.5 seconds')
    return config
