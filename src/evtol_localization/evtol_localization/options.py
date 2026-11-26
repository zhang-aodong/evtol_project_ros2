"""Validate branch localization settings and migrate earlier console saves."""
import copy
import math
from dataclasses import asdict

from .absolute import Settings


def filter_defaults():
    d = asdict(Settings(min_rsrp_dbm=-32.))
    for name in ('gnss_lever_flu_m', 'radio_lever_flu_m'):
        d[name] = d[name].tolist()
    d['gnss_outage_s'] = list(d['gnss_outage_s'])
    return d


DEFAULTS = dict(enabled=True, mode='gnss_imu', state_source='truth', evaluate=True,
    estimator='absolute', fix_profile='standalone', fiveg_profile='moderate', fiveg_added_sigma_m=1.2,
    absolute=filter_defaults(), vio=dict(keyframe_interval_s=.25, max_image_width=640),
    # Retained as a conservative downstream scan/braking budget, not a filter wait.
    reorder_s=.24, stale_s=.30,
    gnss_fault=dict(enabled=False, time_base='flight', default_mode='normal',
        segments=[dict(start_s=40., end_s=60., mode='dropout', bias_enu_m=[0.,0.,0.])]))

# Migration only: these fields no longer select or run the removed filter.
OLD_FILTER_FIELDS = {
    'max_imu_gap_s': 'max_imu_gap_s', 'accel_noise': 'accel_noise_density',
    'gyro_noise': 'gyro_noise_density', 'accel_bias_noise': 'accel_bias_walk',
    'gyro_bias_noise': 'gyro_bias_walk', 'gnss_sigma_floor_m': 'gnss_sigma_floor_m',
    'gnss_nis_gate': 'gnss_nis_max', 'fiveg_sigma_floor_m': 'range_sigma_floor_m',
    'fiveg_min_rsrp_dbm': 'min_rsrp_dbm', 'fiveg_nis_gate': 'range_nis_max',
    'gnss_lever_flu_m': 'gnss_lever_flu_m', 'fiveg_lever_flu_m': 'radio_lever_flu_m',
}
OBSOLETE_FIELDS = {'max_queue', 'gravity_mps2', 'initial_velocity_std_mps',
    'initial_attitude_std_deg', 'initial_accel_bias_std', 'initial_gyro_bias_std',
    'stationary_window_s', 'stationary_accel_std', 'stationary_gyro_max', 'fiveg_cauchy_scale'}


def options(value=None):
    value = copy.deepcopy(value or {})
    if not isinstance(value, dict):
        raise ValueError('localization must be an object')
    unknown = set(value) - set(DEFAULTS) - set(OLD_FILTER_FIELDS) - OBSOLETE_FIELDS
    if unknown:
        raise ValueError('Unknown localization parameters: ' + str(unknown))
    absolute = value.get('absolute', {})
    if not isinstance(absolute, dict):
        raise ValueError('Invalid localization.absolute')
    for old, new in OLD_FILTER_FIELDS.items():
        if old in value:
            absolute.setdefault(new, value.pop(old))
    for old in OBSOLETE_FIELDS:
        value.pop(old, None)
    value['absolute'] = absolute
    d = copy.deepcopy(DEFAULTS)
    for k, v in value.items():
        if k in ('gnss_fault', 'absolute', 'vio'):
            if not isinstance(v, dict):
                raise ValueError('Invalid localization.' + k)
            d[k].update(v)
        else:
            d[k] = v
    for k in ('enabled', 'evaluate'):
        if type(d[k]) is not bool:
            raise ValueError('Invalid localization.' + k)
    if d['mode'] not in ('gnss_imu', 'gnss_imu_5g', 'camera_imu'):
        raise ValueError('Invalid localization.mode')
    # Accept the previously saved selector only to migrate it. No legacy backend remains.
    if d['estimator'] not in ('legacy', 'absolute', 'vio'):
        raise ValueError('Invalid localization.estimator')
    d['estimator'] = 'vio' if d['mode'] == 'camera_imu' else 'absolute'
    if d['state_source'] not in ('truth', 'fused'):
        raise ValueError('Invalid localization.state_source')
    if d['state_source'] == 'fused' and (not d['enabled'] or d['mode'] == 'camera_imu'):
        raise ValueError('Fused control requires enabled absolute localization in map_enu')
    if d['fix_profile'] not in ('native', 'standalone') or d['fiveg_profile'] not in ('native', 'moderate'):
        raise ValueError('Invalid localization observation profile')
    for name in ('fiveg_added_sigma_m', 'reorder_s', 'stale_s'):
        v = d[name]
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
            raise ValueError('Invalid localization.' + name)
    if not 0 <= d['fiveg_added_sigma_m'] <= 10 or not .1 <= d['reorder_s'] < d['stale_s'] <= .5:
        raise ValueError('Invalid localization noise or freshness budget')
    try:
        Settings(**d['absolute'])
    except (TypeError, ValueError) as exc:
        raise ValueError('Invalid localization.absolute: ' + str(exc)) from exc
    vio = d['vio']
    if set(vio) != {'keyframe_interval_s', 'max_image_width'}:
        raise ValueError('Invalid localization.vio')
    interval, width = vio['keyframe_interval_s'], vio['max_image_width']
    if (isinstance(interval, bool) or not isinstance(interval, (int, float))
            or not math.isfinite(interval) or not .05 <= interval <= 1.
            or type(width) is not int or not 96 <= width <= 2048):
        raise ValueError('Invalid localization.vio timing or resolution')

    f=d['gnss_fault']
    if set(f)!=set(DEFAULTS['gnss_fault']) or type(f['enabled']) is not bool:raise ValueError('Invalid GNSS fault parameters')
    if f['time_base'] not in ('flight','session') or f['default_mode'] not in ('normal','dropout'):raise ValueError('Invalid GNSS fault time base/default')
    if not isinstance(f['segments'],list) or len(f['segments'])>100:raise ValueError('Invalid GNSS fault segments')
    previous=-1.
    for s in f['segments']:
        if set(s)!= {'start_s','end_s','mode','bias_enu_m'}:raise ValueError('Invalid GNSS fault segment fields')
        if any(isinstance(s[k],bool) or not isinstance(s[k],(int,float)) or not math.isfinite(s[k]) for k in ('start_s','end_s')) or not 0<=s['start_s']<s['end_s'] or s['start_s']<previous:raise ValueError('Fault segments must be ordered, finite and non-overlapping')
        previous=s['end_s']
        if s['mode'] not in ('normal','dropout','bias','no_fix'):raise ValueError('Invalid GNSS fault mode')
        b=s['bias_enu_m']
        if not isinstance(b,list) or len(b)!=3 or not all(isinstance(x,(int,float)) and not isinstance(x,bool) and math.isfinite(x) and abs(x)<=10000 for x in b):raise ValueError('Invalid GNSS fault bias')
    return d


def fault_at(config, elapsed):
    if not config['enabled'] or elapsed is None:return 'normal',[0.,0.,0.]
    for s in config['segments']:
        if s['start_s']<=elapsed<s['end_s']:return s['mode'],s['bias_enu_m']
    return config['default_mode'],[0.,0.,0.]
