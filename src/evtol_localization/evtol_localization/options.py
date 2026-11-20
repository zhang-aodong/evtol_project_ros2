"""Portable options shared by direct launch, console and replay."""
import copy
import math

DEFAULTS = dict(enabled=True, mode='gnss_imu', state_source='truth', evaluate=True,
    reorder_s=.24, max_queue=3000, max_imu_gap_s=.12, stale_s=.30,
    gravity_mps2=9.80665, accel_noise=.08, gyro_noise=.004,
    accel_bias_noise=.005, gyro_bias_noise=.0005,
    initial_velocity_std_mps=2., initial_attitude_std_deg=2.,
    initial_accel_bias_std=.1, initial_gyro_bias_std=.01,
    stationary_window_s=.5, stationary_accel_std=.08, stationary_gyro_max=.03,
    gnss_sigma_floor_m=.3, gnss_nis_gate=16.27,
    fiveg_sigma_floor_m=.15, fiveg_min_rsrp_dbm=-100., fiveg_nis_gate=16.,
    fiveg_cauchy_scale=3., gnss_lever_flu_m=[0.,0.,0.], fiveg_lever_flu_m=[0.,0.,0.],
    gnss_fault=dict(enabled=False, time_base='flight', default_mode='normal',
        segments=[dict(start_s=40., end_s=60., mode='dropout', bias_enu_m=[0.,0.,0.])]))


def options(value=None):
    d=copy.deepcopy(DEFAULTS)
    if value:
        if set(value)-set(d):raise ValueError('Unknown localization parameters: '+str(set(value)-set(d)))
        for k,v in value.items():
            d[k]=({**d[k],**copy.deepcopy(v)} if k=='gnss_fault' and isinstance(v,dict) else copy.deepcopy(v))
    for k,v in d.items():
        if k in ('enabled','evaluate'):
            if type(v) is not bool:raise ValueError('Invalid localization.'+k)
        elif k=='mode':
            if v not in ('gnss_imu','gnss_imu_5g'):raise ValueError('Invalid localization.mode')
        elif k=='state_source':
            if v not in ('truth','fused'):raise ValueError('Invalid localization.state_source')
        elif k.endswith('_flu_m'):
            if not isinstance(v,list) or len(v)!=3 or not all(isinstance(x,(int,float)) and not isinstance(x,bool) and math.isfinite(x) and abs(x)<=10 for x in v):raise ValueError('Invalid antenna lever arm')
        elif k=='gnss_fault':continue
        elif isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or (k!='fiveg_min_rsrp_dbm' and v<=0):raise ValueError('Invalid localization.'+k)
    if not .1<=d['reorder_s']<d['stale_s']<=.5:raise ValueError('Localization reorder must fit the state freshness budget')
    if type(d['max_queue']) is not int or not 100<=d['max_queue']<=10000:raise ValueError('Invalid localization.max_queue')
    if d['state_source']=='fused' and not d['enabled']:raise ValueError('Fused control requires enabled localization')
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
