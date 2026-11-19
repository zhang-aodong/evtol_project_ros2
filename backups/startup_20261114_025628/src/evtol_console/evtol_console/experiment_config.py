"""Versioned configuration migration; import never starts or modifies a live run."""
import copy
import math
from evtol_targeting.experiment import DEFAULTS as EXPERIMENT
from evtol_safety.dynamic import DEFAULTS as DYNAMIC


def upgrade(value):
    d=copy.deepcopy(value)
    if d.get('version') not in (1,2):raise ValueError('Unsupported console configuration version')
    if d['version']==1:
        d.update(version=2,sensors=dict(horizontal_fov_deg=70.,imgsz=960),
                 experiment=copy.deepcopy(EXPERIMENT),dynamic=copy.deepcopy(DYNAMIC),
                 ui=dict(log_filter='',log_errors=False,log_follow=True,profile_default_distance_m=100.))
    return d


def validate_options(d):
    required={'version','ue','actors','mission','run','sensors','experiment','dynamic','ui'}
    if set(d)!=required:raise ValueError('Missing or unknown configuration sections: '+str(set(d)^required))
    shapes=dict(ue={'editor','project','settings','visible','width','height'},actors={'count','agents','pedestrians_enabled'},
        mission={'mode','speed_mps','waypoints_enu_m','land_at_last','profile','max_speed_mps','max_acceleration_mps2'},
        run={'execute','actors','evaluate','new_session'},sensors={'horizontal_fov_deg','imgsz'},
        experiment=set(EXPERIMENT),dynamic=set(DYNAMIC),ui={'log_filter','log_errors','log_follow','profile_default_distance_m'})
    for section,fields in shapes.items():
        if not isinstance(d[section],dict) or set(d[section])!=fields:raise ValueError('Unknown or missing fields in '+section)
    for section,values in [('experiment',EXPERIMENT),('dynamic',DYNAMIC)]:
        for name,default in values.items():
            value=d[section][name]
            if section=='experiment' and name=='warning_events':
                from evtol_targeting.warning_audit import validate_events
                validate_events(value);continue
            if isinstance(default,bool):
                if not isinstance(value,bool):raise ValueError('Invalid '+section+'.'+name)
            elif isinstance(default,list):
                if not isinstance(value,list) or len(value)!=len(default) or any(isinstance(v,bool) or not isinstance(v,(float,int)) or not math.isfinite(v) or v<0 for v in value):
                    raise ValueError('Invalid '+section+'.'+name)
            elif isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
                raise ValueError('Invalid '+section+'.'+name)
    o=d['experiment'];dy=d['dynamic']
    if o['radius_m']>100 or o['evidence_s']>1 or o['buffer_s']>2 or o['publish_hz']>20 or o['truth_hz']>50:
        raise ValueError('Experiment range/rate/latency exceeds supported budget')
    if not 1<=o['association_gate_m']<=5:raise ValueError('Association gate must be 1..5 m')
    if o['class_ids']!=[6]:raise ValueError('Aircraft evaluation currently requires class_ids [6]')
    if not .05<=dy['step_s']<=.25 or not dy['step_s']<=dy['horizon_s']<=5:
        raise ValueError('Dynamic prediction horizon/step exceeds budget')
    if type(dy['max_targets']) is not int or not 1<=dy['max_targets']<=128:raise ValueError('max_targets must be 1..128')
    if any(x<=0 for x in dy['min_size_m']+o['target_size_m']):raise ValueError('Aircraft sizes must be positive')
    sensors=d['sensors']
    if isinstance(sensors['horizontal_fov_deg'],bool) or not isinstance(sensors['horizontal_fov_deg'],(int,float)) or not 20<=sensors['horizontal_fov_deg']<=120:
        raise ValueError('Camera FOV must be 20..120 degrees')
    if type(sensors['imgsz']) is not int or not 320<=sensors['imgsz']<=1536 or sensors['imgsz']%32:
        raise ValueError('Inference size must be a multiple of 32 in 320..1536')
    ui=d['ui']
    if not isinstance(ui['log_filter'],str) or len(ui['log_filter'])>200 or any(type(ui[k]) is not bool for k in ('log_errors','log_follow')):
        raise ValueError('Invalid UI options')
    if not isinstance(ui['profile_default_distance_m'],(int,float)) or not 0<ui['profile_default_distance_m']<=10000:
        raise ValueError('Invalid profile default distance')
    for a in d['actors']['agents']:
        if set(a)!={'name','speed_mps','spawn_ned_m','waypoints_enu_m','repeat'}:raise ValueError('Unknown or missing actor fields')
    if any(e['truth_id']>d['actors']['count'] for e in o['warning_events']):raise ValueError('Warning event references absent truth target')
    if set(d['mission']['profile'])!={'heading_deg','points'}:raise ValueError('Unknown or missing profile fields')
    for p in d['mission']['profile']['points']:
        if set(p)!={'speed_mps','height_m','distance_m'}:raise ValueError('Unknown profile point fields')
