"""Pure configuration and ENU/global-NED box geometry."""
from dataclasses import dataclass
import itertools
import math
import numpy as np
from evtol_simulation.coordinates import NED_ENU


DEFAULTS=dict(enabled=True,input_topic='perception/tracks',class_ids=[6],rate_hz=10.,
    message_timeout_s=.5,measurement_timeout_s=1.,clock_timeout_s=.5,
    line_lifetime_s=.2,line_thickness=3.,color_rgba=[0.,1.,0.,1.],
    min_size_enu_m=[1.,1.,.5],padding_m=.15,max_boxes=16,
    show_labels=True,label_scale=1.,rpc_timeout_s=.5,
    source_coordinates='vehicle_local',ego_actor_name='',global_ned_offset_m=None)


def options(c):
    result=dict(DEFAULTS);result.update(c.get('ue_detection_boxes',{}))
    for key in ('rate_hz','message_timeout_s','measurement_timeout_s','clock_timeout_s',
                'line_lifetime_s','line_thickness','label_scale','rpc_timeout_s'):
        value=result[key]
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
            raise ValueError('Invalid ue_detection_boxes.'+key)
    if not 1./result['rate_hz']<=result['line_lifetime_s']<=1.:
        raise ValueError('UE box lifetime must cover one refresh period and be <= 1 s')
    if not isinstance(result['max_boxes'],int) or isinstance(result['max_boxes'],bool) or not 1<=result['max_boxes']<=128:
        raise ValueError('ue_detection_boxes.max_boxes must be an integer in 1..128')
    for key,count in (('min_size_enu_m',3),('color_rgba',4)):
        a=np.asarray(result[key],dtype=float)
        if a.shape!=(count,) or not np.isfinite(a).all():raise ValueError('Invalid ue_detection_boxes.'+key)
        if key=='min_size_enu_m' and np.any(a<=0):raise ValueError('UE minimum box dimensions must be positive')
        if key=='color_rgba' and (np.any(a<0) or np.any(a>1) or a[3]<=0):raise ValueError('Invalid UE box color')
    if not isinstance(result['padding_m'],(int,float)) or not math.isfinite(result['padding_m']) or result['padding_m']<0:
        raise ValueError('Invalid ue_detection_boxes.padding_m')
    if result['source_coordinates'] not in ('vehicle_local','global_ned'):
        raise ValueError('Invalid ue_detection_boxes.source_coordinates')
    if not isinstance(result['input_topic'],str) or not result['input_topic'].strip('/'):
        raise ValueError('Invalid UE detection input topic')
    if not isinstance(result['class_ids'],list) or not result['class_ids'] or any(type(i)!=int or i<0 for i in result['class_ids']):
        raise ValueError('Invalid ue_detection_boxes.class_ids')
    for key in ('enabled','show_labels'):
        if not isinstance(result[key],bool):raise ValueError('Invalid ue_detection_boxes.'+key)
    if not isinstance(result['ego_actor_name'],str):raise ValueError('Invalid UE ego actor name')
    offset=result['global_ned_offset_m']
    if offset is not None:
        a=np.asarray(offset,dtype=float)
        if a.shape!=(3,) or not np.isfinite(a).all():raise ValueError('Invalid UE global NED offset')
    return result


def to_global_ned(position_enu,origin_enu,spawn_ned):
    return NED_ENU.T@(np.asarray(position_enu)+np.asarray(origin_enu))+np.asarray(spawn_ned)


def box_edges_ned(center_enu,size_enu,origin_enu,spawn_ned):
    corners=np.asarray(list(itertools.product((-1.,1.),repeat=3)))*np.asarray(size_enu)/2+np.asarray(center_enu)
    corners=np.asarray([to_global_ned(p,origin_enu,spawn_ned) for p in corners])
    edges=[]
    for i in range(8):
        for axis in (1,2,4):
            j=i^axis
            if i<j:edges.extend((corners[i],corners[j]))
    return edges


def wire_point(point):return dict(zip(('x_val','y_val','z_val'),map(float,point)))


@dataclass(frozen=True)
class Box:
    track_id:int
    center:tuple
    size:tuple
    last_measurement_ns:int
    state:str


