"""Portable, versioned collector options; independent from simulation startup."""
import copy
import math
import re

CLASSES={0:'background',1:'aircraft',2:'high_rise',3:'pedestrian',4:'ground',5:'runway',6:'drone'}

def defaults(namespace='drone1'):
    root='/'+namespace.strip('/')
    return dict(output_dir='data/collections',bag=dict(storage_id='mcap',use_sim_time=True,
        topics=['/simulation/geodetic_reference',root+'/imu/data',root+'/gnss/fix',root+'/five_ga/observations',
                root+'/ground_truth/odometry',root+'/ground_truth/local_odometry',root+'/localization/odometry',
                root+'/localization/status',root+'/localization/metrics',root+'/localization/path',root+'/localization/truth_path',
                '/clock','/tf','/tf_static','/simulation/reference_origin',root+'/camera/rgb/image_raw',
                root+'/camera/semantic/class_ids',root+'/camera/rgb/camera_info',root+'/state/odometry',root+'/lidar/points'],
        queue_mb=256,split_size_mb=2048),dataset=dict(namespace=namespace.strip('/'),rate_hz=5.,
        class_ids=[6],labels=['detection','segmentation'],split='train',min_area_px=20,
        polygon_epsilon_px=1.,jpeg_quality=95,queue_frames=8,pair_capacity=12))

def validate(value):
    d=copy.deepcopy(value);expected=defaults()
    if not isinstance(d,dict) or set(d)!=set(expected):raise ValueError('Invalid collection sections')
    path=d['output_dir']
    if not isinstance(path,str) or not path.strip() or len(path)>2000 or any(c in path for c in '\0\r\n'):
        raise ValueError('Invalid collection output directory')
    for kind in ('bag','dataset'):
        if not isinstance(d[kind],dict) or set(d[kind])!=set(expected[kind]):raise ValueError('Invalid collection.'+kind+' fields')
    bag=d['bag'];ds=d['dataset']
    if bag['storage_id'] not in ('mcap','sqlite3') or type(bag['use_sim_time']) is not bool:raise ValueError('Invalid bag storage/clock')
    topics=bag['topics']
    if not isinstance(topics,list) or not 1<=len(topics)<=256 or len(set(topics))!=len(topics):raise ValueError('Select 1..256 unique topics')
    for topic in topics:
        if not isinstance(topic,str) or len(topic)>256 or not re.fullmatch(r'/(?:[A-Za-z_][A-Za-z0-9_]*)(?:/[A-Za-z_][A-Za-z0-9_]*)*',topic):
            raise ValueError('Invalid absolute ROS topic: '+str(topic))
    if not isinstance(ds['namespace'],str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*',ds['namespace']):raise ValueError('Invalid dataset namespace')
    if ds['split'] not in ('train','val','test'):raise ValueError('Choose train, val or test dataset split')
    if not isinstance(ds['labels'],list) or not ds['labels'] or len(set(ds['labels']))!=len(ds['labels']) or any(x not in ('detection','segmentation') for x in ds['labels']):
        raise ValueError('Select detection and/or segmentation labels')
    ids=ds['class_ids']
    if not isinstance(ids,list) or not ids or any(type(i) is not int or i not in CLASSES or i==0 for i in ids) or len(set(ids))!=len(ids):
        raise ValueError('Training classes must be unique original IDs from 1..6')
    ranges=[(bag,'queue_mb',16,1024,True),(bag,'split_size_mb',0,65536,True),
        (ds,'rate_hz',.1,30,False),(ds,'min_area_px',1,100000,True),
        (ds,'polygon_epsilon_px',0,10,False),(ds,'jpeg_quality',50,100,True),
        (ds,'queue_frames',1,32,True),(ds,'pair_capacity',2,32,True)]
    for section,key,lo,hi,integer in ranges:
        v=section[key]
        if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or not lo<=v<=hi or (integer and type(v) is not int):
            raise ValueError('Invalid collector '+key)
    return d
