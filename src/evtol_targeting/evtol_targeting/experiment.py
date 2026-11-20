"""Deterministic range/encounter audit. Truth is evaluation-only."""
from collections import defaultdict,deque
from bisect import bisect_left
import numpy as np
from scipy.optimize import linear_sum_assignment

DEFAULTS=dict(radius_m=10.,region='front_hemisphere',association_gate_m=3.,evidence_s=.3,window_s=10.,
              timely_s=.5,exit_margin_m=2.,exit_hold_s=1.,buffer_s=.4,
              truth_max_gap_s=.15,truth_hz=20.,publish_hz=5.,class_ids=[6],
              target_size_m=[1.,1.,.5],ego_radius_m=.5,risk_clearance_m=2.,risk_horizon_s=3.,score_only_flying=True,warning_events=[])


def gated_assignment(detections,truth,gate):
    """Maximum legal cardinality first, minimum legal distance second."""
    if not detections or not truth:return {}
    a=np.asarray([d['position'] for d in detections]);b=np.asarray([d['position'] for d in truth])
    distances=np.linalg.norm(a[:,None,:]-b[None,:,:],axis=2)
    legal=distances<=gate
    for i,d in enumerate(detections):
        for j,t in enumerate(truth):legal[i,j]&=d.get('class_id',6)==t.get('class_id',6)
    penalty=(min(len(a),len(b))+1)*(gate+1.)
    costs=np.full((len(a),len(b)+len(a)),penalty)
    costs[:,:len(b)]=np.where(legal,distances,penalty*2)
    rows,cols=linear_sum_assignment(costs)
    return {int(i):(int(j),float(distances[i,j])) for i,j in zip(rows,cols) if j<len(b) and legal[i,j]}


class RangeAudit:
    def __init__(self,expected_ids,options=None):
        self.o={**DEFAULTS,**(options or {})};self.expected=list(expected_ids)
        self.truth=defaultdict(lambda:deque(maxlen=400));self.ego=deque(maxlen=400)
        self.observations=deque(maxlen=500);self.window=deque();self.last_t=None
        self.active={};self.events=[];self.totals=dict(tp=0,truth=0,fp=0,valid=0,invalid=0)
        self.completed=dict(success=0,late=0,missed=0,censored=0)
        self.camera=None;self.session='';self.min_distance=None;self.forward=deque(maxlen=400)

    def add_truth(self,identity,t,position,velocity=(0.,0.,0.)):
        self._add(self.truth[identity],t,position,velocity)

    def add_ego(self,t,position,velocity=(0.,0.,0.),forward=(1.,0.,0.)):
        self._add(self.ego,t,position,velocity)
        self._add(self.forward,t,forward,(0.,0.,0.))

    def contains(self,relative,forward,radius=None):
        r=self.o['radius_m'] if radius is None else radius
        return bool(np.linalg.norm(relative)<=r and (self.o['region']=='sphere' or relative@forward>=-1e-9))

    @staticmethod
    def _add(history,t,p,v):
        p=np.asarray(p,float);v=np.asarray(v,float)
        if p.shape!=(3,) or v.shape!=(3,) or not np.isfinite(p).all() or not np.isfinite(v).all():return
        if history and t<=history[-1][0]:return
        history.append((float(t),p,v))

    def at(self,history,t):
        samples=list(history);i=bisect_left(samples,t,key=lambda sample:sample[0])
        for index in (i,i-1):
            if 0<=index<len(samples) and abs(samples[index][0]-t)<1e-7:
                return samples[index][1],samples[index][2]
        if 0<i<len(samples):
            a,b=samples[i-1],samples[i]
            if b[0]-a[0]<=self.o['truth_max_gap_s']:
                f=(t-a[0])/(b[0]-a[0]);return a[1]*(1-f)+b[1]*f,a[2]*(1-f)+b[2]*f
        return None

    def measure(self,t,available,targets):
        truth=[]
        for identity in self.expected:
            state=self.at(self.truth[identity],t)
            if state is None:return None  # wait for truth; never declare absent
            truth.append(dict(id=identity,position=state[0],class_id=6))
        targets=[d for d in targets if d.get('class_id',6) in self.o['class_ids'] and np.isfinite(d['position']).all()]
        pairs=gated_assignment(targets,truth,self.o['association_gate_m'])
        matches={truth[j]['id']:dict(track_id=targets[i]['track_id'],distance_m=error) for i,(j,error) in pairs.items()}
        fp=[dict(position=np.asarray(d['position']).tolist(),track_id=d['track_id']) for i,d in enumerate(targets) if i not in pairs]
        row=dict(stamp_s=t,available_s=available,matches=matches,false_positives=fp,
                 frame_truth_count=len(truth),frame_tp=len(matches),late=available-t>self.o['buffer_s'])
        self.observations.append(row)
        return row

    def _event(self,identity,state,t,**extra):
        item=dict(identity=identity,state=state,stamp_s=t,**extra);self.events.append(item)
        self.events=self.events[-500:];return item

    def snapshot(self,t,visible=None,score=True):
        if self.last_t is not None and t<=self.last_t:return None
        self.last_t=t;ego=self.at(self.ego,t);states={i:self.at(self.truth[i],t) for i in self.expected}
        missing=[i for i,s in states.items() if s is None]
        valid=ego is not None and not missing
        if not valid:
            if score:self.totals['invalid']+=1
            return dict(schema_version=1,session_id=self.session,stamp_s=t,valid=False,reason='truth_incomplete',
                expected=len(self.expected),known=len(self.expected)-len(missing),missing=missing,ego_valid=ego is not None,
                radius_m=self.o['radius_m'],totals=dict(self.totals),recall=None,targets=[],scoring=score)
        position,velocity=ego
        direction=self.at(self.forward,t)
        if direction is None or np.linalg.norm(direction[0])<1e-9:
            if score:self.totals['invalid']+=1
            return dict(schema_version=1,session_id=self.session,stamp_s=t,valid=False,reason='orientation_incomplete',
                totals=dict(self.totals),targets=[],scoring=score)
        forward=direction[0]/max(np.linalg.norm(direction[0]),1e-12)
        inside={i:s for i,s in states.items() if self.contains(s[0]-position,forward)}
        observations=[r for r in self.observations if 0<=t-r['stamp_s']<=self.o['evidence_s'] and not r['late']]
        # A track that changes association cannot cover both its old and new
        # identities during the evidence hold window.
        latest_tracks={}
        for row in sorted(observations,key=lambda r:r['stamp_s']):
            for identity,match in row['matches'].items():latest_tracks[match['track_id']]=(identity,row,match)
            for item in row['false_positives']:latest_tracks[item['track_id']]=(None,row,item)
        evidence={}
        for identity,row,match in latest_tracks.values():
            if identity is not None and (identity not in evidence or row['stamp_s']>evidence[identity][0]['stamp_s']):evidence[identity]=(row,match)
        matched=set(inside)&set(evidence);fp=0
        # A held false detection counts once per track, not once per publish.
        false_tracks={key:item for key,(identity,row,item) in latest_tracks.items() if identity is None}
        for d in false_tracks.values():fp+=self.contains(np.asarray(d['position'])-position,forward)
        targets=[]
        for identity,state in states.items():
            distance=float(np.linalg.norm(state[0]-position));seen=identity in matched
            if score and identity in inside and identity not in self.active:
                self.active[identity]=dict(entered=t,outside_since=None,result='pending')
                self._event(identity,'entered',t)
            event=self.active.get(identity)
            if event and score:
                if identity in matched and event['result']=='missed':
                    row=evidence[identity][0];delay=max(0.,row['available_s']-event['entered'])
                    event['result']='late';self.completed['missed']-=1;self.completed['late']+=1
                    self._event(identity,'late_after_deadline',t,delay_s=delay)
                if identity in matched and event['result']=='pending':
                    row=evidence[identity][0];delay=max(0.,row['available_s']-event['entered'])
                    event['result']='success' if delay<=self.o['timely_s'] else 'late'
                    self.completed[event['result']]+=1
                    self._event(identity,event['result'],t,delay_s=delay,entered_s=event['entered'])
                if event['result']=='pending' and t-event['entered']>=self.o['timely_s']:
                    event['result']='missed';self.completed['missed']+=1;self._event(identity,'missed',t)
                exited=distance>self.o['radius_m']+self.o['exit_margin_m'] or (self.o['region']=='front_hemisphere' and (state[0]-position)@forward < -self.o['exit_margin_m'])
                if exited:
                    if event['outside_since'] is None:event['outside_since']=t
                    if t-event['outside_since']>=self.o['exit_hold_s']:
                        if event['result']=='pending':self.completed['missed']+=1
                        self._event(identity,'exited',t,result=event['result']);del self.active[identity]
                else:event['outside_since']=None
            in_view=bool(visible(identity,state[0])) if visible else None
            targets.append(dict(identity=identity,position_enu_m=state[0].tolist(),distance_m=distance,
                in_range=identity in inside,detected=seen,in_common_fov=in_view,
                track_id=evidence[identity][1]['track_id'] if identity in evidence else None,
                observation_age_s=t-evidence[identity][0]['stamp_s'] if identity in evidence else None,
                event=event['result'] if event else 'outside'))
        tp=len(matched);n=len(inside);fp=int(fp)
        if score:
            self.totals['tp']+=tp;self.totals['truth']+=n;self.totals['fp']+=fp;self.totals['valid']+=1
            self.window.append((t,tp,n,fp))
        while self.window and t-self.window[0][0]>self.o['window_s']:self.window.popleft()
        wtp=sum(r[1] for r in self.window);wn=sum(r[2] for r in self.window)
        visible_ids={r['identity'] for r in targets if r['in_range'] and r['in_common_fov']}
        if states and score:
            nearest=min(float(np.linalg.norm(s[0]-position)) for s in states.values())
            self.min_distance=nearest if self.min_distance is None else min(nearest,self.min_distance)
        done=sum(self.completed[k] for k in ('success','late','missed'))
        return dict(schema_version=1,session_id=self.session,stamp_s=t,valid=True,reason='ok',radius_m=self.o['radius_m'],region=self.o['region'],forward_enu=forward.tolist(),
            expected=len(self.expected),known=len(states),truth_count=n,detected_count=tp,missed_count=n-tp,false_positive_count=fp,scoring=score,
            recall=tp/n if n else None,precision=tp/(tp+fp) if tp+fp else None,
            window_recall=wtp/wn if wn else None,cumulative_recall=self.totals['tp']/self.totals['truth'] if self.totals['truth'] else None,
            fov_truth_count=len(visible_ids),fov_detected_count=len(visible_ids&matched),
            fov_recall=len(visible_ids&matched)/len(visible_ids) if visible_ids else None,
            timely_success_rate=self.completed['success']/done if done else None,
            encounters=dict(self.completed,pending=sum(e['result']=='pending' for e in self.active.values())),
            totals=dict(self.totals),minimum_center_distance_m=self.min_distance,
            minimum_conservative_sphere_gap_m=self.min_distance-self.o['ego_radius_m']-.5*float(np.linalg.norm(self.o['target_size_m'])) if self.min_distance is not None else None,
            targets=targets,
            matching_basis='geometric_surface_to_vehicle_origin',evidence_s=self.o['evidence_s'])
