"""Event confusion matrix against frozen user/scenario labels, never own warnings."""
import math


def validate_events(events):
    if not isinstance(events,list) or len(events)>10000:raise ValueError('warning_events must be a list of at most 10000 events')
    names=set();by_target={}
    for e in events:
        if not isinstance(e,dict) or set(e)!={'id','truth_id','start_s','end_s','deadline_s','risk'}:
            raise ValueError('Warning event fields: id, truth_id, start_s, end_s, deadline_s, risk')
        if not isinstance(e['id'],str) or not e['id'] or e['id'] in names:raise ValueError('Warning event IDs must be unique')
        names.add(e['id'])
        if type(e['truth_id']) is not int or e['truth_id']<1 or type(e['risk']) is not bool:raise ValueError('Invalid warning event target/risk')
        for k in ('start_s','end_s','deadline_s'):
            if isinstance(e[k],bool) or not isinstance(e[k],(int,float)) or not math.isfinite(e[k]):raise ValueError('Invalid warning event timing')
        if not 0<=e['start_s']<=e['deadline_s']<=e['end_s'] or e['end_s']==e['start_s']:
            raise ValueError('Warning event requires 0 <= start <= deadline <= end, with end > start')
        prior=by_target.setdefault(e['truth_id'],[])
        if any(max(a,e['start_s'])<min(b,e['end_s']) for a,b in prior):raise ValueError('Overlapping warning windows for same truth target')
        prior.append((e['start_s'],e['end_s']))


class WarningAudit:
    def __init__(self,events):
        validate_events(events);self.events=[dict(e,first_warning_s=None,closed=False,unscorable=False) for e in events]
        self.unmatched_warnings=0;self.seen_warning_samples=set();self.invalid_epochs=0

    def observe(self,elapsed,track_to_truth,warnings,sample_id,valid=True):
        if not valid:
            self.invalid_epochs+=1
            for e in self.events:
                if e['start_s']<=elapsed<=e['end_s']:e['unscorable']=True
        if valid and sample_id not in self.seen_warning_samples:
            self.seen_warning_samples.add(sample_id)
            if len(self.seen_warning_samples)>10000:self.seen_warning_samples={sample_id}
            for warning in warnings:
                if not warning.get('warning'):continue
                identity=track_to_truth.get(warning['track_id']);accepted=False
                for e in self.events:
                    if e['truth_id']==identity and e['start_s']<=elapsed<=e['end_s']:
                        if e['first_warning_s'] is None:e['first_warning_s']=elapsed
                        accepted=True
                if not accepted:self.unmatched_warnings+=1
        counts=dict(tp=0,fp=0,fn=0,tn=0,pending=0,late=0,unscorable=0)
        for e in self.events:
            first=e['first_warning_s'];finished=elapsed>=e['end_s']
            if not finished:counts['pending']+=1;continue
            if e['unscorable']:counts['unscorable']+=1;continue
            if e['risk']:
                timely=first is not None and first<=e['deadline_s']
                counts['tp' if timely else 'fn']+=1
                counts['late']+=first is not None and not timely
            else:counts['fp' if first is not None else 'tn']+=1
        tp,fp,fn,tn=(counts[k] for k in ('tp','fp','fn','tn'))
        return dict(basis='frozen_configured_event_windows',label_count=len(self.events),counts=counts,
            precision=tp/(tp+fp) if tp+fp else None,recall=tp/(tp+fn) if tp+fn else None,
            miss_rate=fn/(tp+fn) if tp+fn else None,false_alarm_rate=fp/(fp+tn) if fp+tn else None,
            false_discovery_rate=fp/(tp+fp) if tp+fp else None,accuracy=(tp+tn)/(tp+fp+fn+tn) if tp+fp+fn+tn else None,
            unmatched_warning_samples=self.unmatched_warnings,invalid_epochs=self.invalid_epochs,
            events=[dict(e) for e in self.events])
