"""Bounded timestamp ordering; no wall-clock rewriting, no truth inputs."""
from collections import Counter, deque
import heapq
import numpy as np
from .eskf import ESKF, rotation
from .options import options, fault_at


class FusionStream:
    def __init__(self,config=None):
        self.o=options(config);self.reset('startup')

    def reset(self,reason):
        self.filter=ESKF(self.o);self.queue=[];self.seen={};self.serial=0
        self.processed_ns=0;self.last_imu=None;self.imu_window=deque(maxlen=300)
        self.stats=Counter();self.anchors={};self.received={};self.used={};self.last_rejection=''
        self.status='waiting_initialization';self.reset_reason=reason;self.fault='normal'
        self.last_gnss_mode='normal';self.recoveries=[];self.corrections=[];self.last_observation=0
        self.fix_history=deque(maxlen=10)

    def push(self,kind,t,payload,key=None):
        t=int(t);self.received[kind]=max(t,self.received.get(kind,0));self.stats['received_'+kind]+=1
        identity=(kind,t,key)
        if identity in self.seen:self.stats['duplicate_'+kind]+=1;return False
        if t<=self.processed_ns:self.reject(kind,'late_outside_reorder');return False
        if len(self.queue)>=self.o['max_queue']:self.reject(kind,'queue_full');return False
        self.serial+=1;priority={'imu':0,'gnss':1,'fiveg':2}[kind]
        heapq.heappush(self.queue,(t,priority,self.serial,kind,payload));self.seen[identity]=t
        # Dedupe memory is bounded even during missing /clock.
        if len(self.seen)>self.o['max_queue']*2:
            cutoff=t-int(3e9);self.seen={k:v for k,v in self.seen.items() if v>=cutoff}
            if len(self.seen)>self.o['max_queue']*2:self.seen=dict(list(self.seen.items())[-self.o['max_queue']:])
        return True

    def reject(self,kind,reason,anchor=None):
        self.stats['rejected_'+kind]+=1;self.stats[kind+'_'+reason]+=1;self.last_rejection=kind+':'+reason
        if anchor is not None:
            v=self.anchors.setdefault(str(anchor),dict(accepted=0,rejected=0));v['rejected']+=1;v['reason']=reason
        return dict(accepted=False,reason=reason)

    def stationary(self):
        if len(self.imu_window)<5:return False,None,None
        first,last=self.imu_window[0],self.imu_window[-1]
        if (last[0]-first[0])*1e-9<self.o['stationary_window_s']*.9:return False,None,None
        acc=np.array([x[1]['accel'] for x in self.imu_window]);gyro=np.array([x[1]['gyro'] for x in self.imu_window])
        if last[1].get('q') is None:return False,None,None
        residual=acc.mean(axis=0)+rotation(last[1]['q']).T@self.filter.g
        good=(np.max(acc.std(axis=0))<self.o['stationary_accel_std'] and
              np.linalg.norm(gyro.mean(axis=0))<self.o['stationary_gyro_max'] and np.linalg.norm(residual)<.5)
        return good,residual,gyro.mean(axis=0)

    def drain(self,now_ns,base_ns=None):
        cutoff=int(now_ns-self.o['reorder_s']*1e9)
        while self.queue and self.queue[0][0]<=cutoff:
            t,_,_,kind,data=heapq.heappop(self.queue)
            if kind=='imu':
                a=np.r_[data['accel'],data['gyro']]
                if not np.isfinite(a).all():self.reject(kind,'nonfinite');continue
                if self.last_imu and (t-self.last_imu[0])*1e-9>self.o['max_imu_gap_s']:
                    # Explicitly invalidate; never extrapolate across an unobserved gap.
                    self.filter=ESKF(self.o);self.imu_window.clear();self.fix_history.clear();self.status='waiting_initialization';self.reset_reason='imu_gap'
                    self.stats['imu_gap_reset']+=1
                self.last_imu=(t,data);self.imu_window.append((t,data))
                while self.imu_window and (t-self.imu_window[0][0])*1e-9>self.o['stationary_window_s']+.03:self.imu_window.popleft()
            f=self.filter
            if f.t is not None:
                try:
                    # Previous sample is held until the next IMU epoch. At an IMU
                    # epoch use the current sample (end-point discretization).
                    imu=self.last_imu[1];f.propagate(t,imu['accel'],imu['gyro'])
                except ValueError:
                    self.reject(kind,'integration_gap');continue
            if kind=='gnss':self.gnss(t,data,base_ns)
            elif kind=='fiveg':self.fiveg(t,data)
            else:self.used['imu']=t;self.stats['used_imu']+=1
            self.processed_ns=t
        if self.filter.t is not None:
            if (now_ns-self.filter.t)*1e-9>self.o['stale_s']:self.status='stale'
            elif (self.filter.t-self.last_observation)*1e-9>1.:self.status='inertial_only'
            else:self.status='normal'

    def gnss(self,t,d,base_ns):
        elapsed=None if base_ns is None else (t-base_ns)*1e-9
        mode,bias=fault_at(self.o['gnss_fault'],elapsed);self.fault=mode
        recovering=mode=='normal' and self.last_gnss_mode!='normal';self.last_gnss_mode=mode
        if mode in ('dropout','no_fix'):
            return self.reject('gnss','injected_'+mode)
        if not d.get('valid'):return self.reject('gnss','no_fix')
        pos=np.asarray(d['position'],float)+ (np.asarray(bias) if mode=='bias' else 0.)
        cov=np.asarray(d['covariance'],float)
        if cov.shape!=(3,3) or not np.isfinite(np.r_[pos,cov.ravel()]).all() or not np.allclose(cov,cov.T,atol=1e-8):return self.reject('gnss','invalid_covariance')
        eig,vec=np.linalg.eigh(cov)
        if eig.min()<-1e-8:return self.reject('gnss','negative_covariance')
        cov=vec@np.diag(np.maximum(eig,self.o['gnss_sigma_floor_m']**2))@vec.T
        f=self.filter
        if f.t is None:
            if not self.last_imu:return self.reject('gnss','waiting_imu')
            q=self.last_imu[1].get('q')
            if q is None:return self.reject('gnss','waiting_imu_orientation')
            stationary,ba,bg=self.stationary()
            # Wait for a measured stationary window at cold start. Moving-start
            # initialization uses consecutive fixes for velocity, never truth.
            self.fix_history.append((t,pos.copy()))
            span=(self.imu_window[-1][0]-self.imu_window[0][0])*1e-9 if self.imu_window else 0.
            if not stationary and (len(self.fix_history)<5 or span<self.o['stationary_window_s']*.9):return self.reject('gnss','waiting_initialization_window')
            velocity=None
            if not stationary:
                dt=(t-self.fix_history[0][0])*1e-9
                if dt<=0:return self.reject('gnss','waiting_velocity')
                velocity=(pos-self.fix_history[0][1])/dt
            f.initialize(t,pos-rotation(q)@np.asarray(self.o['gnss_lever_flu_m']),q,cov,
                         ba=ba if stationary else None,bg=bg if stationary else None,velocity=velocity,stationary=stationary)
            result=dict(accepted=True,reason='initialized_stationary' if stationary else 'initialized_moving',jump_m=0.,nis=0.)
            self.stats['initializations']+=1
        else:result=f.gnss(pos,cov)
        if result['accepted']:
            self.stats['used_gnss']+=1;self.used['gnss']=t;self.last_observation=t
        else:self.reject('gnss',result['reason'])
        event=dict(stamp_ns=t,elapsed_s=elapsed,mode=mode,**result)
        self.corrections.append(event);self.corrections=self.corrections[-200:]
        if recovering:self.recoveries.append(event);self.recoveries=self.recoveries[-100:]
        return result

    def fiveg(self,t,links):
        if self.o['mode']!='gnss_imu_5g':return
        for d in links:
            name=str(d['id'])
            if not d['valid']:self.reject('fiveg','invalid',name);continue
            if not np.isfinite(np.r_[d['anchor'],d['range'],d['sigma'],d['rsrp']]).all() or d['range']<=0 or d['sigma']<0:self.reject('fiveg','nonfinite_or_range',name);continue
            if d['rsrp']<self.o['fiveg_min_rsrp_dbm']:self.reject('fiveg','low_rsrp',name);continue
            if self.filter.t is None:self.reject('fiveg','waiting_initialization',name);continue
            result=self.filter.range(d['anchor'],d['range'],d['sigma'])
            if result['accepted']:
                self.stats['used_fiveg']+=1;self.used['fiveg']=t;self.last_observation=t
                v=self.anchors.setdefault(name,dict(accepted=0,rejected=0));v['accepted']+=1;v.update(reason='accepted',stamp_ns=t,nis=result.get('nis'))
            else:self.reject('fiveg',result['reason'],name)

    def diagnostic(self):
        return dict(status=self.status,reset_reason=self.reset_reason,queue=len(self.queue),
            counts=dict(self.stats),last_received_ns=self.received,last_used_ns=self.used,
            last_rejection=self.last_rejection,anchors=self.anchors,gnss_fault_mode=self.fault,
            recovery_events=self.recoveries,recent_corrections=self.corrections[-10:],
            imu_orientation_for_initial_heading=True,truth_used_for_estimation=False)
