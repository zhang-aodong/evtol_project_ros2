"""Summarize every test, including failures; never select a peak score."""
from pathlib import Path
import argparse,json,math,statistics

def wilson(k,n):
    if not n:return None
    z=1.96;p=k/n;den=1+z*z/n;center=(p+z*z/(2*n))/den
    delta=z*math.sqrt(p*(1-p)/n+z*z/(4*n*n))/den
    return [center-delta,center+delta]

def main():
    p=argparse.ArgumentParser();p.add_argument('--root',required=True,type=Path);p.add_argument('--labels',nargs='+',required=True)
    p.add_argument('--final-labels',nargs='*',default=[])
    a=p.parse_args();runs=[]
    for label in a.labels:
        folder=a.root/'reports'/('perception_'+label);result=json.loads((folder/'result.json').read_text(encoding='utf8'))
        run=Path(result['run']);metrics=next((run/'evaluation').glob('range_*/metrics.jsonl'),None);last_valid=None;last_any=None
        if metrics is None:
            result.update(label=label,is_final_configuration=label in a.final_labels,meets_95=False,
                acceptance_limitation='No scoring metrics (startup/flight failed); this run is retained as a failure.')
            runs.append(result);continue
        images=[];sensor=[];collisions=0;skipped=0;output=[];jitter=[];scoring=collision_valid=0
        for line in metrics.open(encoding='utf8'):
            row=json.loads(line);last_any=row
            if row.get('valid') and row.get('scoring'):last_valid=row
            skipped+=row.get('skipped_epochs',0);collisions=max(collisions,row.get('collision_count',0))
            if row.get('scoring'):
                scoring+=1;collision_valid+=bool(row.get('collision_status_valid'))
        latency=[]
        for line in (metrics.parent/'matches.jsonl').open(encoding='utf8'):
            row=json.loads(line);latency.append((row['available_s']-row['stamp_s'])*1000)
        telemetry=json.loads((folder/'telemetry.json').read_text(encoding='utf8'))
        for sample in telemetry:
            diag=sample['telemetry'].get('diagnostics',{}).get('/drone1/target_localizer',{}).get('values',{})
            try:images.append(float(diag['image_to_output_ms']))
            except (KeyError,ValueError):pass
        enc=(last_valid or {}).get('encounters',{});n=sum(enc.get(k,0) for k in ('success','late','missed'));success=enc.get('success',0)
        invalid_reasons={}
        for line in metrics.open(encoding='utf8'):
            row=json.loads(line)
            if row.get('scoring') and not row.get('valid'):
                reason=row.get('reason','unknown');invalid_reasons[reason]=invalid_reasons.get(reason,0)+1
        result.update(label=label,is_final_configuration=label in a.final_labels,
            historical_probe_pass_not_final_acceptance=bool(result.get('meets_95')) and label not in a.final_labels,
            invalid_scoring_reasons=invalid_reasons,encounters=enc,completed_encounters=n,timely_success_rate=success/n if n else None,
            timely_wilson_95_interval=wilson(success,n),collision_count=collisions,skipped_evaluation_epochs=skipped,
            last_valid_stamp_s=(last_valid or {}).get('stamp_s'),
            image_to_output_ms_sampled_median=statistics.median(images) if images else None,
            depth_to_available_ms_median=statistics.median(latency) if latency else None,
            evidence_s=.3,timely_s=.5,radius_m=20.,truth_used_by_detection=False)
        result['collision_status_valid_fraction']=collision_valid/scoring if scoring else None
        result['final_acceptance_pass']=bool(label in a.final_labels and result.get('meets_95') and collisions==0)
        runs.append(result)
    report=dict(schema_version=1,criterion='Each entry into the full 20m sphere detected within 0.5s',
        final_labels=a.final_labels,runs=runs,
        statistical_note='Repeated encounters from the same actors are correlated; Wilson intervals are descriptive only. A finite test is not a universal guarantee.')
    report['final_acceptance_pass']=bool(a.final_labels) and all(
        any(r.get('label')==label and r.get('final_acceptance_pass') for r in runs) for label in a.final_labels)
    target=a.root/'docs/perception_recall_acceptance_20261119.json'
    target.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8');print(target)

if __name__=='__main__':main()
