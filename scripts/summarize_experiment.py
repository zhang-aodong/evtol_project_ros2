"""Recompute range scores from recorded raw counts, no ROS dependency."""
import argparse
import csv
import json
from pathlib import Path


def summarize(folder):
    folder=Path(folder)
    rows=[json.loads(line) for line in (folder/'metrics.jsonl').read_text(encoding='utf8').splitlines() if line.strip()]
    scoring=[r for r in rows if r.get('scoring',True)]
    valid=[r for r in scoring if r['valid']]
    tp=sum(r['detected_count'] for r in valid);truth=sum(r['truth_count'] for r in valid);fp=sum(r['false_positive_count'] for r in valid)
    result=dict(schema_version=1,evaluated_epochs=len(rows),scoring_epochs=len(scoring),valid_epochs=len(valid),invalid_epochs=len(scoring)-len(valid),
        valid_ratio=len(valid)/len(scoring) if scoring else None,truth_exposures=truth,matched_exposures=tp,false_positive_exposures=fp,
        cumulative_recall=tp/truth if truth else None,precision=tp/(tp+fp) if tp+fp else None,
        collision_count=valid[-1].get('collision_count') if valid else None,
        minimum_center_distance_m=valid[-1].get('minimum_center_distance_m') if valid else None,
        minimum_conservative_sphere_gap_m=valid[-1].get('minimum_conservative_sphere_gap_m') if valid else None,
        encounters=valid[-1].get('encounters') if valid else None,
        caveat='frame/epoch scores are correlated; do not claim independent-event confidence from these counts')
    groups={}
    for row in scoring:
        key=(row['session_id'],row.get('evaluation_segment',0))
        group=groups.setdefault(key,dict(session_id=key[0],evaluation_segment=key[1],valid=0,invalid=0,tp=0,truth=0,fp=0))
        group['valid' if row['valid'] else 'invalid']+=1
        if row['valid']:
            group['tp']+=row['detected_count'];group['truth']+=row['truth_count'];group['fp']+=row['false_positive_count']
    for group in groups.values():group['recall']=group['tp']/group['truth'] if group['truth'] else None
    result['segments']=list(groups.values())
    with (folder/'metrics.csv').open('w',encoding='utf-8-sig',newline='') as f:
        fields=['session_id','stamp_s','valid','reason','radius_m','truth_count','detected_count','missed_count','false_positive_count','recall','window_recall','cumulative_recall','timely_success_rate']
        writer=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');writer.writeheader();writer.writerows(rows)
    (folder/'recomputed_summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('folder');opts=parser.parse_args()
    print(json.dumps(summarize(opts.folder),ensure_ascii=False,indent=2))
