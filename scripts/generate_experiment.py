"""Generate portable scene configuration files. Does not launch or arm vehicles."""
import argparse
import json
from pathlib import Path
from evtol_console.configuration import defaults,artifacts


def generate(root,count,output,seed=42):
    import random
    rng=random.Random(seed);d=defaults(root);d['actors']['agents']=[];d['actors']['count']=count
    # Lateral crossing lanes. The first waypoint is a staging point; the
    # console waits until every actor has reached it before enabling ego.
    for i in range(count):
        north=25.+(i//4)*12.;east=(-1 if i%2 else 1)*(10.+(i%4)*3.)
        height=10.+(i%3-1)*3.
        d['actors']['agents'].append(dict(name='Drone'+str(i+2),speed_mps=1.5+rng.random(),
            spawn_ned_m=[north,east,0.],waypoints_enu_m=[[east,north,height],[-east,north,height]],repeat=0))
    d['actors']['pedestrians_enabled']=False
    d['mission'].update(mode='waypoints',waypoints_enu_m=[[0,0,10],[0,max(100.,25+(count//4)*12.),10]],land_at_last=False)
    d['run']['execute']=False  # deliberate: review first, enable explicitly in panel
    d,_,_,_,_=artifacts(d,root)
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(d,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf8')
    return output


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--count',type=int,choices=[4,8,16,32,64],default=4)
    parser.add_argument('--root',default=str(Path(__file__).resolve().parents[1]));parser.add_argument('--output',required=True)
    parser.add_argument('--seed',type=int,default=42);opts=parser.parse_args()
    print(generate(Path(opts.root),opts.count,opts.output,opts.seed))
