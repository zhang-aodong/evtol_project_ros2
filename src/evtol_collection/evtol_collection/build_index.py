"""Combine independent train/val/test sessions without random video-frame leakage."""
import argparse
import json
from pathlib import Path
import yaml

def build(sessions,output,task):
    if task not in ('detection','segmentation'):raise ValueError('Unsupported task')
    output=Path(output).resolve()
    if output.exists() and any(output.iterdir()):raise ValueError('Index output must be new or empty')
    photos={k:[] for k in ('train','val','test')};mapping=None;sources=[];seen=set()
    for value in sessions:
        session=Path(value).resolve()
        if session in seen:raise ValueError('Duplicate dataset session')
        seen.add(session)
        metadata=json.loads((session/'collection.json').read_text(encoding='utf8'))
        if not metadata.get('complete') or metadata.get('mode')!='dataset':raise ValueError('Session is not a completed dataset: '+str(session))
        schema=json.loads((session/'label_schema.json').read_text(encoding='utf8'))
        if mapping is None:mapping=schema['yolo_to_source']
        elif mapping!=schema['yolo_to_source']:raise ValueError('Class ID mapping differs between sessions')
        if not (session/task).is_dir():raise ValueError('Task missing in session: '+str(session))
        for split in photos:
            images=sorted((session/task/'images'/split).glob('*.jpg'))
            for image in images:
                if not (session/task/'labels'/split/(image.stem+'.txt')).is_file():raise ValueError('Missing label: '+str(image))
            photos[split].extend(images)
        sources.append(str(session))
    if not photos['train'] or not photos['val']:raise ValueError('Collect at least one independent nonempty train AND val session')
    output.mkdir(parents=True,exist_ok=True)
    config=dict(path=str(output).replace('\\','/'))
    for split,images in photos.items():
        if not images:continue
        (output/(split+'.txt')).write_text('\n'.join(p.as_posix() for p in images)+'\n',encoding='utf8')
        config[split]=split+'.txt'
    from .options import CLASSES
    config['names']={int(i):CLASSES[source] for i,source in mapping.items()}
    (output/'dataset.yaml').write_text(yaml.safe_dump(config,sort_keys=False,allow_unicode=True),encoding='utf8')
    (output/'index.json').write_text(json.dumps(dict(task=task,sessions=sources,counts={k:len(v) for k,v in photos.items()},
        note='Regenerate this index after moving datasets to another computer'),ensure_ascii=False,indent=2),encoding='utf8')
    return output/'dataset.yaml'

def main():
    p=argparse.ArgumentParser();p.add_argument('--sessions',nargs='+',required=True);p.add_argument('--output',required=True)
    p.add_argument('--task',choices=['detection','segmentation'],required=True);a=p.parse_args()
    print(build(a.sessions,a.output,a.task))

if __name__=='__main__':main()
