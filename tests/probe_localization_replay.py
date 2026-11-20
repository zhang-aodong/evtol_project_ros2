"""Replay real sensor inputs into ROS estimator/evaluator in a private domain.

No simulator, state adapter, sensor producer or dynamic TF is launched.
"""
import argparse,json,os,subprocess,sys,time,shutil
from pathlib import Path
import yaml
import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from evtol_localization.node import SENSOR_QOS

def main():
    p=argparse.ArgumentParser();p.add_argument('bag',type=Path);p.add_argument('--config',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--domain',type=int,default=224)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    c=yaml.safe_load(a.config.read_text(encoding='utf-8-sig'));c['project_root']=str(a.output.resolve())
    c['evaluation']['output_dir']='evaluation';config=a.output/'targeting.yaml';config.write_text(yaml.safe_dump(c),encoding='utf8')
    os.environ['ROS_DOMAIN_ID']=str(a.domain);rclpy.init(domain_id=a.domain)
    node=rclpy.create_node('recorded_localization_probe');messages=[];metrics=[];statuses=[]
    prefix='/'+c['namespace']
    node.create_subscription(Odometry,prefix+'/localization/odometry',lambda m:messages.append(m),SENSOR_QOS)
    node.create_subscription(String,prefix+'/localization/metrics',lambda m:metrics.append(json.loads(m.data)),10)
    node.create_subscription(String,prefix+'/localization/status',lambda m:statuses.append(json.loads(m.data)),10)
    processes=[];files=[];failed=None;result={}
    def start(name,cmd):
        f=(a.output/(name+'.log')).open('wb');files.append(f)
        child=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,
            creationflags=subprocess.CREATE_NO_WINDOW if os.name=='nt' else 0)
        processes.append(child);return child
    def spin(seconds):
        until=time.perf_counter()+seconds
        while time.perf_counter()<until:rclpy.spin_once(node,timeout_sec=.01)
    try:
        for name,module in [('estimator','evtol_localization.node'),('evaluation','evtol_localization.evaluation_node')]:
            start(name,[sys.executable,'-u','-m',module,'--config',str(config),'--ros-args','-p','use_sim_time:=true'])
        spin(2.)
        topics=['/simulation/reference_origin','/simulation/geodetic_reference']+[prefix+'/'+s for s in ('imu/data','gnss/fix','five_ga/observations','ground_truth/local_odometry','control/flight_status')]
        player=start('player',[shutil.which('ros2'),'bag','play',str(a.bag),'--clock','100','--rate','2','--topics',*topics])
        deadline=time.perf_counter()+100
        while player.poll() is None and time.perf_counter()<deadline:spin(.1)
        if player.poll() is None:raise TimeoutError('Bag replay timed out')
        if player.returncode:raise RuntimeError('Bag player exit '+str(player.returncode))
        spin(1.)
        if len(messages)<100 or not metrics:raise AssertionError('Estimator/evaluator did not produce outputs')
        stamps=[m.header.stamp.sec*10**9+m.header.stamp.nanosec for m in messages]
        if any(x>=y for x,y in zip(stamps,stamps[1:])):raise AssertionError('Output stamp reversal or duplicate')
        last=metrics[-1]
        if last['summary']['coverage']<.9:raise AssertionError('Replay alignment coverage below .9')
        if not statuses[-1]['recovery_events']:raise AssertionError('No replayed GNSS recovery event')
        result=dict(messages=len(messages),metrics=last,status=statuses[-1],private_domain=a.domain,
            no_simulator=True,no_dynamic_tf=True,clock_source='bag_player',recorded_estimates_replayed=False)
    except BaseException as exc:failed=str(exc)
    finally:
        for child in reversed(processes):
            if child.poll() is None:child.terminate()
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:child.kill();child.wait(timeout=5)
        for f in files:f.close()
        node.destroy_node();rclpy.shutdown()
        result['error']=failed;(a.output/'result.json').write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf8')
    if failed:raise RuntimeError(failed)
    print(json.dumps(dict(messages=result['messages'],summary=result['metrics']['summary']),ensure_ascii=False))

if __name__=='__main__':main()
