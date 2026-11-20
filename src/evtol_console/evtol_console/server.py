"""One entry point: persistent manager + Foxglove bridge + local web console."""
import argparse
from collections import deque
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import json
from pathlib import Path
import sys
import threading
import time
import foxglove
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticArray
from evtol_perception_interfaces.msg import ReferenceOrigin
from evtol_targeting.common import LATCHED
from evtol_simulation.runtime import SensorLease
from evtol_visualization.bridge import Bridge
from .engine import Engine
from .processes import OwnedProcess


REQUEST_SCHEMA={'type':'object','properties':{'action':{'type':'string'},'payload':{'type':'object'}},'required':['action']}
RESPONSE_SCHEMA={'type':'object','properties':{'ok':{'type':'boolean'},'error':{'type':'string'},'data':{'type':'object'}}}


def ros_stop(service,timeout,cancel=None):
    # Separate worker-owned node/executor; never block the visualization executor.
    node=rclpy.create_node('console_cleanup_'+str(time.monotonic_ns()))
    executor=SingleThreadedExecutor();executor.add_node(node)
    client=node.create_client(Trigger,service)
    try:
        if not client.wait_for_service(timeout_sec=3.):raise RuntimeError(service+' is unavailable')
        future=client.call_async(Trigger.Request());deadline=time.monotonic()+timeout
        while not future.done() and time.monotonic()<deadline:
            if cancel is not None and cancel.is_set():
                future.cancel();raise RuntimeError(service+' interrupted by force stop')
            executor.spin_once(timeout_sec=.1)
        if not future.done():raise TimeoutError(service+' cleanup timed out')
        result=future.result()
        if not result.success:raise RuntimeError(result.message)
        return service+': '+result.message
    finally:executor.shutdown();node.destroy_node()


def service_for(engine):
    def schema(name,data):return foxglove.MessageSchema(encoding='json',schema=foxglove.Schema(name=name,encoding='jsonschema',data=json.dumps(data).encode()))
    def handler(request):
        try:
            if request.encoding!='json':raise ValueError('Console service requires JSON encoding')
            if len(request.payload)>1_048_576:raise ValueError('Request too large')
            result=engine.request(json.loads(request.payload))
        except Exception as exc:result=dict(ok=False,error=str(exc))
        return json.dumps(result,ensure_ascii=False,allow_nan=False).encode('utf8')
    return foxglove.Service('/simulation/console',schema=foxglove.ServiceSchema(name='evtol/Console',
        request=schema('evtol/ConsoleRequest',REQUEST_SCHEMA),response=schema('evtol/ConsoleResponse',RESPONSE_SCHEMA)),handler=handler)


def http_server(engine,port):
    assets=Path(__file__).with_name('web')
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def write(self,code,data,mime='application/json; charset=utf-8'):
            self.send_response(code);self.send_header('Content-Type',mime);self.send_header('Cache-Control','no-store')
            self.send_header('X-Content-Type-Options','nosniff');self.end_headers();self.wfile.write(data)
        def do_GET(self):
            if self.path=='/api/status':return self.write(200,json.dumps(engine.request({'action':'status'}),ensure_ascii=False).encode('utf8'))
            name={'/':'index.html','/index.html':'index.html','/app.js':'app.js'}.get(self.path)
            if not name:return self.write(404,b'{}')
            self.write(200,(assets/name).read_bytes(),'application/javascript; charset=utf-8' if name.endswith('.js') else 'text/html; charset=utf-8')
        def do_POST(self):
            # Same-origin local fallback only. Foxglove uses its WebSocket
            # service and never needs permissive browser CORS to this server.
            if self.path!='/api/command':return self.write(404,b'{}')
            if self.headers.get('Origin') not in (None,f'http://127.0.0.1:{self.server.server_port}',f'http://localhost:{self.server.server_port}'):
                return self.write(403,b'{"ok":false,"error":"Origin is not allowed"}')
            if not self.headers.get('Content-Type','').startswith('application/json'):return self.write(415,b'{}')
            try:
                size=int(self.headers.get('Content-Length','0'))
                if not 0<size<=1_048_576:raise ValueError('Invalid request size')
                result=engine.request(json.loads(self.rfile.read(size)))
                self.write(200,json.dumps(result,ensure_ascii=False,allow_nan=False).encode('utf8'))
            except Exception as exc:self.write(400,json.dumps({'ok':False,'error':str(exc)}).encode())
    server=ThreadingHTTPServer(('127.0.0.1',port),Handler)
    server.daemon_threads=True
    return server


class ConsoleBridge(Bridge):
    def __init__(self,engine,namespace,port):
        self.engine=engine
        super().__init__(namespace,port=port,services=[service_for(engine)])
        self.ros_status=self.create_publisher(String,'/simulation/console/status',10)
        self.control_services=[self.create_service(Trigger,'/simulation/console/'+action,
            self.trigger(action)) for action in ('start_ue','stop_ue','start_ros','stop_ros','stop_all','cancel','force_stop_all')]
        self.samples=deque(maxlen=1500);self.counts={};root='/'+namespace.strip('/')
        self.session=None
        self.create_subscription(ReferenceOrigin,'/simulation/reference_origin',self.origin,LATCHED)
        self.create_subscription(Odometry,root+'/state/odometry',self.odom,qos_profile_sensor_data)
        self.create_subscription(String,root+'/mission/status',lambda m:self.text('mission',m),10)
        self.create_subscription(String,root+'/control/flight_status',lambda m:self.text('flight',m),10)
        self.create_subscription(String,'/scenario/status',lambda m:self.text('actors',m),LATCHED)
        self.create_subscription(DiagnosticArray,'/diagnostics',self.diagnostics,10)
        for topic,key in [('/evaluation/detection_metrics','detection_metrics'),('/evaluation/warning_metrics','warning_metrics'),('/simulation/truth/status','truth_status'),(root+'/localization/status','localization_status'),(root+'/localization/metrics','localization_metrics')]:
            self.create_subscription(String,topic,lambda m,k=key:self.metric(k,m),10)
        self.status_channel=foxglove.Channel('/simulation/console/status',message_encoding='json',schema=foxglove.Schema(
            name='evtol/ConsoleStatus',encoding='jsonschema',data=json.dumps({'type':'object'}).encode()))
        self.create_timer(1.,self.console_status)
        self.create_timer(1.,self.collection_topics)
    def origin(self,msg):
        if msg.session_id!=self.session:
            self.session=msg.session_id;self.samples.clear()
            with self.engine.lock:
                for key in ('localization_status','localization_metrics','localization_status_received_wall','localization_metrics_received_wall','detection_metrics','warning_metrics','truth_status','detection_metrics_received_wall','warning_metrics_received_wall','truth_status_received_wall','mission','flight','actors','mission_received_wall','flight_received_wall','actors_received_wall','profile_actual','position_enu_m','speed_mps','stamp_s','received_wall'):
                    self.engine.telemetry.pop(key,None)
    def trigger(self,action):
        def callback(request,response):
            result=self.engine.request({'action':action})
            response.success=result['ok'];response.message=json.dumps(result,ensure_ascii=False)
            return response
        return callback
    def text(self,key,msg):
        try:
            value=json.loads(msg.data)
            with self.engine.lock:
                old=self.engine.telemetry.get(key);self.engine.telemetry[key]=value
                self.engine.telemetry[key+'_received_wall']=time.time()
            fields=('phase','reason','waypoint_index','waypoint','completed')
            changed=old!=value if key!='mission' else old is None or any(old.get(k)!=value.get(k) for k in fields)
            if changed:self.engine.log(json.dumps(value,ensure_ascii=False),source=key,
                level='ERROR' if value.get('phase')=='failed' else 'INFO')
        except ValueError:pass
    def odom(self,msg):
        import math
        p=msg.pose.pose.position;v=msg.twist.twist.linear;t=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        speed=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
        if not all(math.isfinite(x) for x in (t,speed,p.x,p.y,p.z)):return
        if self.samples and t<self.samples[-1]['t']:self.samples.clear()
        self.samples.append(dict(t=t,speed_mps=speed,height_m=p.z))
        with self.engine.lock:self.engine.telemetry.update(position_enu_m=[p.x,p.y,p.z],speed_mps=speed,stamp_s=t,
            received_wall=time.time(),profile_actual=list(self.samples)[::max(1,len(self.samples)//250)])
        if time.monotonic()-getattr(self,'last_motion_log',0.)>5.:
            self.last_motion_log=time.monotonic()
            self.engine.log(f'本机 ENU=({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) m，地速={speed:.2f} m/s',source='ego')
    def metric(self,key,msg):
        try:
            value=json.loads(msg.data)
            if self.session and value.get('session_id')!=self.session:return
            with self.engine.lock:
                self.engine.telemetry[key]=value;self.engine.telemetry[key+'_received_wall']=time.time()
        except (ValueError,TypeError):pass

    def diagnostics(self,msg):
        with self.engine.lock:
            statuses=self.engine.telemetry.setdefault('diagnostics',{})
            for s in msg.status:
                # ROS 'byte' maps to one-byte bytes on this Windows build.
                level=int.from_bytes(s.level,'little') if isinstance(s.level,(bytes,bytearray)) else int(s.level)
                statuses[s.name]=dict(level=level,message=s.message,values={v.key:v.value for v in s.values},received=time.time())
    def collection_topics(self):
        topics=[]
        for name,types in self.get_topic_names_and_types():
            infos=self.get_publishers_info_by_topic(name)
            if infos:topics.append(dict(name=name,types=types,publishers=len(infos)))
        with self.engine.lock:self.engine.topic_catalog=sorted(topics,key=lambda t:t['name'])

    def console_status(self):
        data=json.dumps(self.engine.status(),ensure_ascii=False,allow_nan=False)
        self.ros_status.publish(String(data=data))
        self.status_channel.log(data.encode(),log_time=time.time_ns())
    def destroy_node(self):
        self.status_channel.close();return super().destroy_node()


def main(args=None):
    parser=argparse.ArgumentParser();parser.add_argument('--root',required=True);parser.add_argument('--port',type=int,default=9040)
    parser.add_argument('--bridge-port',type=int,default=8765);parser.add_argument('--namespace',default='drone1')
    parser.add_argument('--open-browser',action='store_true')
    parser.add_argument('--config',help='Import and save a validated experiment JSON before launching the manager')
    opts,ros=parser.parse_known_args(args)
    if not 1<=opts.port<=65535 or not 1<=opts.bridge_port<=65535 or opts.port==opts.bridge_port:parser.error('Invalid ports')
    lease=SensorLease(dict(backend='airsim',airsim=dict(host='console',port=opts.port,vehicle='manager')),'console')
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=ros,signal_handler_options=SignalHandlerOptions.NO)
    engine=Engine(opts.root,ros_stop=ros_stop);bridge=None;http=None;preview=None
    try:
        if opts.config:
            value=json.loads(Path(opts.config).read_text(encoding='utf-8-sig'))
            imported=engine.request(dict(action='import_config',payload=value))
            if not imported['ok']:raise ValueError(imported.get('error'))
            saved=engine.request(dict(action='save',payload=imported['data']))
            if not saved['ok']:raise ValueError(saved.get('error'))
        bridge=ConsoleBridge(engine,opts.namespace,opts.bridge_port);http=http_server(engine,opts.port)
        preview=OwnedProcess('preview',[sys.executable,'-m','evtol_visualization.preview','--namespace',opts.namespace,
            '--hz','5','--width','640','--max-points','16000'],engine.root,
            engine.root/'reports'/('console_preview_'+time.strftime('%Y%m%d_%H%M%S')+'.log'))
        thread=threading.Thread(target=http.serve_forever,daemon=True);thread.start()
        print(f'Console: http://127.0.0.1:{http.server_port}',flush=True)
        print(f'Foxglove: ws://127.0.0.1:{bridge.server.port}; panel EVTOL Simulation Console',flush=True)
        if opts.open_browser:
            import webbrowser
            webbrowser.open(f'http://127.0.0.1:{http.server_port}')
        engine.log('Manager ready; UE and ROS are not started')
        rclpy.spin(bridge)
    except KeyboardInterrupt:pass
    finally:
        if http:http.shutdown();http.server_close()
        engine.close()
        if preview:preview.stop()
        if bridge:bridge.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        lease.close()


if __name__=='__main__':main()
