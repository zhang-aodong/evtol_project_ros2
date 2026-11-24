"""Independent truth subscriber. Estimator outputs never flow backwards."""
import json
from itertools import islice
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry,Path as NavPath
from geometry_msgs.msg import Vector3Stamped,PoseStamped
from std_msgs.msg import String,Float64
from evtol_perception_interfaces.msg import ReferenceOrigin
from evtol_targeting.common import args_config,ns,xyz,put,header,clone,LATCHED,project_path
from .qos import SENSOR_QOS
from .options import options,fault_at
from .evaluation import ErrorAudit
from .storage import EvaluationStorage


class EvaluationNode(Node):
    def __init__(self,c):
        super().__init__('ego_localization_evaluation');self.c=c;self.o=options(c.get('localization'))
        root='/'+c['namespace'];self.audit=ErrorAudit();self.origin=None;self.status={};self.base_ns=None
        self.truth_path=[];self.last_truth_path=0;self.last_saved=0;self.folder=None;self.storage=None;self.last_storage_warning=0.
        self.pub=self.create_publisher(String,root+'/localization/metrics',10)
        self.error=self.create_publisher(Vector3Stamped,root+'/localization/error_enu',SENSOR_QOS)
        self.norm=self.create_publisher(Float64,root+'/localization/error_3d',SENSOR_QOS)
        self.truth_pub=self.create_publisher(NavPath,root+'/localization/truth_path',LATCHED)
        self.create_subscription(ReferenceOrigin,'/simulation/reference_origin',self.reference,LATCHED)
        self.create_subscription(Odometry,root+'/ground_truth/local_odometry',self.truth,SENSOR_QOS)
        self.create_subscription(Odometry,root+'/localization/odometry',self.estimate,SENSOR_QOS)
        self.create_subscription(String,root+'/localization/status',self.diagnostic,10)
        self.create_timer(.05,self.tick)

    def reference(self,msg):
        if self.origin and self.origin.session_id==msg.session_id:return
        if self.storage:self.storage.close()
        self.origin=msg;self.audit=ErrorAudit();self.truth_path=[];self.last_truth_path=0;self.last_saved=0;self.status={};self.base_ns=None
        self.folder=project_path(self.c,self.c['evaluation']['output_dir'])/('ego_'+msg.session_id)/self.o['mode']
        self.storage=EvaluationStorage(self.folder)
        self.truth_pub.publish(NavPath(header=header(ns(msg.header.stamp))))

    def estimate(self,msg):
        if not self.origin or msg.header.frame_id!='map_enu':return
        q=msg.pose.pose.orientation
        self.audit.estimate(ns(msg.header.stamp),xyz(msg.pose.pose.position),np.array([q.x,q.y,q.z,q.w]))

    def truth(self,msg):
        if not self.origin:return
        q=msg.pose.pose.orientation;t=ns(msg.header.stamp)
        self.audit.truth(t,xyz(msg.pose.pose.position),np.array([q.x,q.y,q.z,q.w]))
        if t-self.last_truth_path>=int(.2e9):
            self.truth_path.append(PoseStamped(header=clone(msg.header),pose=clone(msg.pose.pose)))
            self.truth_path=self.truth_path[-20000:];self.last_truth_path=t
            self.truth_pub.publish(NavPath(header=clone(msg.header),poses=self.truth_path))

    def diagnostic(self,msg):
        try:
            s=json.loads(msg.data)
            if self.origin and s.get('session_id')==self.origin.session_id:self.status=s;self.base_ns=s.get('gnss_fault_time_base_ns')
        except ValueError:pass

    def phase(self,t):
        elapsed=(t-self.base_ns)*1e-9 if self.base_ns is not None else None
        return fault_at(self.o['gnss_fault'],elapsed)[0]

    def tick(self):
        if not self.origin:return
        now=self.get_clock().now().nanoseconds
        for r in self.audit.drain(now-int(.55e9),self.phase):
            vector=Vector3Stamped(header=header(r['stamp_ns']));put(vector.vector,r['error_enu_m'])
            self.error.publish(vector);self.norm.publish(Float64(data=r['error_3d_m']))
            if self.storage:self.storage.append(r)
        if now-self.last_saved<int(.5e9):return
        self.last_saved=now
        s=self.audit.summary();latest=self.audit.last
        fresh=latest is not None and 0<=(now-latest['stamp_ns'])*1e-9<1.
        report=dict(session_id=self.origin.session_id,origin_stamp_ns=ns(self.origin.header.stamp),stamp_ns=now,valid=fresh,mode=self.o['mode'],
            current=latest,summary=s,estimator=self.status,alignment='session_enu_no_truth_fitting',
            recent_errors=list(reversed(list(islice(reversed(self.audit.rows),60)))),
            outage_end_errors=self.outage_errors(),recovery_events=self.status.get('recovery_events',[]),
            storage=self.storage.status() if self.storage else None)
        self.pub.publish(String(data=json.dumps(report,allow_nan=False)))
        if self.storage:
            was_failed=self.storage.error is not None
            saved=self.storage.save(report)
            wall=time.monotonic()
            if self.storage.error and wall-self.last_storage_warning>=5.:
                self.get_logger().warning('Evaluation storage unavailable; live error topics continue: '+self.storage.error)
                self.last_storage_warning=wall
            elif saved and was_failed:self.get_logger().info('Evaluation storage recovered')

    def outage_errors(self):
        if self.base_ns is None:return []
        result=[]
        for seg in self.o['gnss_fault']['segments']:
            if seg['mode']=='normal':continue
            end=self.base_ns+int(seg['end_s']*1e9);start=self.base_ns+int(seg['start_s']*1e9)
            if self.audit.last is None or self.audit.last['stamp_ns']<end:continue
            row=next((r for r in reversed(self.audit.rows) if start<=r['stamp_ns']<=end),None)
            result.append(dict(start_s=seg['start_s'],end_s=seg['end_s'],mode=seg['mode'],last_error=row,
                end_gap_s=(end-row['stamp_ns'])*1e-9 if row else None))
        return result

    def destroy_node(self):
        if self.storage:self.storage.close()
        return super().destroy_node()


def main(args=None):
    c,ros=args_config(args);rclpy.init(args=ros);node=EvaluationNode(c)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
