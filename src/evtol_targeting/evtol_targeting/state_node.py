"""Truth pose adapter and the sole dynamic TF owner for the algorithm stack."""
import json
import time
import uuid
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from evtol_perception_interfaces.msg import ReferenceOrigin, StateStatus
from .common import args_config, project_path, xyz, put, clone, LATCHED, ns, Health
from evtol_simulation.runtime import SensorLease


class StateNode(Node):
    def __init__(self,c):
        super().__init__('ego_state_adapter')
        self.lease=SensorLease(dict(backend='airsim',airsim=c['airsim']),'state_adapter')
        self.c=c;self.root='/'+c['namespace'];self.health=Health(self,self.root+'/state')
        self.origin=None;self.last_ns=0;self.last_wall=None;self.last_state_ns=0
        self.file=project_path(c,c['state']['origin_file'])
        if self.file.exists():self.saved=json.loads(self.file.read_text(encoding='utf8'))
        else:self.saved=None
        self.origin_pub=self.create_publisher(ReferenceOrigin,'/simulation/reference_origin',LATCHED)
        self.pub=self.create_publisher(Odometry,self.root+'/state/odometry',qos_profile_sensor_data)
        self.truth=self.create_publisher(Odometry,self.root+'/ground_truth/local_odometry',qos_profile_sensor_data)
        self.status=self.create_publisher(StateStatus,self.root+'/state/status',10)
        self.tf=TransformBroadcaster(self)
        self.create_subscription(Odometry,self.root+'/ground_truth/odometry',self.receive_truth,qos_profile_sensor_data)
        if c['state']['source']=='fused':
            self.create_subscription(Odometry,c['state']['fused_topic'],self.receive_fused,qos_profile_sensor_data)
        elif c['state']['source']!='truth':raise ValueError('state.source must be truth or fused')
        self.create_timer(.2,self.report)

    def receive_truth(self,msg):
        try:
            t=ns(msg.header.stamp)
            if t<=0:raise ValueError('Missing truth stamp')
            if self.last_ns and t<self.last_ns:
                self.origin=None;self.saved=None;self.last_state_ns=0;self.last_wall=None
            if self.origin is None:
                saved=self.saved
                if saved and (saved['stamp_ns']>t or saved['source_frame']!=msg.header.frame_id):saved=None
                if saved is None:
                    q=msg.pose.pose.orientation
                    saved=dict(session_id=uuid.uuid4().hex,stamp_ns=t,source_frame=msg.header.frame_id,
                        position=xyz(msg.pose.pose.position).tolist(),orientation=[q.x,q.y,q.z,q.w])
                    self.file.parent.mkdir(parents=True,exist_ok=True)
                    tmp=self.file.with_suffix('.tmp');tmp.write_text(json.dumps(saved,indent=2),encoding='utf8');tmp.replace(self.file)
                self.saved=saved
                self.origin=ReferenceOrigin(header=clone(msg.header),session_id=saved['session_id'],source_frame=saved['source_frame'])
                self.origin.header.frame_id='map_enu'
                from .common import stamp
                self.origin.header.stamp=stamp(saved['stamp_ns'])
                put(self.origin.source_origin_enu,saved['position'])
                q=self.origin.initial_orientation;q.x,q.y,q.z,q.w=saved['orientation']
                self.origin_pub.publish(self.origin)
            local=clone(msg);local.header.frame_id='map_enu'
            put(local.pose.pose.position,xyz(msg.pose.pose.position)-xyz(self.origin.source_origin_enu))
            self.truth.publish(local)
            self.last_ns=t
            if self.c['state']['source']=='truth':self.publish_state(local)
        except Exception as exc:self.health.error(exc)

    def receive_fused(self,msg):
        if self.origin is None:return
        if msg.header.frame_id!='map_enu':
            self.health.error('Fused input must use the same session map_enu');return
        try:self.publish_state(msg)
        except Exception as exc:self.health.error(exc)

    def publish_state(self,msg):
        from .common import odom_arrays
        arrays=odom_arrays(msg)
        import numpy as np
        if not all(np.isfinite(a).all() for a in arrays):raise ValueError('Non-finite state')
        t=ns(msg.header.stamp)
        age=(self.get_clock().now().nanoseconds-t)*1e-9
        if t<=self.last_state_ns or t<ns(self.origin.header.stamp):return
        if msg.child_frame_id!=self.c['namespace']+'/base_link':raise ValueError('Wrong state child frame')
        if self.c['state']['source']=='fused' and not -.05<=age<=self.c['state']['timeout_s']:
            self.health.details=dict(state_stamp_age_s=age,reason='stale_state_stamp');return
        self.last_state_ns=t
        self.pub.publish(msg)
        tf=TransformStamped(header=clone(msg.header),child_frame_id=msg.child_frame_id)
        tf.transform.translation.x=msg.pose.pose.position.x
        tf.transform.translation.y=msg.pose.pose.position.y
        tf.transform.translation.z=msg.pose.pose.position.z
        tf.transform.rotation=clone(msg.pose.pose.orientation)
        self.tf.sendTransform(tf);self.last_wall=time.perf_counter();self.health.ok()

    def report(self):
        age=time.perf_counter()-self.last_wall if self.last_wall else 1e9
        from .common import header
        stamp_age=(self.get_clock().now().nanoseconds-self.last_state_ns)*1e-9 if self.last_state_ns else 1e9
        valid=age<self.c['state']['timeout_s'] and -.05<=stamp_age<self.c['state']['timeout_s']
        self.health.details=dict(state_stamp_age_s=stamp_age,receive_age_s=age,source=self.c['state']['source'])
        self.status.publish(StateStatus(header=header(self.get_clock().now().nanoseconds),session_id=self.origin.session_id if self.origin else '',
            source=self.c['state']['source'],valid=valid,
            reason='streaming' if valid else 'no_fresh_pose_or_stale_stamp',age_s=max(age,stamp_age)))


def main(args=None):
    c,ros=args_config(args);rclpy.init(args=ros);node=StateNode(c)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.lease.close();node.destroy_node();rclpy.shutdown()
