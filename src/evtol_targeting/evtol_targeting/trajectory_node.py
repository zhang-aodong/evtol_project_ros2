"""Read-only mission and traveled paths in the current session's ENU frame."""
import math
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from evtol_perception_interfaces.msg import ReferenceOrigin
from evtol_simulation.runtime import SensorLease
from .common import LATCHED, args_config, clone, header, ns


class TrajectoryNode(Node):
    def __init__(self, c):
        super().__init__('trajectory_visualization')
        root='/' + c['namespace'].strip('/')
        options=c.get('trajectory', {})
        self.spacing=float(options.get('min_distance_m', .05))
        self.max_points=int(options.get('max_points', 20000))
        self.initial_spacing=self.spacing
        self.waypoints=c['mission']['waypoints_enu_m']
        self.session=None;self.origin_ns=0;self.last_ns=0;self.waiting_for_origin=False
        self.points=[];self.latest=None;self.dirty=False
        self.waypoint_pub=self.create_publisher(Path,root+'/navigation/waypoint_path',LATCHED)
        self.traveled_pub=self.create_publisher(Path,root+'/navigation/traveled_path',LATCHED)
        self.create_subscription(ReferenceOrigin,'/simulation/reference_origin',self.receive_origin,LATCHED)
        self.create_subscription(Odometry,root+'/state/odometry',self.receive_pose,qos_profile_sensor_data)
        self.create_timer(1./float(options.get('publish_hz',2.)),self.publish_traveled,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('Publishing waypoint_path and traveled_path in map_enu (read only)')

    def clear_traveled(self, stamp):
        self.points=[];self.latest=None;self.last_ns=0;self.spacing=self.initial_spacing
        self.dirty=False
        self.traveled_pub.publish(Path(header=header(stamp)))

    def receive_origin(self, msg):
        if not msg.session_id or msg.header.frame_id!='map_enu':return
        if msg.session_id==self.session:return
        self.session=msg.session_id;self.origin_ns=ns(msg.header.stamp)
        self.waiting_for_origin=False
        self.clear_traveled(self.origin_ns)
        route=Path(header=header(self.origin_ns))
        for waypoint in self.waypoints:
            pose=PoseStamped(header=clone(route.header))
            pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=map(float,waypoint)
            pose.pose.orientation.w=1.
            route.poses.append(pose)
        self.waypoint_pub.publish(route)

    def receive_pose(self, msg):
        if self.session is None or msg.header.frame_id!='map_enu':return
        stamp=ns(msg.header.stamp)
        if stamp<=0:return
        if self.last_ns and stamp<self.last_ns:
            # A reset must never draw a line between different simulation runs.
            self.clear_traveled(stamp);self.waiting_for_origin=True
            self.waypoint_pub.publish(Path(header=header(stamp)))
            return
        if self.waiting_for_origin or stamp<self.origin_ns or stamp==self.last_ns:return
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        if not all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w)):return
        if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)>.01:return
        pose=PoseStamped(header=clone(msg.header),pose=clone(msg.pose.pose))
        self.last_ns=stamp;self.latest=pose
        if not self.points or self.distance(self.points[-1],pose)>=self.spacing:
            self.points.append(pose)
            if len(self.points)>self.max_points:
                # Retain the whole session's extent, including its first point;
                # progressively simplify old samples rather than crop the tail.
                last=self.points[-1]
                self.points=self.points[::2]
                if ns(self.points[-1].header.stamp)!=stamp:self.points.append(last)
                self.spacing*=2.
                self.get_logger().info(f'Trajectory simplified to {len(self.points)} points; spacing {self.spacing:g} m')
        self.dirty=True

    @staticmethod
    def distance(a,b):
        a=a.pose.position;b=b.pose.position
        return math.sqrt((a.x-b.x)**2+(a.y-b.y)**2+(a.z-b.z)**2)

    def publish_traveled(self):
        if not self.dirty or self.latest is None:return
        poses=list(self.points)
        if ns(poses[-1].header.stamp)!=ns(self.latest.header.stamp):poses.append(self.latest)
        self.traveled_pub.publish(Path(header=clone(self.latest.header),poses=poses))
        self.dirty=False


def main(args=None):
    c,ros=args_config(args)
    lease=SensorLease(dict(backend='airsim',airsim=c['airsim']),'trajectory_visualization')
    rclpy.init(args=ros);node=TrajectoryNode(c)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown();lease.close()
