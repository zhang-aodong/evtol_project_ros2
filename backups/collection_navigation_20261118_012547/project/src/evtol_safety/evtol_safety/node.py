import math
from collections import deque
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import json
from evtol_simulation.coordinates import extrinsic, quaternion_matrix
from evtol_perception_interfaces.msg import ReferenceOrigin, StateStatus, TargetArray, Risk, VelocityCommand
from evtol_targeting.common import args_config, ns, header, stamp, xyz, put, odom_arrays, LATCHED, Health
from evtol_targeting.geometry import PoseHistory, cloud_xyz
from .velocity import LocalVelocityAvoidance
from .navigation import WaypointMission,VoxelNavigator
from .profile import make_mission
from .dynamic import DEFAULTS as DYNAMIC_DEFAULTS,from_targets,candidate_clearance
from .async_navigation import AsyncNavigator


def closest_approach(relative_position,relative_velocity,horizon):
    vv=float(relative_velocity@relative_velocity)
    t=float(np.clip(-relative_position@relative_velocity/vv,0,horizon)) if vv>1e-9 else 0.
    return t,float(np.linalg.norm(relative_position+t*relative_velocity))


class SafetyNode(Node):
    def __init__(self,c):
        super().__init__('local_safety');self.c=c;self.root='/'+c['namespace'];self.origin=None
        self.health=Health(self,self.root+'/safety');self.pose=None;self.pose_wall=None;self.state_valid=False
        self.navigation=c.get('navigation_lidar',{}).get('enabled',False)
        self.mission=make_mission(c['mission'])
        self.navigator=(AsyncNavigator if c.get('navigation',{}).get('async_planning',True) else VoxelNavigator)(c.get('navigation',{}))
        self.dynamic={**DYNAMIC_DEFAULTS,**c['safety'].get('dynamic',{})};self.last_risks=0.
        self.tracks_wall=None
        self.lidar_tracks=None;self.lidar_tracks_wall=None
        self.flight_phase='waiting' if self.navigation else 'flying';self.flight_wall=0.
        self.last_plan=0.;self.aim=None;self.plan_goal=None;self.status_reason='waiting'
        self.history=PoseHistory(c['fusion']['pose_max_gap_s']);self.cloud=None;self.cloud_wall=None
        self.cloud_history=deque(maxlen=8)
        self.last_cloud_ns=0;self.tracks=None;self.sequence=0;self.goal_index=0;self.previous_wall=time.perf_counter()
        self.planner=LocalVelocityAvoidance(c['safety']['local_velocity'],sensor_timeout_s=c['safety']['cloud_timeout_s'],
            command_duration_s=c['control']['duration_s'],sweep_retention_s=0.)
        lc=c['extrinsics']['lidar'];self.tl,self.ql=extrinsic(lc['xyz_frd_m'],lc['rpy_deg'])
        self.rbl=quaternion_matrix(self.ql)
        self.pub=self.create_publisher(VelocityCommand,self.root+'/control/safe_velocity',qos_profile_sensor_data)
        self.risk_pub=self.create_publisher(Risk,self.root+'/safety/risk',qos_profile_sensor_data)
        self.risks_pub=self.create_publisher(String,self.root+'/safety/risks',10)
        self.desired_pub=self.create_publisher(VelocityCommand,self.root+'/control/desired_velocity',qos_profile_sensor_data)
        self.mission_pub=self.create_publisher(String,self.root+'/mission/status',LATCHED)
        self.path_pub=self.create_publisher(Path,self.root+'/navigation/path',qos_profile_sensor_data)
        self.create_subscription(String,self.root+'/control/flight_status',self.receive_flight,LATCHED)
        self.create_subscription(ReferenceOrigin,'/simulation/reference_origin',self.receive_origin,LATCHED)
        self.create_subscription(Odometry,self.root+'/state/odometry',self.receive_pose,qos_profile_sensor_data)
        self.create_subscription(StateStatus,self.root+'/state/status',self.receive_status,10)
        self.create_subscription(PointCloud2,self.root+('/navigation/lidar/points' if self.navigation else '/lidar/points'),self.receive_cloud,qos_profile_sensor_data)
        self.create_subscription(TargetArray,self.root+'/perception/tracks',self.receive_tracks,qos_profile_sensor_data)
        self.create_subscription(TargetArray,self.root+'/perception/lidar_obstacles',self.receive_lidar_tracks,qos_profile_sensor_data)
        # Safety checks must keep running during a paused/stalled /clock, and
        # may run faster than the truth publisher that provides /clock.
        self.wall_clock=Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1./c['safety']['rate_hz'],self.tick,clock=self.wall_clock)

    def receive_origin(self,msg):
        if self.origin is None or self.origin.session_id!=msg.session_id:
            self.history.samples.clear();self.cloud=None;self.pose=None;self.tracks=None
            self.lidar_tracks=None;self.lidar_tracks_wall=None
            self.cloud_history.clear()
            self.planner.reset();self.goal_index=0;self.last_cloud_ns=0
            self.mission=make_mission(self.c['mission']);self.aim=None;self.last_plan=0.
            self.state_valid=False
            if hasattr(self.navigator,'close'):self.navigator.close()
        self.origin=msg

    def receive_flight(self,msg):
        try:
            value=json.loads(msg.data)
            if self.origin and value['session_id']==self.origin.session_id:
                self.flight_phase=value['phase'];self.flight_wall=time.perf_counter()
        except (ValueError,KeyError):pass

    def receive_status(self,msg):
        self.state_valid=bool(self.origin and msg.session_id==self.origin.session_id and msg.valid)

    def receive_pose(self,msg):
        if self.origin is None or msg.header.frame_id!='map_enu':return
        try:
            q=msg.pose.pose.orientation
            self.history.add(ns(msg.header.stamp),xyz(msg.pose.pose.position),[q.x,q.y,q.z,q.w])
            self.pose=msg;self.pose_wall=time.perf_counter()
        except Exception as exc:self.health.error(exc)

    def receive_cloud(self,msg):
        expected=self.c['namespace']+('/base_link' if self.navigation else '/lidar_link')
        if msg.header.frame_id!=expected:return
        t=ns(msg.header.stamp)
        if t<=self.last_cloud_ns:return
        self.last_cloud_ns=t;self.cloud=msg;self.cloud_wall=time.perf_counter()
        self.cloud_history.append((msg,self.cloud_wall))

    def receive_tracks(self,msg):
        if self.origin and msg.session_id==self.origin.session_id and msg.header.frame_id=='map_enu':
            self.tracks=msg;self.tracks_wall=time.perf_counter()

    def receive_lidar_tracks(self,msg):
        if self.origin and msg.session_id==self.origin.session_id and msg.header.frame_id=='map_enu':
            self.lidar_tracks=msg;self.lidar_tracks_wall=time.perf_counter()

    def tick(self):
        if self.origin is None:return
        wall=time.perf_counter();dt=min(.1,max(.001,wall-self.previous_wall));self.previous_wall=wall
        now=self.get_clock().now().nanoseconds
        self.sequence+=1
        command=VelocityCommand(header=header(now),session_id=self.origin.session_id,sequence=self.sequence,
            valid_until=stamp(now+int(self.c['control']['ttl_s']*1e9)),reason='no_fresh_data')
        risk=Risk(header=header(now),session_id=self.origin.session_id,valid=False,reason='no_fresh_data',nearest_obstacle_m=-1.)
        try:
            timeout=self.c['safety']['cloud_timeout_s']
            if not self.state_valid or self.pose is None or wall-self.pose_wall>self.c['state']['timeout_s']:
                raise ValueError('State stale or invalid')
            if self.cloud is None or wall-self.cloud_wall>timeout:raise ValueError('LiDAR stale')
            if self.cloud.width==0:raise ValueError('Empty LiDAR does not establish free space')
            cloud=next((m for m,w in reversed(self.cloud_history) if self.history.at(ns(m.header.stamp)) is not None),None)
            if cloud is None:raise ValueError('No acquisition-time pose for LiDAR')
            age=(now-ns(cloud.header.stamp))*1e-9
            if age<-.05 or age>timeout:raise ValueError('LiDAR acquisition age exceeds budget')
            lp=self.history.at(ns(cloud.header.stamp))
            if lp is None:raise ValueError('No acquisition-time pose for LiDAR')
            points=cloud_xyz(cloud)
            points=points[np.isfinite(points).all(axis=1)]
            if not len(points):raise ValueError('Empty LiDAR does not establish free space')
            # Keep nearest returns within each voxel; downsampling must not remove
            # a close obstacle in favour of a far return in the same bin.
            maximum=self.c['safety']['max_points']
            if len(points)>maximum:raise ValueError('LiDAR protection point budget exceeded')
            p,r,velocity=odom_arrays(self.pose)
            world=points@lp[1].T+lp[0] if self.navigation else (points@self.rbl.T+self.tl)@lp[1].T+lp[0]
            if self.navigation:
                scans=[];stamps=[]
                window=self.c['navigation_lidar'].get('scan_window_s',.3)
                for sample,received in self.cloud_history:
                    t=ns(sample.header.stamp);capture=self.history.at(t)
                    if wall-received>window or capture is None or t>ns(cloud.header.stamp) or (now-t)*1e-9>timeout:continue
                    raw=cloud_xyz(sample);raw=raw[np.isfinite(raw).all(axis=1)]
                    if len(raw)>maximum:raise ValueError('LiDAR protection point budget exceeded')
                    scans.append(raw@capture[1].T+capture[0]);stamps.append(t)
                if not scans or (max(stamps)-min(stamps))*1e-9<self.c['navigation_lidar'].get('minimum_sweep_s',.04):
                    raise ValueError('Waiting for complete navigation LiDAR sweep')
                world=np.concatenate(scans)
                # A wall-clock LiDAR trigger with a fixed physics step can
                # produce partial rotations. Merge acquisition poses before
                # decimating; never assume each single frame spans 360 deg.
                order=np.argsort(np.linalg.norm(world-p,axis=1))
                _,indices=np.unique(np.floor(world[order]/.15).astype(np.int64),axis=0,return_index=True)
                world=world[order[indices]]
                if len(world)>maximum:raise ValueError('LiDAR protection point budget exceeded')
                age=max(0.,(now-min(stamps))*1e-9)
            relative=world-p
            desired=np.zeros(3);distance=0.;phase='waypoint';target=p.copy()
            if self.navigation:
                flight=self.flight_phase if wall-self.flight_wall<1. else 'waiting'
                target,phase=self.mission.reference(p,velocity,wall,flight)
                self.goal_index=self.mission.index
                distance=float(np.linalg.norm(target-p))
                if phase=='takeoff':
                    # The observed starting support surface is allowed only
                    # beneath the initial takeoff cylinder. Other returns
                    # still constrain every climb command.
                    base=self.mission.takeoff_target.copy();base[2]-=self.c['mission'].get('takeoff_height_m',1.5)
                    support=(world[:,2]<=base[2]+self.c['mission']['landing_surface_tolerance_m']) & (np.linalg.norm(world[:,:2]-base[:2],axis=1)<2.)
                    world=world[~support];relative=world-p
                if phase in ('waypoint','landing_approach','takeoff'):
                    if self.plan_goal is None or not np.allclose(target,self.plan_goal) or wall-self.last_plan>=self.c['navigation']['replan_period_s']:
                        if self.plan_goal is None or not np.allclose(target,self.plan_goal):self.planner.preferred_direction=None
                        self.aim=self.navigator.plan(p,target,world);self.last_plan=wall;self.plan_goal=target.copy()
                        route=Path(header=header(now))
                        for point in self.navigator.path:
                            pose=PoseStamped(header=header(now));put(pose.pose.position,point);pose.pose.orientation.w=1.;route.poses.append(pose)
                        self.path_pub.publish(route)
                    if self.aim is None:raise ValueError(self.navigator.reason)
                    delta=self.aim-p
                    speed_limit=self.mission.speed_limit(p) if hasattr(self.mission,'speed_limit') else self.c['mission']['speed_mps']
                    unit=delta/max(np.linalg.norm(delta),1e-9)
                    passing=hasattr(self.mission,'pass_through') and self.mission.pass_through()
                    approach=speed_limit if passing else min(speed_limit,self.c['navigation']['position_gain']*distance,
                        math.sqrt(2*self.c['safety']['local_velocity']['braking_acceleration_mps2']*max(0.,distance)))
                    desired=unit*approach
                    damping=self.c['navigation'].get('velocity_damping',.4)
                    # Profile speed is a feed-forward reference. Damping only
                    # lateral motion avoids shrinking its speed by 1/(1+k).
                    desired-=damping*(velocity-unit*(velocity@unit) if hasattr(self.mission,'speed_limit') else velocity)
                    length=np.linalg.norm(desired)
                    desired*=min(1.,speed_limit/max(length,1e-9))
                elif phase=='landing':
                    # Only the expected support surface at the configured
                    # touchdown height may be excluded during final landing.
                    touchdown=self.mission.points[-1]
                    support=(world[:,2]<=touchdown[2]+self.c['mission']['landing_surface_tolerance_m']) & (np.linalg.norm(world[:,:2]-touchdown[:2],axis=1)<2.)
                    if not np.any(support):raise ValueError('landing_surface_not_observed')
                    obstacles=relative[~support]
                    cylinder=(np.linalg.norm(obstacles[:,:2],axis=1)<self.planner.clearance) & (obstacles[:,2]<.5)
                    if np.any(cylinder):raise ValueError('landing_corridor_obstructed')
                    # Continue publishing the explicit landing request while
                    # gateway performs its monitored landing task.
                    command.reason='mission_land';risk.valid=True;risk.reason=command.reason
                    self.pub.publish(command);self.risk_pub.publish(risk);self.publish_mission(now,phase,command.reason,target);return
            else:
                goals=self.c['mission']['waypoints_enu_m']
                if self.goal_index<len(goals):
                    delta=np.asarray(goals[self.goal_index])-p;distance=float(np.linalg.norm(delta))
                    if distance<self.c['mission']['tolerance_m']:self.goal_index+=1
                    else:desired=delta/max(distance,1e-9)*min(self.c['mission']['speed_mps'],distance)
            desire=VelocityCommand(header=header(now),session_id=self.origin.session_id,sequence=self.sequence,
                valid_until=command.valid_until,reason='mission_reference');put(desire.velocity,desired);self.desired_pub.publish(desire)
            obstacles=[];rejected=0
            fusion_fresh=bool(self.tracks and self.tracks_wall is not None and wall-self.tracks_wall<=self.dynamic['max_age_s']
                and 0<=(now-ns(self.tracks.image_stamp))*1e-9<=self.dynamic['max_age_s'])
            if self.dynamic['enabled'] and fusion_fresh:
                obstacles,rejected=from_targets(self.tracks.targets,ns(self.tracks.header.stamp),now,p,self.dynamic)
            elif self.dynamic['enabled']:
                desired*=min(1.,self.dynamic['degraded_speed_mps']/max(np.linalg.norm(desired),1e-9))
            fused_count=len(obstacles)
            lidar_fresh=bool(self.lidar_tracks and self.lidar_tracks_wall is not None
                and wall-self.lidar_tracks_wall<=self.dynamic['max_age_s'])
            if self.dynamic['enabled'] and lidar_fresh:
                # Unclassified fragments need measured motion confirmation.
                # Static and tentative returns are already protected by the
                # unfiltered all-round raw cloud used above.
                moving=[t for t in self.lidar_tracks.targets if t.state=='confirmed']
                extra,bad=from_targets(moving,ns(self.lidar_tracks.header.stamp),now,p,self.dynamic)
                rejected+=bad
                # A semantic observation wins duplicate association. Unknown
                # side/rear returns remain safety obstacles, never class 6.
                obstacles.extend(o for o in extra if not any(np.linalg.norm(o.position-f.position)<1.5 for f in obstacles[:fused_count]))
            if len(obstacles)>self.dynamic['max_targets']:raise ValueError('Combined dynamic obstacle budget exceeded')
            decision=self.planner.command(relative,desired,velocity,dt,age,distance,obstacles,self.dynamic)
            safe=decision.velocity_enu_mps.copy();reason='avoidance' if decision.avoidance_active else 'clear'
            # Enforce the measured lidar angular coverage after candidate
            # smoothing. Unknown side/rear/up/down directions are not free.
            local=safe@r@self.rbl
            if not self.navigation and np.linalg.norm(safe)>.01:
                yaw=abs(math.degrees(math.atan2(local[1],local[0])))
                pitch=abs(math.degrees(math.atan2(local[2],math.hypot(local[0],local[1]))))
                if yaw>self.c['safety']['observed_half_yaw_deg'] or pitch>self.c['safety']['observed_half_pitch_deg']:
                    safe[:]=0.;self.planner.reset();reason='unobserved_direction_stop'
            risk.valid=True;risk.reason=reason
            risk.nearest_obstacle_m=decision.nearest_obstacle_m if decision.nearest_obstacle_m is not None else -1.
            risks=[]
            for obstacle in obstacles:
                tcpa,dcpa=closest_approach(obstacle.position,obstacle.velocity-velocity,self.dynamic['horizon_s'])
                margin=obstacle.radius+self.dynamic['sigma_scale']*(obstacle.position_std+obstacle.velocity_std*tcpa)
                warning=dcpa<margin
                risks.append(dict(track_id=obstacle.track_id,warning=bool(warning),cpa_time_s=tcpa,cpa_distance_m=dcpa,envelope_m=margin,
                    relative_position_enu_m=obstacle.position.tolist(),radius_m=obstacle.radius,
                    position_std_m=obstacle.position_std,velocity_std_mps=obstacle.velocity_std,
                    source='lidar_only' if obstacle.track_id>=1_000_000 else 'vision_lidar'))
                if warning and (not risk.warning or tcpa<risk.cpa_time_s):
                    risk.warning=True;risk.track_id=obstacle.track_id;risk.cpa_time_s=tcpa;risk.cpa_distance_m=dcpa
            if obstacles:
                gap=candidate_clearance([safe],velocity,obstacles,self.planner.avoidance_acceleration,self.dynamic,
                                        self.planner.reaction+self.planner.command_duration)[0]
                if gap<0.:
                    safe[:]=0.;self.planner.reset();reason='dynamic_no_safe_candidate'
                elif decision.dynamic_escape:reason='dynamic_evasive_velocity'
                elif risk.warning:reason='fused_dynamic_avoidance' if fused_count else 'lidar_dynamic_avoidance'
            if wall-self.last_risks>=.1:
                self.last_risks=wall
                self.risks_pub.publish(String(data=json.dumps(dict(schema_version=1,session_id=self.origin.session_id,
                    stamp_ns=now,valid=True,targets=risks,rejected_targets=rejected,
                    fusion_fresh=fusion_fresh,reason=reason))))
                self.health.details=dict(fusion_fresh=fusion_fresh,fused_obstacles=fused_count,
                    lidar_fresh=lidar_fresh,lidar_obstacles=len(obstacles)-fused_count,rejected_targets=rejected)
            if self.navigation and phase not in ('waypoint','landing_approach','takeoff'):
                # Mission completion means hovering, not disabling collision
                # avoidance while still airborne. Never release this exception
                # during arming, landing, failed/stale flight status or on ground.
                airborne_hover=(phase=='complete' and self.flight_phase=='flying' and wall-self.flight_wall<1.)
                if not (airborne_hover and risk.warning and not decision.emergency_stop):
                    safe[:]=0.;reason='mission_'+phase;self.planner.reset()
            put(command.velocity,safe);command.reason=reason;risk.reason=reason;self.health.ok()
        except Exception as exc:
            command.reason=str(exc);risk.reason=str(exc);self.planner.reset()
            self.health.error(exc)
            if wall-self.last_risks>=.1:
                self.last_risks=wall
                self.risks_pub.publish(String(data=json.dumps(dict(schema_version=1,session_id=self.origin.session_id,
                    stamp_ns=now,valid=False,targets=[],fusion_fresh=False,reason=str(exc)))))
        self.pub.publish(command);self.risk_pub.publish(risk)
        self.publish_mission(now,self.mission.phase if self.navigation else 'legacy',command.reason,
            self.plan_goal if self.plan_goal is not None else np.zeros(3))

    def destroy_node(self):
        if hasattr(self.navigator,'close'):self.navigator.close()
        return super().destroy_node()

    def publish_mission(self,now,phase,reason,target):
        value=dict(session_id=self.origin.session_id,stamp_ns=now,phase=phase,waypoint_index=self.goal_index,
            waypoint_count=len(self.c['mission']['waypoints_enu_m']),reason=reason,target_enu_m=np.asarray(target).tolist(),
            planning=self.navigator.reason,flight_mode=self.c['mission'].get('mode','waypoints'),
            profile_speed_reference_mps=getattr(self.mission,'current_speed_mps',None))
        if reason!=self.status_reason:
            self.get_logger().info(f'Mission waypoint {self.goal_index+1}: {phase}, {reason}')
            self.status_reason=reason
        self.mission_pub.publish(String(data=json.dumps(value)))


def main(args=None):
    from evtol_simulation.runtime import TimerResolution
    resolution=TimerResolution()
    c,ros=args_config(args);rclpy.init(args=ros);node=SafetyNode(c)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown();resolution.close()
