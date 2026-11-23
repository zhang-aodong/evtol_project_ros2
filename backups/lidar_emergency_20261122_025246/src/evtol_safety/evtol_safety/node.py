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
from evtol_targeting.common import args_config, ns, header, stamp, xyz, put, odom_arrays, LATCHED, LATEST, Health
from evtol_targeting.geometry import PoseHistory, cloud_xyz
from .velocity import LocalVelocityAvoidance
from .navigation import WaypointMission,VoxelNavigator,HorizontalFirstNavigator
from .route_following import SampledLeg,cruise_velocity
from .profile import make_mission
from .dynamic import DEFAULTS as DYNAMIC_DEFAULTS,from_targets,candidate_clearance
from .async_navigation import AsyncNavigator
from .corridor import local_points,near_obstacles,route_aim
from .layers import emergency_points,fused_route_points,descent_limit,hold_velocity
from .raw_emergency import RawLidarEmergency


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
        nav=c.get('navigation',{})
        synchronous=HorizontalFirstNavigator if nav.get('strategy')=='horizontal_first_astar' else VoxelNavigator
        self.navigator=(AsyncNavigator if nav.get('async_planning',True) else synchronous)(nav)
        self.reference=SampledLeg(nav)
        self.dynamic={**DYNAMIC_DEFAULTS,**c['safety'].get('dynamic',{})};self.last_risks=0.
        self.tracks_wall=None
        self.lidar_tracks=None;self.lidar_tracks_wall=None
        self.flight_phase='waiting' if self.navigation else 'flying';self.flight_wall=0.
        self.last_plan=0.;self.aim=None;self.plan_goal=None;self.status_reason='waiting'
        self.local_goal=None;self.plan_count=0
        self.history=PoseHistory(c['fusion']['pose_max_gap_s']);self.cloud=None;self.cloud_wall=None
        self.cloud_history=deque(maxlen=8)
        self.scan_cache={};self.navigation_ready_since=None;self.safety_data={}
        if getattr(self.mission,'ground_hold',False):
            self.get_logger().warning('Ground endpoint with land_at_last=false: hold above touchdown at landing_handoff_m; enable land_at_last for touchdown')
        self.last_cloud_ns=0;self.tracks=None;self.sequence=0;self.goal_index=0;self.previous_wall=time.perf_counter()
        raw_options=c['safety'].get('raw_emergency',{})
        local_options={**c['safety']['local_velocity'],'detection_radius_m':raw_options.get('radius_m',2.)}
        self.planner=LocalVelocityAvoidance(local_options,sensor_timeout_s=c['safety']['cloud_timeout_s'],
            command_duration_s=c['control']['duration_s'],sweep_retention_s=0.)
        self.raw_guard=RawLidarEmergency(c['safety'].get('raw_emergency'))
        if c['safety'].get('speed_limit_notice'):self.get_logger().warning(c['safety']['speed_limit_notice'])
        lc=c['extrinsics']['lidar'];self.tl,self.ql=extrinsic(lc['xyz_frd_m'],lc['rpy_deg'])
        self.rbl=quaternion_matrix(self.ql)
        self.pub=self.create_publisher(VelocityCommand,self.root+'/control/safe_velocity',qos_profile_sensor_data)
        self.risk_pub=self.create_publisher(Risk,self.root+'/safety/risk',qos_profile_sensor_data)
        self.risks_pub=self.create_publisher(String,self.root+'/safety/risks',10)
        self.desired_pub=self.create_publisher(VelocityCommand,self.root+'/control/desired_velocity',qos_profile_sensor_data)
        self.mission_pub=self.create_publisher(String,self.root+'/mission/status',LATCHED)
        self.path_pub=self.create_publisher(Path,self.root+'/navigation/path',qos_profile_sensor_data)
        self.reference_pub=self.create_publisher(Path,self.root+'/navigation/reference_path',LATCHED)
        self.create_subscription(String,self.root+'/control/flight_status',self.receive_flight,LATCHED)
        self.create_subscription(ReferenceOrigin,'/simulation/reference_origin',self.receive_origin,LATCHED)
        self.create_subscription(Odometry,self.root+'/state/odometry',self.receive_pose,LATEST)
        self.create_subscription(StateStatus,self.root+'/state/status',self.receive_status,10)
        self.create_subscription(PointCloud2,self.root+('/navigation/lidar/points' if self.navigation else '/lidar/points'),self.receive_cloud,LATEST)
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
            self.scan_cache.clear();self.navigation_ready_since=None;self.safety_data={}
            self.planner.reset();self.goal_index=0;self.last_cloud_ns=0
            self.raw_guard.reset();self.flight_phase='waiting';self.flight_wall=0.
            self.mission=make_mission(self.c['mission']);self.aim=None;self.last_plan=0.
            self.reference.reset();self.plan_goal=None
            self.local_goal=None;self.plan_count=0
            self.state_valid=False
            if hasattr(self.navigator,'close'):self.navigator.close()
        self.origin=msg

    def receive_flight(self,msg):
        try:
            value=json.loads(msg.data)
            if self.origin and value['session_id']==self.origin.session_id:
                self.flight_phase=value['phase'];self.flight_wall=time.perf_counter()
                if self.flight_phase!='flying':self.raw_guard.reset()
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
        self.safety_data=dict(pose_age_s=wall-self.pose_wall if self.pose_wall else None,
            lidar_receive_age_s=wall-self.cloud_wall if self.cloud_wall else None,
            scan_frames=0,scan_span_s=0.,planning_hz=1./self.c['navigation']['replan_period_s'] if self.navigation else 0.,
            raw_emergency_radius_m=self.planner.radius,fused_avoidance_radius_m=self.c['safety'].get('fused_avoidance_radius_m',4.),
            achievable_speed_limit_mps=self.planner.speed_limit)
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
            self.safety_data['lidar_acquisition_age_s']=age
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
            raw_snapshot=world;raw_age=max(0.,age);estimate_raw_motion=True
            if self.navigation:
                scans=[];stamps=[]
                window=self.c['navigation_lidar'].get('scan_window_s',.3)
                minimum=self.c['navigation_lidar'].get('minimum_sweep_s',.04)
                live={ns(m.header.stamp) for m,w in self.cloud_history}
                self.scan_cache={t:value for t,value in self.scan_cache.items() if t in live}
                # Use the newest pose-aligned frames needed for the configured
                # sweep span. Merging the entire .3 s buffer repeatedly at
                # 50 Hz adds latency and old moving-object ghosts.
                for sample,received in reversed(self.cloud_history):
                    t=ns(sample.header.stamp);capture=self.history.at(t)
                    if wall-received>window or capture is None or t>ns(cloud.header.stamp) or (now-t)*1e-9>timeout:continue
                    cached=self.scan_cache.get(t)
                    if cached is None or cached[0] is not sample:
                        raw=cloud_xyz(sample);raw=raw[np.isfinite(raw).all(axis=1)]
                        if len(raw)>maximum:raise ValueError('LiDAR protection point budget exceeded')
                        cached=(sample,raw@capture[1].T+capture[0]);self.scan_cache[t]=cached
                    scans.append(cached[1]);stamps.append(t)
                    if (stamps[0]-t)*1e-9>=minimum:break
                span=(max(stamps)-min(stamps))*1e-9 if stamps else 0.
                self.safety_data.update(scan_frames=len(stamps),scan_span_s=span,required_sweep_s=minimum)
                if not scans or span<minimum:
                    raise ValueError('Waiting for complete navigation LiDAR sweep')
                world=np.concatenate(scans)
                # Warning nodes retain the full scan. Only the near planning
                # neighbourhood needs voxel sorting in this control loop.
                world=local_points(world,p,max(self.planner.radius,self.c['navigation']['planning_radius_m']))
                # The navigation producer verifies the shared-memory full-scan
                # flag before publishing. Default minimum_sweep_s=0 consumes a
                # complete snapshot without waiting for a second pose bracket.
                # Explicit nonzero windows retain motion-compensated merging.
                order=np.argsort(np.linalg.norm(world-p,axis=1))
                _,indices=np.unique(np.floor(world[order]/.15).astype(np.int64),axis=0,return_index=True)
                world=world[order[indices]]
                if len(world)>maximum:raise ValueError('LiDAR protection point budget exceeded')
                age=max(0.,(now-min(stamps))*1e-9)
                if minimum>0.:
                    # Partial sweeps need the retained returns for clearance.
                    # Their mixed timestamps cannot give a frame-to-frame rate.
                    raw_snapshot=world;raw_age=age;estimate_raw_motion=False
            raw_armed=self.flight_phase=='flying' and wall-self.flight_wall<1.
            raw_obstacles,raw_world=self.raw_guard.update(raw_snapshot,p,lp[0],ns(cloud.header.stamp),raw_age,raw_armed,
                self.c['mission'].get('landing_ground_z_m',0.),self.c['mission']['landing_surface_tolerance_m'],self.planner.clearance,
                estimate_motion=estimate_raw_motion)
            self.safety_data['lidar_emergency']=dict(self.raw_guard.diagnostics)
            self.safety_data['raw_emergency_radius_m']=self.raw_guard.o['radius_m']
            # Raw scans are an emergency-only layer; unclassified returns at
            # 2+ metres never enter normal route search or velocity selection.
            world=emergency_points(world,p,self.planner.radius)
            relative=world-p
            desired=np.zeros(3);distance=0.;phase='waypoint';target=p.copy()
            if self.navigation:
                flight=self.flight_phase if wall-self.flight_wall<1. else 'waiting'
                target,phase=self.mission.reference(p,velocity,wall,flight)
                self.goal_index=self.mission.index
                distance=float(np.linalg.norm(target-p))
                if phase=='takeoff':
                    # During a non-descending takeoff, the observed support
                    # below the starting floor cannot intersect an executable
                    # climb/side-step. A fixed XY cylinder turned its edge into
                    # an artificial wall after a valid evasive side-step, leaving
                    # the aircraft unable to climb again near the ground.
                    base=self.mission.takeoff_target.copy();base[2]-=self.c['mission'].get('takeoff_height_m',1.5)
                    support=world[:,2]<=base[2]+self.c['mission']['landing_surface_tolerance_m']
                    world=world[~support];relative=world-p
                if phase in ('waypoint','landing_approach','takeoff'):
                    if self.plan_goal is None or not np.allclose(target,self.plan_goal):
                        self.planner.preferred_direction=None;self.navigator.path=[];self.local_goal=None;self.aim=None
                        self.plan_goal=target.copy()
                    sampled=self.c['mission'].get('mode','waypoints')=='waypoints' and phase=='waypoint'
                    if wall-self.last_plan>=self.c['navigation']['replan_period_s']:
                        route_world=raw_world if raw_armed and self.raw_guard.o['enabled'] else world
                        route_points=local_points(route_world,p,self.c['navigation']['planning_radius_m'])
                        if self.dynamic['enabled'] and self.tracks is not None and self.tracks_wall is not None and wall-self.tracks_wall<=self.dynamic['max_age_s']:
                            predicted,_=from_targets(self.tracks.targets,ns(self.tracks.header.stamp),now,p,self.dynamic)
                            predicted=near_obstacles(predicted,velocity,self.c['safety'].get('fused_avoidance_radius_m',4.),self.dynamic['horizon_s'])
                            cloud=fused_route_points(predicted,p,self.dynamic['horizon_s'])
                            if len(cloud):route_points=np.vstack((route_points,cloud))
                        self.local_goal=self.reference.reference(p,target,(phase,self.mission.index),route_points) if sampled else target.copy()
                        self.aim=self.navigator.plan(p,self.local_goal,route_points)
                        self.plan_count+=1
                        self.last_plan=wall
                        route=Path(header=header(now))
                        for point in self.navigator.path:
                            pose=PoseStamped(header=header(now));put(pose.pose.position,point);pose.pose.orientation.w=1.;route.poses.append(pose)
                        self.path_pub.publish(route)
                        reference=Path(header=header(now))
                        for point in self.reference.samples if sampled else [p,target]:
                            pose=PoseStamped(header=header(now));put(pose.pose.position,point);pose.pose.orientation.w=1.;reference.poses.append(pose)
                        self.reference_pub.publish(reference)
                    else:
                        self.aim=route_aim(p,self.navigator.path,self.c['navigation']['lookahead_m'])
                    local_goal=self.local_goal if self.local_goal is not None else target
                    self.safety_data.update(reference_goal_enu_m=local_goal.tolist(),reference_spacing_m=self.reference.spacing,
                        reference_samples=len(self.reference.samples) if sampled else 0,planning_updates=self.plan_count,
                        planning_hz=1./self.c['navigation']['replan_period_s'],raw_emergency_radius_m=self.raw_guard.o['radius_m'],
                        fused_avoidance_radius_m=self.c['safety'].get('fused_avoidance_radius_m',4.),raw_nearby_returns=len(relative),
                        ground_endpoint_hold=bool(getattr(self.mission,'ground_hold',False)),
                        dynamic_horizon_s=self.dynamic['horizon_s'])
                    if self.aim is None:
                        if raw_obstacles or (isinstance(self.navigator,AsyncNavigator) and phase=='waypoint' and self.navigator.reason!='goal_obstructed'):
                            # Waiting for an expensive detour is not itself a
                            # reason to reset the velocity ramp. Use the local
                            # sample heading as a preference; the unchanged
                            # all-round braking/dynamic layer chooses the actual
                            # executable velocity or stops if none is safe.
                            delta=local_goal-p
                            self.aim=p+delta*min(1.,self.c['navigation']['lookahead_m']/max(np.linalg.norm(delta),1e-9))
                            self.safety_data['reference_fallback']='local_velocity_while_planning'
                        else:raise ValueError(self.navigator.reason)
                    speed_limit=self.mission.speed_limit(p) if hasattr(self.mission,'speed_limit') else self.c['mission']['speed_mps']
                    passing=hasattr(self.mission,'pass_through') and self.mission.pass_through()
                    desired=cruise_velocity(p,self.aim,velocity,speed_limit,distance,not passing,
                        self.c['navigation'],self.c['safety']['local_velocity']['braking_acceleration_mps2'])
                elif phase=='complete' and flight=='flying':
                    # Zero velocity alone slowly loses height with the default
                    # AirSim velocity loop. Keep the completed mission's actual
                    # endpoint, including an explicitly disabled ground landing.
                    target=self.mission.points[-1].copy()
                    if getattr(self.mission,'ground_hold',False):target[2]+=self.c['mission'].get('landing_handoff_m',1.2)
                    desired=hold_velocity(p,velocity,target,self.c['navigation'])
                elif phase=='landing':
                    self.raw_guard.reset()
                    self.safety_data['lidar_emergency']=dict(self.raw_guard.diagnostics,reason='landing_handoff')
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
            # LiDAR-only tracks remain available for diagnostics. Only fused
            # targets influence normal avoidance; raw returns protect 2 m.
            if len(obstacles)>self.dynamic['max_targets']:raise ValueError('Combined dynamic obstacle budget exceeded')
            before=len(obstacles)
            obstacles=near_obstacles(obstacles,velocity,self.c['safety'].get('fused_avoidance_radius_m',4.),self.dynamic['horizon_s'])
            self.safety_data.update(dynamic_targets_total=before,dynamic_targets_local=len(obstacles))
            fused_count=len(obstacles)
            if raw_armed and self.raw_guard.o['enabled']:
                relative=raw_world-p
                desired=self.raw_guard.preference(desired,raw_obstacles,self.planner.clearance,self.planner.speed_limit)
                obstacles=obstacles+raw_obstacles
            decision=self.planner.command(relative,desired,velocity,dt,age,distance,obstacles,self.dynamic,
                minimum_vertical_velocity_mps=descent_limit(p,target,self.reference.anchor,phase,
                    self.c['mission'].get('landing_ground_z_m',0.),self.planner.clearance,self.planner.braking) if self.navigation else None,
                best_effort_escape=bool(raw_obstacles))
            safe=decision.velocity_enu_mps.copy();reason='avoidance' if decision.avoidance_active else 'clear'
            self.safety_data['raw_emergency_active']=bool(raw_obstacles)
            self.safety_data['local_velocity_avoidance']=bool(decision.avoidance_active)
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
                    source='raw_lidar' if obstacle.track_id<0 else ('lidar_only' if obstacle.track_id>=1_000_000 else 'vision_lidar')))
                if warning and (not risk.warning or tcpa<risk.cpa_time_s):
                    risk.warning=True;risk.track_id=obstacle.track_id;risk.cpa_time_s=tcpa;risk.cpa_distance_m=dcpa
            if obstacles:
                gap=candidate_clearance([safe],velocity,obstacles,self.planner.avoidance_acceleration,self.dynamic,
                                        self.planner.reaction+self.planner.command_duration)[0]
                if gap<0. and not decision.best_effort_escape:
                    safe[:]=0.;self.planner.reset();reason='dynamic_no_safe_candidate'
                elif decision.dynamic_escape:reason='dynamic_evasive_velocity'
                elif risk.warning:reason='fused_dynamic_avoidance' if fused_count else 'lidar_dynamic_avoidance'
            if raw_obstacles:
                if decision.best_effort_escape:reason='lidar_emergency_best_effort'
                elif decision.emergency_stop or reason=='dynamic_no_safe_candidate':reason='lidar_emergency_stop'
                else:reason='lidar_emergency_escape' if decision.avoidance_active or np.linalg.norm(safe)>.01 else 'lidar_emergency_brake'
                risk.warning=True
                raw_nearest=self.raw_guard.diagnostics['nearest_m']
                risk.nearest_obstacle_m=raw_nearest if risk.nearest_obstacle_m<0 else min(risk.nearest_obstacle_m,raw_nearest)
                self.safety_data['lidar_emergency'].update(action=reason,velocity_enu_mps=safe.tolist())
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
                if airborne_hover and np.linalg.norm(desired)>.005 and not decision.emergency_stop:
                    if not risk.warning and not decision.avoidance_active:reason='mission_hold'
                elif not (airborne_hover and (raw_obstacles or ((risk.warning or decision.avoidance_active) and not decision.emergency_stop))):
                    safe[:]=0.;reason='mission_'+phase;self.planner.reset()
            put(command.velocity,safe);command.reason=reason;risk.reason=reason;self.health.ok()
        except Exception as exc:
            self.navigation_ready_since=None
            self.raw_guard.reset()
            self.safety_data['lidar_emergency']=dict(self.raw_guard.diagnostics,reason='safety_hold',action=str(exc))
            risk.valid=False
            command.reason=str(exc);risk.reason=str(exc);self.planner.reset()
            self.health.error(exc)
            if wall-self.last_risks>=.1:
                self.last_risks=wall
                self.risks_pub.publish(String(data=json.dumps(dict(schema_version=1,session_id=self.origin.session_id,
                    stamp_ns=now,valid=False,targets=[],fusion_fresh=False,reason=str(exc)))))
        compute=time.perf_counter()-wall
        if risk.valid and self.navigation_ready_since is None:self.navigation_ready_since=wall
        self.safety_data['compute_ms']=round(compute*1000,3)
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
            safety_data=self.safety_data,navigation_ready_s=max(0.,time.perf_counter()-self.navigation_ready_since) if self.navigation_ready_since is not None else 0.,
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
