"""Reactive 3-D velocity selection; no route, occupancy map or graph search.

Coordinates are world ENU relative to the aircraft, so command smoothing does
not change reference frame when the aircraft turns. All returns inside the
detection sphere participate, independently of semantic class or point count.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from .dynamic import candidate_clearance, DEFAULTS


@dataclass(frozen=True)
class LocalVelocityDecision:
    velocity_enu_mps: np.ndarray
    avoidance_active: bool
    emergency_stop: bool
    nearest_obstacle_m: float | None
    speed_limit_mps: float
    dynamic_escape: bool = False


class LocalVelocityAvoidance:
    def __init__(self, settings, sensor_timeout_s=0.25,
                 command_duration_s=0.1, sweep_retention_s=0.25):
        self.radius = float(settings.get("detection_radius_m", 5.0))
        self.clearance = float(settings.get("clearance_m", 0.8))
        self.acceleration = float(settings.get("max_acceleration_mps2", 2.0))
        self.braking = float(settings.get("braking_acceleration_mps2", 3.0))
        self.avoidance_acceleration = float(settings.get("avoidance_acceleration_mps2", self.braking))
        self.reaction = float(settings.get("reaction_time_s", 0.15))
        self.max_speed = float(settings.get("max_speed_mps", 4.0))
        self.command_duration = float(command_duration_s)
        self.retention = float(sweep_retention_s)
        # Even an obstacle first seen at 5 m must leave a stopping budget.
        self.speed_limit = min(self.max_speed, self._stopping_speed(
            self.radius - self.clearance,
            self.reaction + sensor_timeout_s + self.command_duration + self.retention))
        self.previous = None
        self.preferred_direction = None
        self.static_recovery = False
        self.vertical_penalty=float(settings.get('vertical_penalty',.25))

    def reset(self):
        self.previous = np.zeros(3)
        self.preferred_direction = None
        self.static_recovery = False

    def _stopping_speed(self, distance, delay):
        a = self.braking
        return max(0.0, math.sqrt((a * delay) ** 2 + 2 * a * max(0.0, distance)) - a * delay)

    @staticmethod
    def _limit(vector, limit):
        length = float(np.linalg.norm(vector))
        return vector * min(1.0, limit / max(length, 1e-12))

    def command(self, points_relative_enu_m, desired_enu_mps,
                measured_enu_mps, dt_s, observation_age_s=0.0, goal_distance_m=None,
                obstacles=(),dynamic_options=None,minimum_vertical_velocity_mps=None):
        desired = np.asarray(desired_enu_mps, dtype=float).reshape(3)
        measured = np.asarray(measured_enu_mps, dtype=float).reshape(3)
        dt = float(dt_s)
        if (not np.isfinite(desired).all() or not np.isfinite(measured).all()
                or not math.isfinite(dt) or dt <= 0
                or not math.isfinite(observation_age_s) or observation_age_s < 0):
            raise ValueError("Local avoidance requires finite commands, age and positive dt")
        if self.previous is None:
            self.previous = measured.copy()
        points = np.asarray(points_relative_enu_m, dtype=float).reshape(-1, 3)
        points = points[np.isfinite(points).all(axis=1)]
        lengths = np.linalg.norm(points, axis=1)
        points = points[lengths <= self.radius]
        nearest = float(np.min(np.linalg.norm(points, axis=1))) if len(points) else None
        # The existing accumulated scans lack motion compensation. Inflate the
        # vehicle clearance by their worst-case ego displacement, rather than
        # treating old rays as exact current coordinates.
        margin = self.clearance + max(np.linalg.norm(measured), np.linalg.norm(self.previous)) * (
            self.retention + observation_age_s)
        # Observation age already contributes ego displacement to margin.
        delay = self.reaction + self.command_duration
        squared = np.einsum("ij,ij->i", points, points)

        def free_distances(directions, clearance, maximum):
            # Bound scratch memory and evaluate all rays in NumPy, instead of
            # repeatedly traversing the same cloud in a Python candidate loop.
            result = np.full(len(directions), maximum, dtype=float)
            for start in range(0, len(points), 1024):
                block = points[start:start + 1024]
                along = np.einsum("ni,ji->nj", block, directions, optimize=False)
                perpendicular2 = np.maximum(0., squared[start:start+1024, None] - along*along)
                hits = (along > 0.) & (perpendicular2 < clearance*clearance)
                entry = along - np.sqrt(np.maximum(0., clearance*clearance - perpendicular2))
                entry[~hits] = np.inf
                result = np.minimum(result, np.min(entry, axis=0))
            return np.maximum(0., result)

        def stopping_speeds(distances):
            return np.minimum(self.speed_limit, np.maximum(0.,
                np.sqrt((self.braking * delay)**2 + 2*self.braking*distances) - self.braking*delay))

        speed = min(float(np.linalg.norm(desired)), self.speed_limit)
        forward = desired / max(float(np.linalg.norm(desired)), 1e-12)
        dynamic_options=dynamic_options or DEFAULTS
        threatened=bool(obstacles) and candidate_clearance([desired],measured,obstacles,
            self.avoidance_acceleration,dynamic_options,delay)[0]<0.
        # A side/background return should not cause a full angular search or
        # change a clear forward command. Validate the reachable nominal
        # velocity, its braking distance and actual measured-motion segment.
        nominal=self._limit(desired,self.speed_limit)
        delta=nominal-self.previous
        rate=self.braking if delta@self.previous<0. else self.acceleration
        nominal=self.previous+delta*min(1.,rate*dt/max(np.linalg.norm(delta),1e-12))
        norm=float(np.linalg.norm(nominal))
        free=free_distances([nominal/max(norm,1e-12)],margin,self.radius)[0]
        segment=(measured+nominal)*(.5*self.command_duration);length=float(np.linalg.norm(segment))
        entry=free_distances([segment/max(length,1e-12)],self.clearance,float('inf'))[0]
        nominal_safe=norm<=stopping_speeds(free)+1e-7 and (length<1e-8 or entry>=length)
        if minimum_vertical_velocity_mps is not None:nominal_safe &= nominal[2]>=minimum_vertical_velocity_mps-1e-9
        if obstacles:
            nominal_safe &= candidate_clearance([nominal],measured,obstacles,self.avoidance_acceleration,dynamic_options,delay)[0]>=0.
        if nominal_safe and not threatened:
            self.previous=nominal.copy();self.preferred_direction=None
            return LocalVelocityDecision(nominal,False,False,nearest,self.speed_limit,False)
        if speed<1e-8 and threatened:
            # Hover is not safe when another aircraft is moving into us.
            nearest_threat=min(obstacles,key=lambda o:np.linalg.norm(o.position))
            forward=-nearest_threat.position/max(np.linalg.norm(nearest_threat.position),1e-12)
            if np.linalg.norm(forward)<.1:forward=np.array([1.,0.,0.])
            speed=self.speed_limit
        if speed < 1e-8:
            candidates, frees = np.zeros((1, 3)), np.array([self.radius])
        else:
            axis = np.array([0., 0., 1.]) if abs(forward[2]) < .9 else np.array([0., 1., 0.])
            side = np.cross(axis, forward)
            side /= np.linalg.norm(side)
            up = np.cross(forward, side)
            directions = [forward]
            if len(points) or obstacles:
                for angle in (25, 45, 65, 90, 120):
                    radians = math.radians(angle)
                    for azimuth in range(0, 360, 45):
                        phi = math.radians(azimuth)
                        directions.append(forward * math.cos(radians) + math.sin(radians) * (
                            side * math.cos(phi) + up * math.sin(phi)))
                directions.append(-forward)
            directions = np.asarray(directions)
            frees = free_distances(directions, margin, self.radius)
            candidates = directions * np.minimum(speed, stopping_speeds(frees))[:, None]
        candidates = np.vstack((candidates, np.zeros(3)))
        frees = np.append(frees, self.radius)
        delta = candidates - self.previous
        rates = np.where(delta @ self.previous < 0., self.braking, self.acceleration)
        if len(points) and frees[0] < self.radius and (goal_distance_m is None or frees[0] < goal_distance_m):
            rates = np.maximum(rates, self.avoidance_acceleration)
        commands = self.previous + delta * np.minimum(1., rates*dt / np.maximum(
            np.linalg.norm(delta, axis=1), 1e-12))[:, None]
        # Repeated candidates (especially braking in dense scenes) must not
        # cause repeated full-cloud checks.
        unique, inverse = np.unique(commands, axis=0, return_inverse=True)
        norms = np.linalg.norm(unique, axis=1)
        directions = unique / np.maximum(norms, 1e-12)[:, None]
        free = free_distances(directions, margin, self.radius)
        valid_unique = norms <= stopping_speeds(free) + 1e-7
        segments = (measured + unique)*.5*self.command_duration
        lengths = np.linalg.norm(segments, axis=1)
        segment_directions = segments / np.maximum(lengths, 1e-12)[:, None]
        # A sphere-entry distance before the end of the segment means the
        # actual smoothed command would sweep through a return.
        entries = free_distances(segment_directions, self.clearance, float("inf"))
        valid_unique &= (lengths < 1e-8) | (entries >= lengths)
        if minimum_vertical_velocity_mps is not None:
            valid_unique &= unique[:,2] >= float(minimum_vertical_velocity_mps)-1e-9
        valid = valid_unique[inverse]
        if obstacles:
            dynamic_options=dynamic_options or DEFAULTS
            gaps=candidate_clearance(unique,measured,obstacles,self.avoidance_acceleration,
                                     dynamic_options,self.reaction+self.command_duration)
            valid &= gaps[inverse]>=0.
        future_norms = np.linalg.norm(candidates, axis=1)
        scores = candidates @ forward + .18*future_norms
        scores -= .12*np.linalg.norm(candidates-self.previous, axis=1)
        scores -= self.vertical_penalty*abs(candidates[:,2]-speed*forward[2])
        obstruction = ((np.ones(len(scores), dtype=bool) if goal_distance_m is None else frees < goal_distance_m)
                       & (future_norms > 1e-8))
        scores -= np.where(obstruction, .8*speed*(1.-frees/self.radius), 0.)
        if self.preferred_direction is not None:
            scores += .25*(candidates @ self.preferred_direction) / np.maximum(future_norms, 1e-12)
        scores[~valid] = -np.inf
        # A late raw return can put the aircraft inside the inflated planning
        # margin without physical contact. Hover then wins the progress score,
        # while A* cannot leave an occupied start voxel. Select a checked
        # departure that increases point clearance, with hysteresis until the
        # start is outside the route inflation. No invalid candidate is enabled.
        if nearest is None or nearest>=margin+.35:self.static_recovery=False
        elif nearest<margin:self.static_recovery=True
        if self.static_recovery and len(points) and speed>1e-8:
            close=points[np.linalg.norm(points,axis=1)<margin+.7]
            if len(close):
                future=commands*.5
                separations=np.min(np.linalg.norm(close[None,:,:]-future[:,None,:],axis=2),axis=1)
                improving=valid & (separations>nearest+.005) & (np.linalg.norm(commands,axis=1)>.01)
                if np.any(improving):
                    scores=np.where(improving,separations+.02*(commands@forward),-np.inf)
        selected = int(np.argmax(scores))
        if valid[selected]:
            selected = int(np.flatnonzero(valid & (scores >= scores[selected]-1e-9))[0])
        best = commands[selected] if valid[selected] else None
        dynamic_escape=False
        if best is None and threatened:
            # A nominal command ramp can make every immediate velocity unsafe
            # under a constant-target rollout. Test bounded evasive targets
            # against actual measured velocity and avoidance acceleration.
            # Physical acceleration/delay remain in CPA; never teleport or
            # classify the other aircraft as stationary to manufacture a pass.
            # Mission approach/takeoff speeds can be much lower than the
            # physically available escape speed. Keeping that low norm makes
            # an expanding uncertain obstacle envelope a permanent deadlock.
            # Test both the original targets and bounded faster targets; the
            # rollout below still enforces actual acceleration, response delay,
            # raw-cloud stopping distance and the takeoff vertical constraint.
            norms=np.linalg.norm(candidates,axis=1)
            faster=candidates[norms>1e-8]/norms[norms>1e-8,None]*self.speed_limit
            escape=np.vstack((candidates,faster))
            lengths=np.linalg.norm(escape,axis=1)
            directions=escape/np.maximum(lengths,1e-12)[:,None]
            free=free_distances(directions,margin,self.radius)
            allowed=lengths<=stopping_speeds(free)+1e-7
            segments=(measured+escape)*.5*self.command_duration
            segment_lengths=np.linalg.norm(segments,axis=1)
            entries=free_distances(segments/np.maximum(segment_lengths,1e-12)[:,None],self.clearance,float('inf'))
            allowed &= (segment_lengths<1e-8)|(entries>=segment_lengths)
            if minimum_vertical_velocity_mps is not None:
                allowed &= escape[:,2] >= float(minimum_vertical_velocity_mps)-1e-9
            gaps=candidate_clearance(escape,measured,obstacles,self.avoidance_acceleration,dynamic_options,delay)
            allowed &= gaps>=0.
            if np.any(allowed):
                # Prefer larger predicted clearance over mission progress in
                # this emergency override of the nominal command slew only.
                score=np.where(allowed,gaps+.05*(escape@forward),-np.inf)
                best=escape[int(np.argmax(score))];dynamic_escape=True
        emergency = best is None
        if emergency:
            # Newly appearing/too-close obstacles can invalidate every smooth
            # command. Safety takes precedence over the acceleration ramp.
            best = np.zeros(3)
        active = emergency or ((len(points)>0 or bool(obstacles)) and np.linalg.norm(best - self._limit(desired, self.speed_limit)) > .05)
        if active and np.linalg.norm(best) > .1:
            self.preferred_direction = best / np.linalg.norm(best)
        elif not len(points):
            self.preferred_direction = None
        self.previous = best.copy()
        return LocalVelocityDecision(best, bool(active), emergency, nearest, self.speed_limit,dynamic_escape)
