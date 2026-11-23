"""Near-field LiDAR protection independent of visual detections."""

import math

import numpy as np

from .dynamic import Obstacle


DEFAULTS = dict(
    enabled=True,
    radius_m=2.0,
    escape_speed_mps=1.5,
    self_exclusion_m=0.25,
    max_range_rate_mps=12.0,
    range_history_s=0.25,
    release_margin_m=0.4,
    release_hold_s=0.5,
)


def options(value=None):
    if value is not None and not isinstance(value, dict):
        raise ValueError('raw_emergency must be a configuration object')
    settings = {**DEFAULTS, **(value or {})}
    if set(settings) != set(DEFAULTS):
        raise ValueError('Unknown raw_emergency options')
    if type(settings['enabled']) is not bool:
        raise ValueError('raw_emergency.enabled must be boolean')
    bounds = dict(
        radius_m=(1., 10.),
        escape_speed_mps=(.1, 5.),
        self_exclusion_m=(.05, .4),
        max_range_rate_mps=(1., 30.),
        range_history_s=(.05, .5),
        release_margin_m=(.1, 2.),
        release_hold_s=(.1, 2.),
    )
    for name, (lower, upper) in bounds.items():
        number = settings[name]
        if (isinstance(number, bool) or not isinstance(number, (int, float))
                or not math.isfinite(number) or not lower <= number <= upper):
            raise ValueError(f'raw_emergency.{name} must be {lower:g}..{upper:g}')
    return settings


class RawLidarEmergency:
    def __init__(self, value=None):
        self.o = options(value)
        self.reset()

    def reset(self):
        self.stamp = 0
        self.cells = {}
        self.rates = {}
        self.active = False
        self.clear_since = None
        self.diagnostics = dict(armed=False, active=False, reason='waiting_for_flight')

    def update(self, world, position, capture_position, stamp_ns, age_s, armed,
               ground_z=None, ground_tolerance=.15, clearance=.8, *, estimate_motion=True):
        if not self.o['enabled'] or not armed:
            self.reset()
            if not self.o['enabled']:
                self.diagnostics['reason'] = 'disabled'
            return [], np.empty((0, 3))

        position = np.asarray(position, float)
        capture = np.asarray(capture_position, float)
        points = np.asarray(world, float).reshape(-1, 3)
        points = points[np.isfinite(points).all(axis=1)]
        relative = points - position
        distance = np.linalg.norm(relative, axis=1)
        limit = self.o['radius_m'] + (self.o['release_margin_m'] if self.active else 0.)
        valid = (distance <= limit) & (distance >= self.o['self_exclusion_m'])
        if ground_z is not None:
            support = (points[:, 2] <= ground_z + ground_tolerance) & (points[:, 2] < position[2] - .3)
            valid &= ~support
        points, relative, distance = points[valid], relative[valid], distance[valid]

        if stamp_ns < self.stamp:
            self.reset()
            return self.update(world,position,capture_position,stamp_ns,age_s,armed,
                ground_z,ground_tolerance,clearance,estimate_motion=estimate_motion)
        if len(points):
            self.active = True
            self.clear_since = None
        if not len(points):
            if self.active:
                if self.clear_since is None:self.clear_since=stamp_ns
                if (stamp_ns-self.clear_since)*1e-9>=self.o['release_hold_s']:
                    self.active=False;self.clear_since=None
            if stamp_ns > self.stamp:
                self.cells.clear()
                self.rates.clear()
                self.stamp = int(stamp_ns)
            self.diagnostics = dict(armed=True, active=self.active, reason='release_hold' if self.active else 'clear', nearest_m=None,
                                    points=0, cells=0, max_closing_mps=0.)
            return [], points

        # Retain the nearest return in each of 72 angular cells for prediction.
        # The velocity planner still checks all measured points for clearance.
        azimuth = np.arctan2(relative[:, 1], relative[:, 0])
        elevation = np.arcsin(np.clip(relative[:, 2] / distance, -1., 1.))
        yaw = np.floor((azimuth + np.pi) / (2 * np.pi) * 18).astype(int) % 18
        pitch = np.minimum(3, np.floor((elevation + np.pi / 2) / np.pi * 4).astype(int))
        keys = yaw + 18 * pitch
        order = np.argsort(distance)
        _, indices = np.unique(keys[order], return_index=True)
        current = {int(keys[i]): points[i] for i in order[indices]}

        if stamp_ns > self.stamp:
            rates = {}
            dt = (stamp_ns - self.stamp) * 1e-9
            if estimate_motion and .005 <= dt <= self.o['range_history_s']:
                for key, point in current.items():
                    previous = self.cells.get(key)
                    if previous is None:
                        continue
                    radial, before = point - capture, previous - capture
                    length, before_length = np.linalg.norm(radial), np.linalg.norm(before)
                    if min(length, before_length) > 1e-6 and radial @ before > .94 * length * before_length:
                        # Compare both ranges from the same origin to remove ego translation.
                        rate = (length - before_length) / dt
                        rates[key] = float(np.clip(rate, -self.o['max_range_rate_mps'], 0.))
            self.cells, self.rates, self.stamp = current, rates, int(stamp_ns)

        obstacles = []
        for key, point in current.items():
            relative_point = point - position
            direction = relative_point / max(np.linalg.norm(relative_point), 1e-9)
            radial_velocity = direction * self.rates.get(key, 0.)
            predicted_point = relative_point + radial_velocity * max(0., age_s)
            obstacles.append(Obstacle(-key - 1, predicted_point, radial_velocity, float(clearance)))
        self.diagnostics = dict(
            armed=True, active=True, reason='nearby_lidar_returns', nearest_m=float(distance.min()),
            points=len(points), cells=len(obstacles),
            max_closing_mps=max((-self.rates.get(key, 0.) for key in current), default=0.),
        )
        return obstacles, points

    def preference(self, desired, obstacles, clearance, speed_limit):
        if not obstacles:
            return np.asarray(desired, float)

        vectors = np.asarray([obstacle.position for obstacle in obstacles])
        distance = np.linalg.norm(vectors, axis=1)
        directions = vectors / np.maximum(distance[:, None], 1e-9)
        closing = np.array([max(0., -obs.velocity @ direction)
                            for obs, direction in zip(obstacles, directions)])
        # Use the most urgent direction to seed the search. The planner checks
        # every remaining direction, including those opposed to this one.
        urgency = (self.o['radius_m'] - distance) / max(self.o['radius_m'] - clearance, .1) + closing
        worst = int(np.argmax(urgency))
        away = -directions[worst]
        escape = min(speed_limit, self.o['escape_speed_mps'])
        reference = np.asarray(desired, float).copy()
        outward = min(escape, max(.15, escape * np.clip(urgency[worst], 0., 1.)))
        reference += away * max(0., outward - reference @ away)
        return reference * min(1., speed_limit / max(np.linalg.norm(reference), 1e-9))
