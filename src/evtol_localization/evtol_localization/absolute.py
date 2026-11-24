"""Causal ENU/FLU ESKF. All event times are simulator header nanoseconds."""
from bisect import bisect_left, bisect_right
from collections import Counter
import copy
from dataclasses import dataclass, field
import math

import numpy as np

from evtol_simulation.coordinates import quaternion_matrix
from evtol_simulation.gnss_model import enu_to_ecef_matrix, geodetic_to_ecef


I3 = np.eye(3)
I15 = np.eye(15)
GRAVITY = 9.80665
G_ENU = np.array([0., 0., -GRAVITY])
# Error-state order: ENU position, ENU velocity, right/body attitude error,
# accelerometer bias, gyroscope bias. Both biases use FLU.
POS, VEL, ATT, BA, BG = [slice(i, i + 3) for i in range(0, 15, 3)]


def vector3(values):
    result = np.asarray(values, dtype=float)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError('expected finite three-vector')
    return result


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def exp_rotation(v):
    angle = np.linalg.norm(v)
    k = skew(v)
    a = np.sinc(angle / np.pi)
    b = 0.5 * np.sinc(angle / (2 * np.pi)) ** 2
    return I3 + a * k + b * (k @ k)


def geodetic(values):
    lat, lon, h = vector3(values)
    if abs(lat) > 90 or abs(lon) > 180:
        raise ValueError('invalid geodetic coordinates')
    return lat, lon, h


class SessionMap:
    def __init__(self, home_lla, source_origin):
        lat, lon, h = geodetic(home_lla)
        self.ecef = geodetic_to_ecef(lat, lon, h)
        self.to_map = enu_to_ecef_matrix(lat, lon).T
        self.origin = vector3(source_origin)

    def fix(self, lla, covariance):
        lat, lon, h = geodetic(lla)
        p = self.to_map @ (geodetic_to_ecef(lat, lon, h) - self.ecef) - self.origin
        rotation = self.to_map @ enu_to_ecef_matrix(lat, lon)
        return p, rotation @ covariance @ rotation.T

    def station(self, source_position):
        # Nonfinite station coordinates remain nonfinite; the per-link gate
        # records that rejection while retaining other links in the same scan.
        return np.asarray(source_position, dtype=float) - self.origin


@dataclass
class Settings:
    history_s: float = .5
    max_imu_gap_s: float = .25
    accel_noise_density: float = .08
    gyro_noise_density: float = .005
    accel_bias_walk: float = .003
    gyro_bias_walk: float = .0002
    gnss_sigma_floor_m: float = .20
    gnss_max_sigma_m: float = 100.
    gnss_unknown_sigma_m: float = 5.
    gnss_nis_max: float = 16.27
    range_sigma_floor_m: float = .20
    range_nis_max: float = 10.83
    min_rsrp_dbm: float = -120.
    gnss_lever_flu_m: tuple = (0., 0., 0.)
    radio_lever_flu_m: tuple = (0., 0., 0.)
    gnss_outage_s: tuple = ()
    stationary_start: bool = False

    def __post_init__(self):
        self.gnss_lever_flu_m = vector3(self.gnss_lever_flu_m)
        self.radio_lever_flu_m = vector3(self.radio_lever_flu_m)
        positive = (self.history_s, self.max_imu_gap_s, self.accel_noise_density,
                    self.gyro_noise_density, self.accel_bias_walk, self.gyro_bias_walk,
                    self.gnss_sigma_floor_m, self.gnss_max_sigma_m, self.gnss_unknown_sigma_m,
                    self.gnss_nis_max, self.range_sigma_floor_m, self.range_nis_max)
        if not all(np.isfinite(x) and x > 0 for x in positive) or not np.isfinite(self.min_rsrp_dbm):
            raise ValueError('invalid localization filter parameters')
        if self.gnss_outage_s:
            a, b = self.gnss_outage_s
            if not (0 <= a < b and np.isfinite(b)):
                raise ValueError('gnss_outage_s requires [start_s, end_s]')


@dataclass
class ImuSample:
    rate: np.ndarray
    force: np.ndarray
    orientation: object = None

    def __post_init__(self):
        self.rate = vector3(self.rate)
        self.force = vector3(self.force)
        if self.orientation is not None:
            self.orientation = quaternion_matrix(self.orientation)


@dataclass
class RangeLink:
    name: str
    station: np.ndarray
    distance: float
    sigma: float
    rsrp: float
    valid: bool


@dataclass
class State:
    t: int = 0
    imu_t: int = 0
    imu: object = None
    fixed: bool = False
    pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    vel: np.ndarray = field(default_factory=lambda: np.zeros(3))
    rot: object = None
    ba: np.ndarray = field(default_factory=lambda: np.zeros(3))
    bg: np.ndarray = field(default_factory=lambda: np.zeros(3))
    P: object = None
    last_fix: int = 0
    last_range: int = 0
    fix_jump: float = 0.
    static_samples: tuple = ()

    # History snapshots share arrays. All state transitions replace arrays;
    # never update a shared array in place.
    def advance(self, stamp, noise):
        elapsed = (stamp - self.t) * 1e-9
        if elapsed < 0:
            raise ValueError('nonmonotonic propagation')
        self.t = stamp
        if not elapsed:
            return
        if not self.fixed:
            if self.rot is not None and self.imu is not None:
                self.rot = self.rot @ exp_rotation((self.imu.rate - self.bg) * elapsed)
            return
        steps = max(1, math.ceil(elapsed / .02))
        dt = elapsed / steps
        rate = self.imu.rate - self.bg
        force = self.imu.force - self.ba
        half_rot = exp_rotation(rate * (.5 * dt))
        full_rot = half_rot @ half_rot
        for _ in range(steps):
            mid = self.rot @ half_rot
            acceleration = mid @ force + G_ENU
            self.pos = self.pos + self.vel * dt + .5 * acceleration * dt ** 2
            self.vel = self.vel + acceleration * dt
            self.rot = self.rot @ full_rot
            f = np.zeros((15, 15))
            f[POS, VEL] = I3
            f[VEL, ATT] = -mid @ skew(force)
            f[VEL, BA] = -mid
            f[ATT, ATT] = -skew(rate)
            f[ATT, BG] = -I3
            a = f * dt
            a2 = a @ a
            phi = I15 + a + .5 * a2
            # Simpson integration of the positive-semidefinite noise integrand.
            half = I15 + .5 * a + .125 * a2
            q = dt / 6 * (noise + 4 * (half @ noise @ half.T) + phi @ noise @ phi.T)
            covariance = phi @ self.P @ phi.T + q
            self.P = .5 * (covariance + covariance.T)

    def observe(self, residual, h, covariance, limit, robust=False):
        projected = h @ self.P @ h.T
        innovation = projected + covariance
        inv = np.linalg.inv(innovation)
        nis = float(residual @ inv @ residual)
        if not np.isfinite(nis) or nis > limit:
            return False
        if robust and nis > 4.:
            covariance = covariance * (math.sqrt(nis) / 2)
            inv = np.linalg.inv(projected + covariance)
        gain = self.P @ h.T @ inv
        delta = gain @ residual
        self.pos = self.pos + delta[POS]
        self.vel = self.vel + delta[VEL]
        self.rot = self.rot @ exp_rotation(delta[ATT])
        self.ba = self.ba + delta[BA]
        self.bg = self.bg + delta[BG]
        joseph = I15 - gain @ h
        updated = joseph @ self.P @ joseph.T + gain @ covariance @ gain.T
        reset = I15.copy()
        reset[ATT, ATT] -= .5 * skew(delta[ATT])
        p = reset @ updated @ reset.T
        self.P = .5 * (p + p.T)
        return True

    def antenna(self, lever):
        h = np.zeros((3, 15))
        h[:, POS] = I3
        h[:, ATT] = -self.rot @ skew(lever)
        return self.pos + self.rot @ lever, h

    def fix(self, position, covariance, cfg):
        if self.rot is None or self.imu is None:
            return 'waiting_imu_attitude'
        try:
            position = vector3(position)
        except ValueError:
            return 'invalid_position'
        covariance = np.asarray(covariance)
        if (covariance.shape != (3, 3) or not np.isfinite(covariance).all()
                or not np.allclose(covariance, covariance.T, atol=1e-8)):
            return 'covariance'
        eigenvalues, axes = np.linalg.eigh(covariance)
        if min(eigenvalues) <= 0. or max(eigenvalues) > cfg.gnss_max_sigma_m ** 2:
            return 'covariance'
        covariance = (axes * np.maximum(eigenvalues, cfg.gnss_sigma_floor_m ** 2)) @ axes.T
        if not self.fixed:
            static = self.static_samples
            if cfg.stationary_start:
                if not static or (static[-1][0] - static[0][0]) < 400_000_000:
                    return 'waiting_static_window'
                # Static launch is an explicit operating assumption. Reject
                # changing attitude, vibration and acceleration before biasing.
                rates = np.array([s[1] for s in static])
                forces = np.array([s[2] for s in static])
                expected = self.rot.T @ -G_ENU
                if (np.max(np.linalg.norm(rates, axis=1)) > .06
                        or np.max(np.linalg.norm(forces - expected, axis=1)) > .35):
                    return 'waiting_stationary'
                self.bg = rates.mean(axis=0)
                self.ba = forces.mean(axis=0) - expected
            self.static_samples = ()
            self.fixed = True
            self.pos = position - self.rot @ cfg.gnss_lever_flu_m
            v_std = .2 if cfg.stationary_start else 5.
            self.P = np.diag(np.repeat([1., v_std, .10, .20, .02], 3) ** 2)
            lever_j = self.rot @ skew(cfg.gnss_lever_flu_m)
            self.P[POS, ATT] = lever_j @ self.P[ATT, ATT]
            self.P[ATT, POS] = self.P[POS, ATT].T
            self.P[POS, POS] = covariance + self.P[POS, ATT] @ lever_j.T
        else:
            previous = self.pos
            predicted, h = self.antenna(cfg.gnss_lever_flu_m)
            if not self.observe(position - predicted, h, covariance, cfg.gnss_nis_max):
                return 'innovation'
            self.fix_jump = float(np.linalg.norm(self.pos - previous))
        self.last_fix = self.t
        return 'accepted'

    def range_link(self, link, cfg):
        if not link.valid:
            return 'invalid'
        if not np.isfinite(link.rsrp) or link.rsrp < cfg.min_rsrp_dbm:
            return 'weak_signal'
        if (not np.isfinite([link.distance, link.sigma]).all()
                or min(link.distance, link.sigma) <= 0 or not np.isfinite(link.station).all()):
            return 'invalid_geometry_or_sigma'
        if not self.fixed:
            return 'waiting_fix'
        antenna, h = self.antenna(cfg.radio_lever_flu_m)
        line = antenna - link.station
        predicted = np.linalg.norm(line)
        if predicted < 1e-5:
            return 'zero_geometry'
        sigma = max(link.sigma, cfg.range_sigma_floor_m)
        accepted = self.observe(np.array([link.distance - predicted]),
                                (line / predicted).reshape(1, 3) @ h, np.array([[sigma ** 2]]),
                                cfg.range_nis_max, robust=True)
        if accepted:
            self.last_range = self.t
        return 'accepted' if accepted else 'innovation'

    def output(self, gyro_variance):
        body_velocity = self.rot.T @ self.vel
        pose = np.zeros((6, 15))
        pose[:3, POS] = I3
        pose[3:, ATT] = self.rot
        twist = np.zeros((6, 15))
        twist[:3, VEL] = self.rot.T
        twist[:3, ATT] = skew(body_velocity)
        twist[3:, BG] = -I3
        twist_cov = twist @ self.P @ twist.T
        twist_cov[3:, 3:] += gyro_variance * I3
        return body_velocity, self.imu.rate - self.bg, pose @ self.P @ pose.T, twist_cov


class Timeline:
    """Bounded checkpoint history. Never advance output past the latest IMU."""
    PRIORITY = {'imu': 0, 'fix': 1, 'radio': 2}

    def __init__(self, settings):
        self.cfg = settings
        self.noise = np.diag(np.repeat([0., settings.accel_noise_density,
                                        settings.gyro_noise_density, settings.accel_bias_walk,
                                        settings.gyro_bias_walk], 3) ** 2)
        self.keys, self.samples, self.states, self.outcomes = [], [], [], []
        self.base = State()
        self.latest_imu = self.first_imu = 0
        self.history_ns = round(settings.history_s * 1e9)
        self.gap_ns = round(settings.max_imu_gap_s * 1e9)
        self.counts = Counter()
        self.last_received = {}

    @property
    def state(self):
        return self.states[-1] if self.states else self.base

    def add(self, kind, stamp, sample):
        self.last_received[kind] = stamp
        key = (stamp, self.PRIORITY[kind])
        if stamp <= 0 or stamp < self.latest_imu - self.history_ns or stamp <= self.base.t:
            self.counts[kind + '/too_old'] += 1
            return False
        if kind != 'imu' and self.latest_imu and stamp > self.latest_imu + self.history_ns:
            self.counts[kind + '/too_far_ahead'] += 1
            return False
        idx = bisect_left(self.keys, key)
        if idx < len(self.keys) and self.keys[idx] == key:
            self.counts[kind + '/duplicate'] += 1
            return False
        if kind != 'imu' and len(self.keys) >= 4096:
            self.counts[kind + '/queue_full'] += 1
            return False
        self.keys.insert(idx, key)
        self.samples.insert(idx, sample)
        if kind == 'imu':
            self.first_imu = self.first_imu or stamp
            self.latest_imu = max(self.latest_imu, stamp)
        # Discard superseded snapshots and their diagnostics exactly once.
        start = min(idx, len(self.states))
        for outcome in self.outcomes[start:]:
            self.counts.subtract(outcome)
        del self.states[start:], self.outcomes[start:]
        for j in range(start, bisect_right(self.keys, (self.latest_imu, 2))):
            t, priority = self.keys[j]
            state = copy.copy(self.state)
            outcome = self._apply(state, t, priority, self.samples[j])
            self.states.append(state)
            self.outcomes.append(outcome)
            self.counts.update(outcome)
        trim = bisect_left(self.keys, (self.latest_imu - self.history_ns, 0))
        if trim:
            self.base = self.states[trim - 1]
            for history in (self.keys, self.samples, self.states, self.outcomes):
                del history[:trim]
        return True

    def _apply(self, s, t, priority, sample):
        gap = s.imu_t and t - s.imu_t > self.gap_ns
        if gap and priority != 0:
            return ['fix/imu_gap' if priority == 1 else 'radio/imu_gap']
        outcome = []
        if gap:
            s.__dict__.update(State().__dict__)
            outcome.append('imu/gap_reset')
        s.advance(t, self.noise)
        if priority == 0:
            if not s.fixed:
                if sample.orientation is not None:
                    s.rot = sample.orientation
                if self.cfg.stationary_start:
                    s.static_samples = tuple(x for x in s.static_samples if t - x[0] <= 500_000_000)
                    s.static_samples += ((t, sample.rate, sample.force),)
            s.imu, s.imu_t = sample, t
            return outcome
        if priority == 1:
            elapsed = (t - self.first_imu) * 1e-9
            outage = self.cfg.gnss_outage_s
            if outage and outage[0] <= elapsed < outage[1]:
                return ['fix/masked']
            # Invalid sensor messages are timeline events too, for diagnostics.
            reason = sample if isinstance(sample, str) else s.fix(*sample, self.cfg)
            return ['fix/' + reason]
        seen = set()
        for link in sample:
            if not link.name or link.name in seen:
                reason = 'duplicate_or_missing_id'
            else:
                reason = s.range_link(link, self.cfg)
            seen.add(link.name)
            outcome.extend(['radio/' + reason, 'station/' + link.name + '/' + reason])
        return outcome
