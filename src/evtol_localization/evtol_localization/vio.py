"""Causal monocular/inertial ego odometry. Local axes are level, yaw starts at zero.

The gyro fixes each camera pair's relative rotation. A two-view translation
direction constrains the camera displacement perpendicular to that direction.
Metric scale comes from specific force integrated from an assumed static start.
"""
from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np

from evtol_simulation.coordinates import extrinsic, quaternion_matrix


GRAVITY = np.array([0., 0., 9.80665])
MIN_TRACKS = 30


@dataclass
class Calibration:
    width: int
    height: int
    k: np.ndarray
    d: np.ndarray

    @classmethod
    def from_info(cls, info):
        k = np.asarray(info.k, dtype=float).reshape(3, 3)
        d = np.asarray(info.d, dtype=float)
        if (info.width < 2 or info.height < 2 or not np.all(np.isfinite(k))
                or k[0, 0] <= 0 or k[1, 1] <= 0 or not np.allclose(k[2], [0, 0, 1])
                or not np.allclose([k[0, 1], k[1, 0]], 0)):
            raise ValueError("Invalid CameraInfo intrinsics")
        if (info.distortion_model not in ("", "plumb_bob", "rational_polynomial")
                or d.size not in (0, 4, 5, 8, 12, 14) or not np.all(np.isfinite(d))):
            raise ValueError("Unsupported CameraInfo distortion")
        return cls(info.width, info.height, k.copy(), d.copy())

    def bearings(self, points, shape):
        # Resize uses pixel centres. Map features back to raw image coordinates.
        scale = np.array([self.width / shape[1], self.height / shape[0]])
        pixels = (points.astype(float) + .5) * scale - .5
        xy = cv2.undistortPoints(pixels.reshape(-1, 1, 2), self.k, self.d).reshape(-1, 2)
        return np.column_stack((xy, np.ones(len(xy))))


def gray_image(msg, max_width):
    """Decode the bridge's BGR images, RGB variants or mono8 with row padding."""
    formats = {"mono8": (1, None), "8UC1": (1, None),
               "bgr8": (3, cv2.COLOR_BGR2GRAY), "rgb8": (3, cv2.COLOR_RGB2GRAY),
               "bgra8": (4, cv2.COLOR_BGRA2GRAY), "rgba8": (4, cv2.COLOR_RGBA2GRAY)}
    if msg.encoding not in formats:
        raise ValueError("Unsupported image encoding: " + msg.encoding)
    channels, conversion = formats[msg.encoding]
    h, w, step = msg.height, msg.width, msg.step
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if h < 2 or w < 2 or step < w * channels or raw.size != h * step:
        raise ValueError("Invalid image stride or buffer size")
    data = raw.reshape(h, step)[:, :w * channels]
    data = data.reshape(h, w, channels) if channels != 1 else data.reshape(h, w)
    if w > max_width:
        data = cv2.resize(data, (max_width, max(2, round(h * max_width / w))),
                          interpolation=cv2.INTER_AREA)
    return cv2.cvtColor(data, conversion) if conversion is not None else data.copy()


def translation_direction(b1, b2, rotation, focal):
    """Return camera-1 origin relative to camera-2, expressed in camera-2."""
    rotated = b1 @ rotation.T
    normals = np.cross(b2, rotated)
    n1 = rotated / np.linalg.norm(rotated, axis=1, keepdims=True)
    n2 = b2 / np.linalg.norm(b2, axis=1, keepdims=True)
    angles = np.linalg.norm(np.cross(n1, n2), axis=1)
    valid = np.isfinite(angles) & (angles > 1e-4)
    if np.sum(valid) < MIN_TRACKS or np.median(angles[valid]) < max(.00087, .5 / focal):
        return None, "low_parallax"
    b1, b2, rotated, normals, angles = (a[valid] for a in (b1, b2, rotated, normals, angles))
    rng = np.random.default_rng(42)
    pair = rng.integers(len(b1), size=(128, 2))
    hypotheses = np.cross(normals[pair[:, 0]], normals[pair[:, 1]])
    lengths = np.linalg.norm(hypotheses, axis=1)
    hypotheses = hypotheses[lengths > 1e-9] / lengths[lengths > 1e-9, None]
    if not len(hypotheses):
        return None, "degenerate_directions"

    def residual(direction):
        # Sampson distance with E = [t]x R. Broadcast across hypotheses.
        ep1 = np.cross(direction[:, None], rotated)
        etp2 = np.cross(b2, direction[:, None]) @ rotation
        numerator = np.abs(np.sum(ep1 * b2, axis=2))
        denom = np.sqrt(np.sum(ep1[:, :, :2]**2 + etp2[:, :, :2]**2, axis=2))
        return focal * numerator / np.maximum(denom, 1e-12)

    errors = residual(hypotheses)
    best = np.argmin(np.minimum(errors, 1.5).sum(axis=1))
    good = errors[best] < 1.5
    if good.sum() < max(MIN_TRACKS, .55 * len(b1)):
        return None, "epipolar_inliers"
    _, singular, vt = np.linalg.svd(normals[good], full_matrices=False)
    if singular[1] < .025 * singular[0]:
        return None, "degenerate_directions"
    t = vt[-1]
    good = residual(t[None])[0] < 1.5
    if good.sum() < max(MIN_TRACKS, .55 * len(b1)) or np.median(angles[good]) < max(.00087, .5 / focal):
        return None, "weak_inliers"
    p1 = np.column_stack((np.eye(3), np.zeros(3)))
    p2 = np.column_stack((rotation, t))
    homogeneous = cv2.triangulatePoints(p1, p2, b1[good, :2].T, b2[good, :2].T)
    with np.errstate(divide="ignore", invalid="ignore"):
        xyz = (homogeneous[:3] / homogeneous[3]).T
        depth = xyz @ rotation[2] + t[2]
    finite = np.isfinite(xyz).all(axis=1) & np.isfinite(depth)
    positive = np.sum(finite & (xyz[:, 2] > 0) & (depth > 0))
    negative = np.sum(finite & (xyz[:, 2] < 0) & (depth < 0))
    if max(positive, negative) < max(MIN_TRACKS, .65 * good.sum()):
        return None, "cheirality"
    return (t if positive > negative else -t), "accepted"


def visual_edge(first, second, old_cal, new_cal, r_wc1, r_wc2):
    if first.shape != second.shape:
        return None, "image_shape_changed"
    features = cv2.goodFeaturesToTrack(first, 400, .01, 8)
    if features is None or len(features) < MIN_TRACKS:
        return None, "no_features"
    params = dict(winSize=(21, 21), maxLevel=3)
    moved, forward, _ = cv2.calcOpticalFlowPyrLK(first, second, features, None, **params)
    if moved is None:
        return None, "no_tracks"
    back, backward, _ = cv2.calcOpticalFlowPyrLK(second, first, moved, None, **params)
    if back is None:
        return None, "no_tracks"
    good = forward.ravel().astype(bool) & backward.ravel().astype(bool)
    good &= np.linalg.norm((back - features).reshape(-1, 2), axis=1) < 1.
    p1, p2 = features.reshape(-1, 2)[good], moved.reshape(-1, 2)[good]
    if len(p1) < MIN_TRACKS:
        return None, "no_tracks"
    b1 = old_cal.bearings(p1, first.shape)
    b2 = new_cal.bearings(p2, second.shape)
    focal = new_cal.k[0, 0] * second.shape[1] / new_cal.width
    direction, reason = translation_direction(b1, b2, r_wc2.T @ r_wc1, focal)
    return (None if direction is None else -r_wc2 @ direction), reason


@dataclass
class Sample:
    stamp: int
    acc: np.ndarray
    gyro: np.ndarray


class LocalVio:
    """One stochastic position clone preserves correlations across each visual edge."""

    def __init__(self, camera_xyz_frd, camera_rpy_deg):
        self.arm, camera_q = extrinsic(camera_xyz_frd, camera_rpy_deg)
        self.r_bc = quaternion_matrix(camera_q) @ quaternion_matrix([-.5, .5, -.5, .5])
        if not np.isfinite(self.arm).all() or self.arm.shape != (3,):
            raise ValueError("Invalid camera lever arm")
        self.epoch = 1
        self.resets = 0
        self.last_reset = "none"
        self._clear_state()

    def _clear_state(self):
        self.rewind = deque(maxlen=3)
        self.samples = deque()
        self.startup = deque(maxlen=400)
        self.cursor = None
        self.rotation = np.eye(3)
        self.gyro_bias = np.zeros(3)
        # [current position, velocity, body acceleration bias, previous position]
        self.x = np.zeros(12)
        self.cov = np.diag(np.repeat([.01**2, .1**2, .3**2, 0.], 3))
        self.previous = None
        self.last_stamp = None
        self.accepted = 0
        self.reason = "waiting_for_static_imu"

    @property
    def frame(self):
        return "vio_odom" if self.epoch == 1 else f"vio_odom_epoch_{self.epoch}"

    def _reset(self, reason):
        # A discontinuity after publication invalidates the unanchored frame.
        # Keep the first frame name if the abandoned segment never published.
        if self.accepted:
            self.epoch += 1
        self.resets += 1
        self.last_reset = reason
        self._clear_state()

    def add_imu(self, stamp, acc, gyro):
        acc, gyro = np.asarray(acc, dtype=float), np.asarray(gyro, dtype=float)
        if stamp < 0 or acc.shape != (3,) or gyro.shape != (3,) or not np.isfinite([acc, gyro]).all():
            return False
        if self.last_stamp is not None and stamp <= self.last_stamp:
            # A single late or duplicated message cannot invalidate the frame.
            if self.last_stamp - stamp < 100_000_000:
                self.rewind.clear()
                return False
            if self.rewind and not 0 < stamp - self.rewind[-1] <= 100_000_000:
                self.rewind.clear()
            self.rewind.append(stamp)
            if len(self.rewind) < 3:
                return False
            self._reset("imu_clock_rewind")
        elif self.last_stamp is not None and stamp - self.last_stamp > 100_000_000:
            self._reset("imu_gap")
        self.rewind.clear()
        self.last_stamp = stamp
        sample = Sample(stamp, acc, gyro)
        if self.cursor is None:
            self._initialize(sample)
        else:
            self.samples.append(sample)
            # Keep two seconds for camera transport skew. Propagate older IMU
            # even without images, so a camera outage cannot discard motion.
            while self.samples and (stamp - self.cursor.stamp > 2_000_000_000 or len(self.samples) > 500):
                self._step(self.samples.popleft())
        return True

    def _initialize(self, sample):
        self.reason = "waiting_for_static_imu"
        if np.linalg.norm(sample.gyro) > .05 or abs(np.linalg.norm(sample.acc) - GRAVITY[2]) > .25:
            self.startup.clear()
            return
        self.startup.append(sample)
        while len(self.startup) > 1 and sample.stamp - self.startup[1].stamp >= 500_000_000:
            self.startup.popleft()
        if len(self.startup) < 12 or sample.stamp - self.startup[0].stamp < 500_000_000:
            return
        acc = np.stack([s.acc for s in self.startup])
        gyro = np.stack([s.gyro for s in self.startup])
        if np.max(np.std(acc, axis=0)) > .12 or np.max(np.std(gyro, axis=0)) > .012:
            return
        gravity = np.mean(acc, axis=0)
        roll = np.arctan2(gravity[1], gravity[2])
        pitch = np.arctan2(-gravity[0], np.hypot(gravity[1], gravity[2]))
        self.rotation = cv2.Rodrigues(np.array([0., pitch, 0.]))[0] @ cv2.Rodrigues(np.array([roll, 0., 0.]))[0]
        self.gyro_bias = np.mean(gyro, axis=0)
        self.cursor = sample
        self.startup.clear()
        self.reason = "waiting_for_image"

    def _step(self, sample):
        dt = (sample.stamp - self.cursor.stamp) * 1e-9
        omega = .5 * (sample.gyro + self.cursor.gyro) - self.gyro_bias
        updated = self.rotation @ cv2.Rodrigues(omega * dt)[0]
        r_mid = .5 * (self.rotation + updated)
        world_acc = .5 * (self.rotation @ self.cursor.acc + updated @ sample.acc) - GRAVITY
        # Direct interval integration with a constant body accelerometer bias.
        f = np.eye(12)
        f[0:3, 3:6] = np.eye(3) * dt
        f[0:3, 6:9] = -.5 * dt**2 * r_mid
        f[3:6, 6:9] = -dt * r_mid
        self.x = f @ self.x
        self.x[:3] += .5 * world_acc * dt**2
        self.x[3:6] += world_acc * dt
        # Continuous white acceleration noise density, integrated into p/v.
        q = np.zeros((12, 12))
        accel_psd = .15**2 * np.eye(3)
        q[:3, :3] = accel_psd * dt**3 / 3
        q[3:6, 3:6] = accel_psd * dt
        q[:3, 3:6] = q[3:6, :3] = accel_psd * dt**2 / 2
        self.cov = f @ self.cov @ f.T + q
        self.rotation, self.cursor = updated, sample

    def _advance_to(self, stamp):
        while self.samples and self.samples[0].stamp <= stamp:
            self._step(self.samples.popleft())
        if self.cursor.stamp < stamp:
            a, b = self.cursor, self.samples[0]
            alpha = (stamp - a.stamp) / (b.stamp - a.stamp)
            self._step(Sample(stamp, a.acc + alpha * (b.acc - a.acc),
                              a.gyro + alpha * (b.gyro - a.gyro)))

    def _correct_direction(self, direction, old_rotation):
        delta_arm = (self.rotation - old_rotation) @ self.arm
        delta = self.x[:3] - self.x[9:] + delta_arm
        if delta @ direction <= .01:
            return "unresolved_positive_scale"
        # A unit direction supplies two constraints. Selecting an orthonormal
        # perpendicular basis leaves the along-track scale unmeasured.
        _, _, vt = np.linalg.svd(direction.reshape(1, 3), full_matrices=True)
        basis = vt[1:]
        h = np.zeros((2, 12))
        h[:, :3], h[:, 9:] = basis, -basis
        residual = -basis @ delta
        noise = np.eye(2) * (.04 + .015 * np.linalg.norm(delta))**2
        innovation = h @ self.cov @ h.T + noise
        if residual @ np.linalg.solve(innovation, residual) > 13.82:
            return "direction_innovation"
        gain = np.linalg.solve(innovation, h @ self.cov).T
        candidate = self.x + gain @ residual
        if (candidate[:3] - candidate[9:] + delta_arm) @ direction <= .01:
            return "unresolved_positive_scale"
        self.x = candidate
        update = np.eye(12) - gain @ h
        self.cov = update @ self.cov @ update.T + gain @ noise @ gain.T
        return "accepted"

    def image(self, stamp, image, calibration):
        if self.cursor is None:
            return False
        if (stamp < self.cursor.stamp or stamp > self.last_stamp
                or (self.previous is not None and stamp <= self.previous[0])):
            self.reason = "image_outside_imu_interval"
            return False
        self._advance_to(stamp)
        self.reason = "first_keyframe"
        if self.previous is not None:
            old_stamp, old_image, old_cal, old_rotation = self.previous
            self.reason = "image_gap"
            if stamp - old_stamp <= 1_500_000_000:
                direction, self.reason = visual_edge(old_image, image, old_cal, calibration,
                                                     old_rotation @ self.r_bc, self.rotation @ self.r_bc)
                if direction is not None:
                    self.reason = self._correct_direction(direction, old_rotation)
        self.previous = stamp, image, calibration, self.rotation.copy()
        self.x[9:] = self.x[:3]
        self.cov[9:, :] = self.cov[:3, :]
        self.cov[:, 9:] = self.cov[:, :3]
        if self.reason == "accepted":
            self.accepted += 1
            return True
        return False
