"""3-D SORT: constant-velocity Kalman tracks and class-aware global assignment.

Inputs contain measured positions/classes only. Simulator identities and ground
truth must never influence prediction, assignment or track creation.
"""
from __future__ import annotations

import numpy as np
from scipy.optimize import linear_sum_assignment


class MultiTargetTracker:
    def __init__(self, max_age_s=1.0, association_distance_m=3.0,
                 measurement_std_m=0.4, acceleration_std_mps2=2.0):
        self.max_age_ns = int(max_age_s * 1e9)
        self.distance = float(association_distance_m)
        self.variance = float(measurement_std_m) ** 2
        self.acceleration_variance = float(acceleration_std_mps2) ** 2
        self.tracks = {}
        self.next_id = 1
        self.last_timestamp_ns = None

    def update(self, detections, timestamp_ns):
        timestamp_ns = int(timestamp_ns)
        if self.last_timestamp_ns is not None and timestamp_ns < self.last_timestamp_ns:
            self.tracks.clear()  # Keep IDs monotonic, never recycle an old identity.
        # A single scan may pair to multiple camera frames. Do not assimilate
        # that same 3-D measurement repeatedly as independent evidence.
        repeated = timestamp_ns == self.last_timestamp_ns
        self.last_timestamp_ns = timestamp_ns
        for key, track in list(self.tracks.items()):
            if timestamp_ns - track["last_observed_ns"] > self.max_age_ns:
                del self.tracks[key]
                continue
            dt = max(0., (timestamp_ns - track["timestamp_ns"]) * 1e-9)
            f = np.eye(6)
            f[:3, 3:] = np.eye(3) * dt
            g = np.vstack((np.eye(3) * dt**2 / 2, np.eye(3) * dt))
            track["x"] = f @ track["x"]
            track["p"] = f @ track["p"] @ f.T + g @ g.T * self.acceleration_variance
            track["timestamp_ns"] = timestamp_ns
            track["observed"] = False

        measurements = [dict(d) for d in detections]
        valid = [i for i, d in enumerate(measurements)
                 if d.get("position_enu_m") is not None
                 and np.asarray(d["position_enu_m"]).shape == (3,)
                 and np.isfinite(d["position_enu_m"]).all()]
        tracks = list(self.tracks.values())
        costs = np.full((len(valid), len(tracks)), np.inf)
        for i, index in enumerate(valid):
            d = measurements[index]
            z = np.asarray(d["position_enu_m"])
            for j, track in enumerate(tracks):
                if int(d["class_id"]) != track["class_id"]:
                    continue
                residual = z - track["x"][:3]
                covariance = track["p"][:3, :3] + np.eye(3) * self.variance
                # Absolute gate prevents uncertain tracks claiming distant objects.
                if np.linalg.norm(residual) <= self.distance:
                    mahalanobis = float(residual @ np.linalg.solve(covariance, residual))
                    if mahalanobis <= 16.27:  # 99.9% chi-square gate, 3 DOF.
                        costs[i, j] = mahalanobis
        assignments = {}
        if len(valid) and len(tracks):
            penalty = (min(len(valid), len(tracks)) + 1) * 17.
            augmented = np.full((len(valid), len(tracks) + len(valid)), penalty)
            augmented[:, :len(tracks)] = np.where(np.isfinite(costs), costs, penalty * 2)
            rows, cols = linear_sum_assignment(augmented)
            assignments = {valid[i]: (tracks[j], i, j) for i, j in zip(rows, cols)
                           if j < len(tracks) and np.isfinite(costs[i, j])}
        for index, d in enumerate(measurements):
            if index not in valid:
                d.update(track_id=None, track_state="unavailable", velocity_enu_mps=None)
                continue
            z = np.asarray(d["position_enu_m"], dtype=float)
            if index in assignments:
                track, row, col = assignments[index]
                alternatives = np.concatenate((np.delete(costs[row], col), np.delete(costs[:, col], row)))
                ambiguous = bool(np.any(np.isfinite(alternatives) & (alternatives < costs[row, col] + 2.)))
                if not repeated:
                    covariance = track["p"][:3, :3] + np.eye(3) * self.variance
                    gain = np.linalg.solve(covariance, track["p"][:3, :]).T
                    track["x"] += gain @ (z-track["x"][:3])
                    correction = np.eye(6)
                    correction[:, :3] -= gain
                    track["p"] = (correction @ track["p"] @ correction.T
                                  + self.variance * gain @ gain.T)
                    track["hits"] += 1
                track["ambiguous"] = ambiguous
            else:
                track = {"id": self.next_id, "class_id": int(d["class_id"]),
                         "x": np.concatenate((z, np.zeros(3))),
                         "p": np.diag([self.variance]*3 + [self.distance**2]*3),
                         "hits": 1, "timestamp_ns": timestamp_ns, "ambiguous": False}
                self.tracks[self.next_id] = track
                self.next_id += 1
            track["last_observed_ns"] = timestamp_ns
            track["observed"] = True
            d.update(track_id=track["id"], track_state="observed",
                     track_hits=track["hits"], track_ambiguous=track["ambiguous"],
                     filtered_position_enu_m=track["x"][:3].tolist(),
                     velocity_enu_mps=track["x"][3:].tolist())
        return measurements, self.snapshot(timestamp_ns)

    def snapshot(self, timestamp_ns):
        return [{"track_id": t["id"], "class_id": t["class_id"],
                 "track_state": "observed" if t["observed"] else "predicted",
                 "position_enu_m": t["x"][:3].tolist(), "velocity_enu_mps": t["x"][3:].tolist(),
                 "age_since_observation_s": (timestamp_ns-t["last_observed_ns"])*1e-9,
                 "track_hits": t["hits"], "track_ambiguous": t["ambiguous"]}
                for t in self.tracks.values()]
