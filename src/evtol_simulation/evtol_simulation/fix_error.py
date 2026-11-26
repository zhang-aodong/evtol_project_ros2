"""Stationary, repeatable position-domain perturbations for native GNSS fixes."""
import math

import numpy as np

from .gnss_model import enu_to_ecef_matrix, geodetic_to_ecef


def fix_fields(gps, timestamp_ns, error_model=None):
    geo = gps['geo_point']
    coordinates = [float(geo[k]) for k in ('latitude', 'longitude', 'altitude')]
    valid = gps['fix_type'] >= 2
    variance = np.square([float(gps['eph']), float(gps['eph']), float(gps['epv'])])
    if error_model is not None and valid and np.all(np.isfinite(coordinates)):
        coordinates = displace_fix(*coordinates, error_model.sample(timestamp_ns))
        variance += error_model.variance_m2
    return coordinates, valid, np.diag(variance).ravel().tolist()


def displace_fix(latitude, longitude, altitude, enu_m):
    """Apply a metric tangent-plane offset and return WGS84 geodetic coordinates."""
    ecef = geodetic_to_ecef(latitude, longitude, altitude)
    x, y, z = ecef + enu_to_ecef_matrix(latitude, longitude) @ enu_m
    # Bowring's near-surface WGS84 inverse. The normal projection used for
    # height stays well conditioned even for a receiver near either pole.
    a = 6378137.0
    f = 1.0 / 298.257223563
    b = a * (1.0 - f)
    e2 = f * (2.0 - f)
    p = math.hypot(x, y)
    theta = math.atan2(z * a, p * b)
    lat = math.atan2(z + e2 / (1.0 - e2) * b * math.sin(theta)**3,
                     p - e2 * a * math.cos(theta)**3)
    height = p * math.cos(lat) + z * math.sin(lat) - a * math.sqrt(1.0 - e2 * math.sin(lat)**2)
    return math.degrees(lat), math.degrees(math.atan2(y, x)), height


class StandaloneFixError:
    """First-order Gauss-Markov bias plus independent per-epoch white noise (ENU)."""
    bias_std_m = np.array([1.2, 1.2, 1.8])
    white_std_m = np.array([0.8, 0.8, 1.1])
    correlation_time_s = 30.0
    variance_m2 = bias_std_m**2 + white_std_m**2

    def __init__(self, seed=20260911):
        self.rng = np.random.default_rng(seed)
        self.bias_m = None
        self.last_ns = None

    def sample(self, timestamp_ns):
        if self.bias_m is None:
            self.bias_m = self.rng.normal(scale=self.bias_std_m)
        else:
            dt_s = (timestamp_ns - self.last_ns) * 1e-9
            if dt_s <= 0:
                raise ValueError('GNSS acquisition timestamps must increase')
            decay = math.exp(-dt_s / self.correlation_time_s)
            innovation = math.sqrt(-math.expm1(-2.0 * dt_s / self.correlation_time_s))
            self.bias_m = decay * self.bias_m + innovation * self.rng.normal(scale=self.bias_std_m)
        self.last_ns = timestamp_ns
        return self.bias_m + self.rng.normal(scale=self.white_std_m)
