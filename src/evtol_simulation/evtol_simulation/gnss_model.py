"""Synthetic GNSS raw-observation model migrated from airsim_5ga_positioning."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


RAW_OBSERVATION_FIELDS = [
    "epoch_index",
    "sim_timestamp_ns",
    "elapsed_s",
    "phase",
    "prn",
    "pseudorange_m",
    "carrier_phase_cycles",
    "doppler_hz",
    "snr_dbhz",
    "elevation_deg",
    "azimuth_deg",
]


def geodetic_to_ecef(
    latitude_deg: float,
    longitude_deg: float,
    altitude_m: float,
    semi_major_axis_m: float = 6_378_137.0,
    flattening: float = 1.0 / 298.257223563,
) -> np.ndarray:
    latitude = np.deg2rad(latitude_deg)
    longitude = np.deg2rad(longitude_deg)
    eccentricity_sq = flattening * (2.0 - flattening)
    sin_lat = np.sin(latitude)
    radius = semi_major_axis_m / np.sqrt(
        1.0 - eccentricity_sq * sin_lat**2
    )
    return np.array(
        [
            (radius + altitude_m) * np.cos(latitude) * np.cos(longitude),
            (radius + altitude_m) * np.cos(latitude) * np.sin(longitude),
            (radius * (1.0 - eccentricity_sq) + altitude_m) * sin_lat,
        ],
        dtype=float,
    )


def enu_to_ecef_matrix(latitude_deg: float, longitude_deg: float) -> np.ndarray:
    latitude = np.deg2rad(latitude_deg)
    longitude = np.deg2rad(longitude_deg)
    sin_lat, cos_lat = np.sin(latitude), np.cos(latitude)
    sin_lon, cos_lon = np.sin(longitude), np.cos(longitude)
    return np.array(
        [
            [-sin_lon, -sin_lat * cos_lon, cos_lat * cos_lon],
            [cos_lon, -sin_lat * sin_lon, cos_lat * sin_lon],
            [0.0, cos_lat, sin_lat],
        ],
        dtype=float,
    )


def ned_to_ecef_delta(
    ned_vector: np.ndarray, latitude_deg: float, longitude_deg: float
) -> np.ndarray:
    enu_vector = np.array(
        [ned_vector[1], ned_vector[0], -ned_vector[2]], dtype=float
    )
    return enu_to_ecef_matrix(latitude_deg, longitude_deg) @ enu_vector


@dataclass(frozen=True)
class Satellite:
    prn: int
    position_ecef_m: np.ndarray
    velocity_ecef_mps: np.ndarray
    ambiguity_cycles: int


class GnssRawSimulator:
    """Generate pseudorange, carrier phase, Doppler and SNR from AirSim NED truth."""

    def __init__(
        self,
        latitude_deg: float,
        longitude_deg: float,
        origin_ecef_m: np.ndarray,
        settings: dict[str, Any],
    ) -> None:
        self.latitude_deg = latitude_deg
        self.longitude_deg = longitude_deg
        self.origin_ecef_m = origin_ecef_m
        self.settings = settings
        self.light_speed_mps = float(settings["light_speed_mps"])
        self.wavelength_m = self.light_speed_mps / float(
            settings["l1_frequency_hz"]
        )
        self.rng = np.random.default_rng(int(settings["random_seed"]))
        rotation = enu_to_ecef_matrix(latitude_deg, longitude_deg)
        self.satellites: list[Satellite] = []

        for index in range(int(settings["satellite_count"])):
            azimuth = self.rng.uniform(0.0, 2.0 * np.pi)
            elevation = self.rng.uniform(np.deg2rad(15.0), np.deg2rad(80.0))
            los_enu = np.array(
                [
                    np.cos(elevation) * np.sin(azimuth),
                    np.cos(elevation) * np.cos(azimuth),
                    np.sin(elevation),
                ]
            )
            los_ecef = rotation @ los_enu
            los_ecef /= np.linalg.norm(los_ecef)
            satellite_position = origin_ecef_m + los_ecef * float(
                settings["satellite_slant_range_m"]
            )
            random_vector = self.rng.normal(size=3)
            perpendicular = np.cross(los_ecef, random_vector)
            if np.linalg.norm(perpendicular) < 1e-6:
                perpendicular = np.cross(los_ecef, np.array([1.0, 0.0, 0.0]))
            perpendicular /= np.linalg.norm(perpendicular)
            speed = self.rng.uniform(
                float(settings["satellite_speed_min_mps"]),
                float(settings["satellite_speed_max_mps"]),
            )
            self.satellites.append(
                Satellite(
                    prn=index + 1,
                    position_ecef_m=satellite_position,
                    velocity_ecef_mps=perpendicular * speed,
                    ambiguity_cycles=int(
                        self.rng.integers(
                            int(settings["ambiguity_min_cycles"]),
                            int(settings["ambiguity_max_cycles"]),
                        )
                    ),
                )
            )

    def simulate_epoch(
        self,
        receiver_position_ecef_m: np.ndarray,
        receiver_velocity_ecef_mps: np.ndarray,
    ) -> list[dict[str, float | int]]:
        observations: list[dict[str, float | int]] = []
        ecef_to_enu = enu_to_ecef_matrix(
            self.latitude_deg, self.longitude_deg
        ).T
        slant_range_m = float(self.settings["satellite_slant_range_m"])
        for satellite in self.satellites:
            delta = satellite.position_ecef_m - receiver_position_ecef_m
            geometric_range_m = float(np.linalg.norm(delta))
            if geometric_range_m < 1.0:
                continue
            los_ecef = delta / geometric_range_m
            east, north, up = ecef_to_enu @ los_ecef
            elevation_rad = np.arctan2(up, np.hypot(east, north))
            elevation_deg = float(np.rad2deg(elevation_rad))
            if elevation_deg < float(self.settings["elevation_mask_deg"]):
                continue
            azimuth_deg = float(
                (np.rad2deg(np.arctan2(east, north)) + 360.0) % 360.0
            )
            relative_velocity = (
                satellite.velocity_ecef_mps - receiver_velocity_ecef_mps
            )
            range_rate_mps = float(np.dot(relative_velocity, los_ecef))
            clock_bias_m = self.light_speed_mps * float(
                self.settings["receiver_clock_bias_s"]
            )
            pseudorange_m = (
                geometric_range_m
                + clock_bias_m
                + self.rng.normal(0.0, float(self.settings["pseudorange_sigma_m"]))
            )
            carrier_phase_cycles = (
                (geometric_range_m + clock_bias_m) / self.wavelength_m
                + satellite.ambiguity_cycles
                + self.rng.normal(
                    0.0, float(self.settings["carrier_phase_sigma_cycles"])
                )
            )
            doppler_hz = -range_rate_mps / self.wavelength_m + self.rng.normal(
                0.0, float(self.settings["doppler_sigma_hz"])
            )
            snr_dbhz = (
                float(self.settings["snr_base_dbhz"])
                + float(self.settings["snr_elevation_gain_db"])
                * max(0.0, np.sin(elevation_rad))
                - 10.0 * np.log10(max(1.0, geometric_range_m / slant_range_m))
                + self.rng.normal(0.0, float(self.settings["snr_sigma_dbhz"]))
            )
            observations.append(
                {
                    "prn": satellite.prn,
                    "pseudorange_m": float(pseudorange_m),
                    "carrier_phase_cycles": float(carrier_phase_cycles),
                    "doppler_hz": float(doppler_hz),
                    "snr_dbhz": float(
                        np.clip(
                            snr_dbhz,
                            float(self.settings["snr_min_dbhz"]),
                            float(self.settings["snr_max_dbhz"]),
                        )
                    ),
                    "elevation_deg": elevation_deg,
                    "azimuth_deg": azimuth_deg,
                }
            )
        return observations
