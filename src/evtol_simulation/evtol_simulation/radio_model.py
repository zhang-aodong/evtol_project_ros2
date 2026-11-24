"""5G-A PRS/Multi-RTT observation simulation.

The numerical model is migrated from airsim_5ga_positioning/g5a_simulator.py.
Observation model retained; position solver and legacy AirSim client adapter removed.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Callable

import numpy as np



LosChecker = Callable[[np.ndarray, np.ndarray], bool]

# Times are simulation seconds since this radio stream's first acquisition.
_D1_OUTAGES = {"gNB6": (20., 55.), "gNB3": (55., 75.), "gNB5": (35., 43.)}
_D1_NLOS = {"gNB2": (30., 46., 0.), "gNB4": (60., 76., 1.)}


@dataclass(frozen=True)
class FiveGAState:
    sample_index: int
    timestamp_ns: int
    elapsed_s: float
    phase: str
    position_ned_m: np.ndarray
    velocity_ned_mps: np.ndarray


@dataclass(frozen=True)
class LinkObservation:
    sample_index: int
    timestamp_ns: int
    elapsed_s: float
    phase: str
    gnb_id: str
    gnb_n_m: float
    gnb_e_m: float
    gnb_d_m: float
    true_range_m: float
    path_loss_db: float
    prs_rsrp_dbm: float
    is_los: int
    range_sigma_m: float
    nlos_bias_m: float
    measured_range_m: float
    measured_rtt_s: float
    measurement_valid: int

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


class FiveGASimulator:
    """Generate 5G-A link observations each epoch."""

    def __init__(
        self,
        settings: dict[str, Any],
        los_checker: LosChecker | None = None,
        cache_los: bool = True,
        *,
        profile: str = "native",
        added_sigma_m: float = 1.2,
    ) -> None:
        self.settings = settings
        self.los_checker = los_checker
        self.cache_los = cache_los
        self.rng = np.random.default_rng(int(settings["random_seed"]))
        if profile not in ("native", "moderate"):
            raise ValueError(f"Unknown 5G-A profile: {profile}")
        self.added_sigma_m = float(added_sigma_m)
        if profile == "moderate" and (not np.isfinite(self.added_sigma_m) or self.added_sigma_m < 0):
            raise ValueError("Added 5G-A range standard deviation must be finite and nonnegative")
        # Keep the native random stream identical across paired comparisons.
        self.stress_rng = np.random.default_rng(20260910) if profile == "moderate" else None
        self._los_cache: dict[str, tuple[float, bool]] = {}
        self.stations = [
            (
                str(station["id"]),
                np.asarray(station["position_ned_m"], dtype=float),
            )
            for station in settings["base_stations"]
        ]

    def free_space_path_loss_db(self, distance_m: float) -> float:
        frequency_ghz = float(self.settings["carrier_frequency_hz"]) / 1e9
        return float(
            32.4
            + 20.0 * np.log10(frequency_ghz)
            + 20.0 * np.log10(max(distance_m, 1.0))
        )

    def rsrp_to_range_sigma_m(self, rsrp_dbm: float) -> float | None:
        for threshold_dbm, sigma_m in self.settings["rsrp_sigma_table"]:
            if rsrp_dbm >= float(threshold_dbm):
                return float(sigma_m)
        return None

    def _link_is_los(
        self,
        state: FiveGAState,
        gnb_id: str,
        gnb_position_ned_m: np.ndarray,
    ) -> bool:
        if self.los_checker is None:
            return True
        if not self.cache_los:
            return self.los_checker(state.position_ned_m, gnb_position_ned_m)
        cached = self._los_cache.get(gnb_id)
        if (
            cached is not None
            and state.elapsed_s - cached[0]
            < float(self.settings["los"]["update_interval_s"])
        ):
            return cached[1]
        is_los = self.los_checker(state.position_ned_m, gnb_position_ned_m)
        self._los_cache[gnb_id] = (state.elapsed_s, is_los)
        return is_los

    def simulate_link(
        self,
        state: FiveGAState,
        gnb_id: str,
        gnb_position_ned_m: np.ndarray,
    ) -> LinkObservation:
        true_range_m = float(
            np.linalg.norm(state.position_ned_m - gnb_position_ned_m)
        )
        is_los = self._link_is_los(state, gnb_id, gnb_position_ned_m)
        shadow_std_db = float(
            self.settings[
                "los_shadow_std_db" if is_los else "nlos_shadow_std_db"
            ]
        )
        path_loss_db = self.free_space_path_loss_db(true_range_m)
        if not is_los:
            path_loss_db += float(self.settings["nlos_extra_loss_db"])
        shadow_db = float(self.rng.normal(0.0, shadow_std_db))
        prs_rsrp_dbm = (
            float(self.settings["tx_power_dbm"])
            + float(self.settings["gnb_gain_dbi"])
            + float(self.settings["ue_gain_dbi"])
            + float(self.settings["prs_processing_gain_db"])
            - path_loss_db
            + shadow_db
        )
        sigma_m = self.rsrp_to_range_sigma_m(prs_rsrp_dbm)
        measurement_valid = int(
            sigma_m is not None
            and prs_rsrp_dbm >= float(self.settings["min_rsrp_dbm"])
        )
        effective_sigma_m = float(sigma_m) if sigma_m is not None else np.nan
        nlos_bias_m = 0.0
        measured_range_m = np.nan
        measured_rtt_s = np.nan
        if measurement_valid:
            if not is_los:
                nlos_bias_m = max(
                    0.0,
                    float(
                        self.rng.normal(
                            float(self.settings["nlos_range_bias_mean_m"]),
                            float(self.settings["nlos_range_bias_std_m"]),
                        )
                    ),
                )
            range_noise_m = float(self.rng.normal(0.0, effective_sigma_m))
            measured_range_m = true_range_m + nlos_bias_m + range_noise_m
        if self.stress_rng is not None:
            # The D1 schedule overlays the ray-traced scene. A scheduled NLOS
            # event retains any native scene loss and bias and adds its own.
            t = state.elapsed_s
            outage = _D1_OUTAGES.get(gnb_id)
            if outage and outage[0] <= t < outage[1]:
                measurement_valid = 0
            nlos = _D1_NLOS.get(gnb_id)
            if nlos and nlos[0] <= t < nlos[1]:
                bias = 3.0 + np.sin(2.0 * np.pi * t / 8.0 + nlos[2])
                measured_range_m += bias
                nlos_bias_m += bias
                path_loss_db += 5.0
                prs_rsrp_dbm -= 5.0
                is_los = False
                if prs_rsrp_dbm < float(self.settings["min_rsrp_dbm"]):
                    measurement_valid = 0
            effective_sigma_m = float(np.hypot(effective_sigma_m, self.added_sigma_m))
            measured_range_m += self.stress_rng.normal(0.0, self.added_sigma_m)
        if measurement_valid:
            measured_range_m = max(measured_range_m, float(self.settings["min_measured_range_m"]))
            measured_rtt_s = (
                2.0
                * measured_range_m
                / float(self.settings["speed_of_light_mps"])
            )
        else:
            measured_range_m = np.nan
        return LinkObservation(
            sample_index=state.sample_index,
            timestamp_ns=state.timestamp_ns,
            elapsed_s=state.elapsed_s,
            phase=state.phase,
            gnb_id=gnb_id,
            gnb_n_m=float(gnb_position_ned_m[0]),
            gnb_e_m=float(gnb_position_ned_m[1]),
            gnb_d_m=float(gnb_position_ned_m[2]),
            true_range_m=true_range_m,
            path_loss_db=path_loss_db,
            prs_rsrp_dbm=prs_rsrp_dbm,
            is_los=int(is_los),
            range_sigma_m=effective_sigma_m,
            nlos_bias_m=nlos_bias_m,
            measured_range_m=measured_range_m,
            measured_rtt_s=measured_rtt_s,
            measurement_valid=measurement_valid,
        )

    def simulate_epoch(self, state: FiveGAState) -> list[LinkObservation]:
        return [
            self.simulate_link(state, gnb_id, gnb_position)
            for gnb_id, gnb_position in self.stations
        ]


