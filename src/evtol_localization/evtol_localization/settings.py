"""Create reproducible experiment configs without rewriting the operator's YAML."""
import argparse
from copy import deepcopy
from pathlib import Path
import tempfile

import yaml


MODULES = {
    "gnss_imu": "evtol_localization.absolute_node",
    "gnss_imu_5g": "evtol_localization.absolute_node",
    "camera_imu": "evtol_localization.vio_node",
}


def estimator_module(settings):
    section = settings.get("localization", {})
    if not section.get("enabled", False):
        return None
    mode = section["mode"]
    if mode not in MODULES:
        raise ValueError(f"Unknown localization mode {mode!r}")
    if mode == "camera_imu" and settings["state"]["source"] == "fused":
        raise ValueError("Local VIO requires a map alignment before it can own map_enu state")
    return MODULES[mode]


def experiment_config(base, mode, fused, outage):
    settings = deepcopy(base)
    loc = settings.setdefault("localization", {})
    loc.update(enabled=True, mode=mode, estimator='absolute' if mode!='camera_imu' else 'vio',
               state_source='fused' if fused else 'truth')
    # Branch stress profiles are opt-in experiments, never baseline defaults.
    loc.update(fiveg_added_sigma_m=1.2,fiveg_profile='moderate',fix_profile='standalone')
    loc.setdefault('absolute',{}).setdefault('min_rsrp_dbm',-32.)
    if outage:
        if mode == "camera_imu":
            raise ValueError("Camera/IMU mode has no GNSS updates to mask")
        # One shared session time base for estimator, evaluation and console.
        loc.setdefault("absolute", {}).pop("gnss_outage_s", None)
        loc['gnss_fault']=dict(enabled=True,time_base='session',default_mode='normal',
            segments=[dict(start_s=40.,end_s=60.,mode='dropout',bias_enu_m=[0.,0.,0.])])
    settings["state"]["source"] = "fused" if fused else "truth"
    estimator_module(settings)
    from .options import options
    settings['localization']=options(loc)
    return settings


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--mode", choices=MODULES, required=True)
    parser.add_argument("--fused", action="store_true")
    parser.add_argument("--outage", action="store_true")
    args = parser.parse_args()
    path = args.config.resolve()
    settings = experiment_config(yaml.safe_load(path.read_text(encoding="utf-8-sig")),
                                 args.mode, args.fused, args.outage)
    root = Path(settings.get("project_root", path.parent.parent)).resolve()
    settings["project_root"] = str(root)
    parent = root / "data" / "launch_runs"
    parent.mkdir(parents=True, exist_ok=True)
    folder = Path(tempfile.mkdtemp(dir=parent, prefix=f"{args.mode}_"))
    config = folder / "targeting.yaml"
    config.write_text(yaml.safe_dump(settings, allow_unicode=True, sort_keys=False), encoding="utf8")
    print(config)


if __name__ == "__main__":
    main()
