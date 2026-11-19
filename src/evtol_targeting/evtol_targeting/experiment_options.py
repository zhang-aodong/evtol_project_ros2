"""Shared algorithm options without ROS or inter-package dependency cycles."""
DYNAMIC_DEFAULTS=dict(enabled=True,horizon_s=3.,step_s=.1,ego_radius_m=.5,
    clearance_m=1.,min_size_m=[1.,1.,.5],sigma_scale=2.,max_age_s=.5,
    max_velocity_std_mps=3.,max_targets=128,degraded_speed_mps=.5)
