# evtol_localization

GNSS fix + IMU / GNSS fix + IMU + per-anchor 5G-A causal ENU ESKF.

中文启动、配置、Foxglove、异常恢复和回放教程：[`docs/自机定位第一版.md`](../../docs/自机定位第一版.md)。实际 UE 与同包消融结果：[`docs/ego_localization_acceptance.md`](../../docs/ego_localization_acceptance.md)。

The estimator consumes sensor topics and frozen map references only. Truth is consumed by a separate evaluator. Default control remains truth; localization runs alongside it. The state adapter is the sole dynamic TF owner.

This first version uses bounded delayed ordering (0.24 s). It does **not** satisfy the protocol's 30 ms state-age target. VIO is deferred. Offline Ceres references are archived separately under `references/ego_localization`.
