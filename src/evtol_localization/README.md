# evtol_localization

GNSS+IMU 和 GNSS+IMU+5G-A 在会话 ENU 坐标系输出绝对位置。Camera+IMU 使用前向图像、内参与 IMU，在静止起点建立局部坐标系。

[运行说明](../../docs/ego-localization.md)包含三种模式的启动、录包、回放命令和精度指标。第一版的 Windows UE 测试见[历史验收记录](../../docs/ego_localization_acceptance.md)。

The estimator consumes sensor topics and frozen map references only. Truth is consumed by a separate evaluator. Default control remains truth; localization runs alongside it. The state adapter is the sole dynamic TF owner.

The absolute modes publish to `/<namespace>/localization/odometry`. Camera+IMU publishes to `/<namespace>/localization/vio/odometry`. Run `ros2 run evtol_localization accuracy --mode <mode>` to evaluate a flight.
