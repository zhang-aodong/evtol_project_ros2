# 不提供真值文件的传感器读取试验

在 `../gnss_anchor_trial/src` 的隔离复制版上做三项读取器修改：

1. 用 IMU 第一条 `sim_timestamp_ns` 和 `elapsed_s` 计算时间原点，移除结构化数据对 `ground_truth/truth.csv` 时间戳的依赖。本运行计算出的原点与旧读取器完全相同：`1789030146872719872 ns`。
2. 以 `sensors/imu/imu.csv` 和 `sensors/gnss/raw_observations.csv` 识别结构化数据，真值 CSV 变为可选的评价数据。
3. 沿用已修正的同历元 GNSS fix 虚拟卫星坐标锚；仍需预设的 AirSim 地理原点，不能将该仿真解称为独立星历/原始伪距定位。

从只含 `sensors` 链接、没有 `ground_truth` 文件的隔离输入运行，GNSS-only、GNSS+IMU、正常 D1、40–60 s GNSS 中断 D1 均完成，后 3 组 Ceres `CONVERGENCE`，摘要均记录 `groundtruth_samples_loaded: 0`、`groundtruth_used_for_frame_anchor: false`。四条轨迹分别与 `../gnss_anchor_trial/` 的对应结果逐字节相同，检查记录在 `verification.json`。D0/D2 未做无真值文件逐字节交叉检查；八组量化指标仍取自 `../gnss_anchor_trial/phase_evaluation.json`。

`run_no_truth_check.sh` 和 `run_d1_crosscheck.sh` 可重做输入隔离与轨迹比对；`verify_no_truth.py` 校验时间原点、摘要和文件哈希。两项现有 C++ 回归测试通过。原始 `E:\data\Airsim\run_20260910_164912_087142` 未改动。

本试验只验证当前 GNSS/IMU/5G 运行路径不需要真值文件。相机/LiDAR 几何、传感器标定、5G 真实退化及在线滑窗恢复均未由此验证。
