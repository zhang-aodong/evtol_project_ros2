# 自机定位运行说明

三种模式都在 ROS 2 中按观测时间在线更新。前两组使用同一套 GNSS Fix、IMU 和误差状态卡尔曼滤波器；第二组再逐站融合 5G-A 测距。第三组只使用前视图像、相机内参和 IMU，以静止起点建立局部坐标。

## 当前生效的算法与配置

`dev_master_zad` 统一采用 `origin/feat/ego-localization` 的定位算法。默认、控制台、ROS launch、旧命令入口和录包回放均使用该分支实现，已移除原有 ESKF/FusionStream 实现和新旧定位器选择界面。

| 模式 | 生效模块 | 输出 |
|---|---|---|
| `gnss_imu` | `evtol_localization.absolute_node` | 会话 ENU `/drone1/localization/odometry` |
| `gnss_imu_5g` | 同一分支 ESKF，额外逐站融合 5G-A | 同上 |
| `camera_imu` | `evtol_localization.vio_node` | 局部 `/drone1/localization/vio/odometry` |

默认配置保持定位开启，算法默认采用该分支参数。GNSS/5G 仿真观测误差配置采用分支的 `fix_profile: standalone`、`fiveg_profile: moderate`、额外测距标准差 `fiveg_added_sigma_m: 1.2`；`absolute.min_rsrp_dbm: -32`。standalone 包含 GNSS 漂移和白噪声；moderate 包含额外测距噪声、指定基站失链和 NLOS 偏置。设为 `native` 可单独关闭对应观测退化，定位算法仍使用该分支。

只需按原方式启动，无需额外选择新增实现：

```powershell
.\scripts\start_experiment.ps1 -NewSession -Execute -Actors
```

在 Foxglove 定位页可以选择三个输入模式。绝对定位参数直接写入 `localization.absolute`，例如观测历史窗口、最大 IMU 间隔、GNSS/5G 标准差下限和最低 RSRP；更完整的字段见 `absolute.Settings`。VIO 配置在 `localization.vio`。保存后重新启动 ROS 生效。

现有 `console.local.json` 和导入配置在加载时迁移：`estimator: legacy` 转换为 `absolute`；可对应的旧噪声、门限、杆臂参数转入 `absolute`，显式 `absolute` 值优先；新算法没有对应含义的旧初始化/队列参数移除。已明确保存的传感器噪声 profile 和飞行航点予以保留。`estimator` 仅作为派生状态/旧配置兼容字段，不再提供算法选择。

旧 `ros2 run evtol_localization ego_localization` 和 `python -m evtol_localization.node` 兼容入口根据 mode 调用分支节点；`absolute`、`vio`、`accuracy` 独立入口继续可用。`scripts/compare_localization_bag.py` 也通过同一个 AbsoluteNode 处理记录观测，历史对比报告属于旧实现结果，不能视作当前算法验收。

保留的框架适配包括定位轨迹与状态、Foxglove 三维/X/Y/Z 误差曲线、可配置 GNSS 中断/失锁/偏置/恢复、评价文件写入失败重试。GNSS 两种模式共享会话 ENU，真值只由独立评价器使用。VIO 是局部坐标，尚不能直接接管 ENU 控制；请用 `accuracy --mode camera_imu` 评价初始航向/平移对齐后的误差。

GNSS 采集器同时发布相同原点到 `/simulation/geodetic_reference` 与 `/drone1/gnss/home`。定位节点兼容两个话题，因此旧 bag 仅含前者也可回放。录包保留两个参考、定位输出及现有误差/状态话题。

以下历史数值与 WSL 验证记录来自被合入分支，不是合并后 Windows UE 的新实测结果。当前算法的真实飞行精度仍需在目标场景验证。

## 在 Windows 上运行

在已有 ROS 2、UE/AirSim 环境的仓库根目录打开 PowerShell。先停止旧 ROS 实验，再切换分支并构建。

```powershell
.\scripts\build.ps1
```

让 UE 进入 Play，先保持飞机静止，随后按原实验流程飞行一段有平移、有纹理的航线。第一个终端选择一种定位模式并启动。下面以第二组为例。

```powershell
.\scripts\start_experiment.ps1 -NewSession -LocalizationMode gnss_imu_5g
```

第二个终端在同一仓库运行精度评估。建议在航线开始前启动评估器，确保采集到运动段。默认启动命令仅观测；需要原框架驱动 UE 虚拟飞机沿航点运动时，在启动命令末尾加 `-Execute`。

```powershell
. .\scripts\activate.ps1
$mode = 'gnss_imu_5g'
ros2 run evtol_localization accuracy --mode $mode --seconds 90 --output "reports/$mode.json"
```

| 模式 | 两处命令使用的模式名 | Foxglove 中的定位输出 |
|---|---|---|
| GNSS+IMU | `gnss_imu` | `/drone1/localization/odometry`，会话 ENU |
| GNSS+IMU+5G-A | `gnss_imu_5g` | `/drone1/localization/odometry`，会话 ENU |
| Camera+IMU | `camera_imu` | `/drone1/localization/vio/odometry`，启动点局部系 |

切换模式时先按 Ctrl+C 停止上一组，将 UE 恢复到同一起点。绝对定位的两组可以加 `-UseFusedState`，将融合结果接到原框架的 `/drone1/state/odometry`。默认情况下该状态话题仍由框架的真值适配器提供，查看定位误差应使用表中的估计话题。可选的 `-GnssOutage` 按会话原点后第 40–60 s 暂停 GNSS 更新，与误差评价共用时间基准；控制台仍可配置按起飞时刻计时。

## 历史航线回放结果

下表取自 AirSim 归档航线 `run_20260910_164912_087142` 的离线逐条观测回放，表示本分支估计器在该次仿真数据上的表现。绝对定位的共同评分窗为 14.51–88.73 s，2294 个真值历元均有估计。两组使用同一份带噪 GNSS Fix 和 IMU。归档 IMU 缺少姿态四元数，回放用比力初始化倾角并假设起始机头朝北；在线节点从框架 IMU 消息获取初始姿态。

| 模式 | 三维位置 RMSE | 1 s / 5 s 相对位移 RMSE |
|---|---|---|
| GNSS+IMU | 2.119 m，绝对 | 0.590 / 1.282 m |
| GNSS+IMU+5G-A | 1.002 m，绝对 | 0.495 / 1.023 m |
| Camera+IMU | 3.669 m，整段拟合航向与平移后，单位尺度 | 0.735 / 2.452 m |

第二组从会话开始给每条测距独立叠加标准差 **1.2 m** 的白噪声，并保留原测距误差、指定基站的非视距偏差和失链时段。当原测距标准差为 0.15 m 时，合成标准差约为 1.209 m。1.2 m 是本次中度退化场景的仿真假设，实机标定需要对应的基站部署和外场测距数据。

Camera+IMU 在 299 个候选关键帧中有 242 帧通过视觉更新，最长输出间隔为 3.66 s。局部里程计在悬停、弱纹理或转弯中可能暂停发布。历史数据的相对位移指标只使用连续输出段。在线评估器采用首个共同位姿对齐航向和平移，保留单位尺度，并同时给出有效真值历元比例与 1 s / 5 s 相对位移误差；它与上表的整段对齐方法不同。IMU 断档或时钟回退后 VIO 会重新静止初始化，并在新的 `vio_odom_epoch_*` 坐标系输出，需重新启动评估器以单独统计该段。

本分支已在 WSL 的 ROS 2 Jazzy 环境构建，并用独立进程发送带时间戳的测试观测，验证了三种模式的里程计输出与评估 JSON。测试使用 Cyclone DDS 环回和小 UDP 分片。Windows 上的完整 UE 场景、2K 图像吞吐和在线精度需在目标机验证。若视觉没有输出，可在 `/diagnostics` 的 `/drone1/localization/vio` 条目查看 `imu_stamp_ns`、`image_stamp_ns`、`camera_info_stamp_ns` 和 `tracking`，区分缺少输入与弱视差。

## 用同一份数据公平比较

可在启用了任一定位模式的仿真中运行 `.\scripts\record.ps1` 录包，包含相机、IMU、GNSS、5G-A、地理原点、会话原点和评估真值。尽量从静止起点开始录制。回放前停止实时实验。为每种模式依次执行下列三步，每个终端都先运行 `. .\scripts\activate.ps1`。

1. 在第一个终端启动待测估计器。每次切换模式，先停掉上一组进程。

   ```powershell
   $mode = 'gnss_imu_5g'
   $cfg = python -m evtol_localization.settings --config config/targeting.yaml --mode $mode
   $estimator = if ($mode -eq 'camera_imu') { 'vio' } else { 'absolute' }
   ros2 run evtol_localization $estimator --config $cfg --ros-args -p use_sim_time:=true
   ```

2. 在第二个终端启动前面的 `accuracy` 命令，选择同一模式。按 bag 时长调整 `--seconds`，预留启动余量。

3. 在第三个终端将 `$bag` 设为实际录包目录，播放原始输入与评估真值。

   ```powershell
   $bag = 'bags\sim_YYYYMMDD_HHMMSS'
   ros2 bag play $bag --qos-profile-overrides-path src/evtol_bringup/config/bag_qos.yaml --topics /clock /simulation/reference_origin /drone1/gnss/home /drone1/imu/data /drone1/gnss/fix /drone1/five_ga/observations /drone1/camera/rgb/image_raw /drone1/camera/rgb/camera_info /drone1/ground_truth/local_odometry
   ```
