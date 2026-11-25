# EVTOL Windows ROS 2 仿真数据功能包

自机定位支持 GNSS+IMU、GNSS+IMU+5G-A 和 Camera+IMU。三种模式的启动命令、录包回放和精度指标见[自机定位运行说明](docs/ego-localization.md)。第一版的 UE 测试数据见[历史验收记录](docs/ego_localization_acceptance.md)。

当前默认采用 **4 m 近距点云避障、6 m 局部搜索、2 Hz 路径规划、20 Hz 速度碰撞检查**；动态避碰预测缩短至 1 s，预警仍独立保留前向 10 m 半球与 5 m 雷达。相机为每路 70°、偏航 ±15°、总覆盖约 100°。当前配置和实测见 [近距避障与2Hz规划](docs/近距避障与2Hz规划.md)；采样路径和相机说明见 [相机100度与采样匀速飞行](docs/相机100度与采样匀速飞行.md)。此前宽视场 32 机预警率不能代表当前配置性能。


新增可选话题 ROS 2 bag 和视觉训练数据采集：Foxglove 控制台的数据采集页支持参数导入导出、YOLO 检测/分割标签与原始语义掩码保存。见 [数据采集与训练数据导出](docs/数据采集与训练数据导出.md)。


新增融合动态避障、范围内障碍机检测率、Foxglove 实验页及完整配置导入导出；支持 4/32 机场景模板和 64 架配置容量。使用与能力边界见 [融合避障与检测实验使用说明](docs/融合避障与检测实验使用说明.md)。



Foxglove 面板新增常驻详细日志、主机／障碍机状态和独立强制结束按钮，使用与关闭语义见 [控制台详细日志与强制结束](docs/控制台详细日志与强制结束.md)。

新增 Foxglove 仿真控制台：运行 `scripts/start_console.ps1` 后，可在界面中分别启停 UE 和 ROS、编辑障碍机与本机航点、交互配置速度—高度剖面。使用方法见 [Foxglove 仿真控制台使用教程](docs/Foxglove仿真控制台使用教程.md)。

障碍机检测结果可在 UE 窗口绘制绿色 3D 框；启动 UE 时加 `-Visible`，实验默认启用绘图节点。配置与坐标说明见 [UE 检测障碍机 3D 框](docs/UE检测障碍机3D框.md)。


航点折线 `/drone1/navigation/waypoint_path` 与实际已走轨迹 `/drone1/navigation/traveled_path` 已接入实验启动、Foxglove 和录包，类型均为 `nav_msgs/Path`。使用方法见 [航点与实际轨迹可视化](docs/航点与实际轨迹可视化.md)。

Windows 节点以 `3221225477` 退出的排查及已验证的 UDP 配置见 [Fast DDS 崩溃排查与修复](docs/FastDDS崩溃排查与修复.md)。默认 DDS 改用 UDP，UE 图像/雷达共享内存保留；使用前重新激活并重启 ROS 节点。

三维航点跟踪、全向雷达避障和末点降落的启动方式见 [三维航点飞行与避障](docs/三维航点飞行与避障.md)。需重启 UE 加载新增导航雷达和固定步长时钟配置。

本次真实绕障与降落结果见 [三维航点验证记录](docs/三维航点验证记录.md)。新增导航雷达后的双路 2K、双雷达与 IMU 100 Hz 已完成最新修复，完整算法栈和 Foxglove 传输负载下的 60 秒严格验收为 `PASS_AIRSIM`，见 [2026-11-10 频率修复验证](docs/传感器频率修复验证20261110.md)。需要更新并重新编译 UE 插件；IMU 默认改用真实样本共享内存历史环，增加约 100 ms 缓冲年龄。

激光视觉定位、目标跟踪、局部避障和独立行人/障碍机控制的基础说明见 [目标定位与场景控制使用说明](docs/目标定位与场景控制使用说明.md)；当前检测权重及推理链路以本文顶部的新检测文档为准。全链路入口 `scripts/start_experiment.ps1` 默认只观察，不控制主机。

传感器频率修复、共享内存诊断和换机流程见 [频率修复与 Windows 迁移](docs/频率修复与Windows迁移.md)。推荐使用 scripts/start_simulation.ps1 启动采集，scripts/validate.ps1 按实际 2K 配置验收。

最新可视化排查与启动方式见 [RViz2 与 Foxglove 可视化](docs/RViz2与Foxglove可视化.md)。已提供独立预览节点及 Windows 原生 Foxglove SDK 桥接；历史 RViz 订阅检查不代表窗口渲染验收通过。

本工程将 UE/AirSim 传感器统一发布为 ROS 2 话题。采集节点只负责获取、坐标转换、生成仿真观测和发布，不执行 YOLO、定位解算、建图、规划或避障。定位和建图应在新的功能包中订阅本工程的话题。

参数与协议来自 `D:\ZAD_Files\evtol_files\evtol_project\airsim_realtime_uav`。完整说明见 [功能包使用说明](docs/功能包使用说明.md)，迁移关系见 [架构与来源](docs/架构与来源.md)。

本机验证结果与各分辨率适用范围见 [验证记录](docs/验证记录.md)。

640×480 与原工程双路 2048×1536 均已完成真实 AirSim 60 秒验收，结果为 `PASS_AIRSIM`；RViz2 订阅、MCAP 录制回放检查及 20 项单元测试通过。验收负载与测量数据见验证记录。

## 快速开始

以下命令在 Windows PowerShell 中执行：

```powershell
cd D:\ZAD_Files\evtol_files\evtol_project_ros2
.\scripts\build.ps1
. .\scripts\activate.ps1

.\scripts\start_visualization.ps1 -Viewer foxglove
.\scripts\start_experiment.ps1 -NewSession -Execute -Actors

Get-Process UE4Editor -ErrorAction SilentlyContinue | Stop-Process -Force

 .\scripts\start_ue.ps1
```

先不连接 UE，验证 ROS 2 发布链路：

```powershell
python scripts\smoke.py --duration 60
```

结果保存在 `reports/mock_acceptance.json`。`PASS_MOCK` 只证明 ROS/DDS 软件链路，不能代替真实 AirSim 验收。

`python scripts\check_tools.py` 可自动检查 RViz2 订阅、MCAP 录制与回放。

连接真实仿真：

1. 按文档备份并配置 AirSim 的 `settings.json`，安装匹配共享内存插件；
2. 启动 UE4.27 MyProject，进入 Play；
3. 运行预检和发布节点：

```powershell
ros2 run evtol_simulation preflight --config src\evtol_simulation\config\simulation.yaml
ros2 launch evtol_bringup simulation.launch.py rviz:=true

.\scripts\start_ue.ps1 -Settings src\evtol_simulation\config\airsim.source_2k.json
ros2 run evtol_simulation preflight --config src\evtol_simulation\config\simulation.source_2k.yaml
```

另开一个 PowerShell 验收：

```powershell
cd D:\ZAD_Files\evtol_files\evtol_project_ros2
.\scripts\validate.ps1 -Duration 60
```

通过时结果为 `PASS_AIRSIM`，并生成 `reports/airsim_acceptance.json`。仿真源频率不足、空雷达、错误数据、缺失话题、TF 或时间不同步均会导致非零退出码。

## 目录

```text
src/
  evtol_sim_interfaces/  # GNSS 和 5G-A 结构化观测消息
  evtol_simulation/      # 六组传感器节点、RPC/共享内存适配、数据验收
  evtol_scenario/        # 可选行人、障碍机和主机固定任务
  evtol_bringup/         # 统一 launch、RViz 和 rosbag QoS 配置
scripts/                # 环境激活、构建、测试、录包和验收
tests/                  # 协议、坐标系、观测模型、验收失败分支测试
docs/                   # 使用说明、接口和来源
reports/                # 本机实际测试证据，包含后端类型
```

目标频率：RGB/语义/相机标定/LiDAR/真值 20 Hz，IMU 100 Hz，GPS/GNSS 原始观测/5G-A 10 Hz。默认性能档为 640×480；源工程双路 2048×1536 配置为 `airsim.source_2k.json` 与 `simulation.source_2k.yaml`，必须成对使用、单独验收。修改 YAML 目标值不会提高 UE 传感器生产频率。

`scripts/start_ue.ps1` 可使用项目内 settings 启动离屏 UE，无需替换全局设置。


[2m 点云应急与模型重训验收](docs/2m点云应急与检测模型重训验收.md)：本次参数、数据、权重、复现方法和未达标项。


[三相机覆盖与预警验收](docs/三相机覆盖与预警验收.md)：当前三路配置、独立复测结果、运行方法与上下盲区限制。
