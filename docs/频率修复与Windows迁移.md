# 传感器频率修复与 Windows 11 迁移

2026-11-10 最新版本已完成 UE 固定步长调度、雷达计算、原生语义转换和 IMU 历史共享内存修复。双路 2K、双雷达、完整算法栈与 Foxglove 传输负载下，60 秒严格验收为 `PASS_AIRSIM`。本页以下保留前期定位过程，当前结果与换机重编译步骤以 [最新频率修复验证](传感器频率修复验证20261110.md) 为准。

## 数据通路

| 数据 | UE 到采集节点 | ROS 2 发布 |
| --- | --- | --- |
| RGB / 语义图 | UE 异步 GPU 读回 → Windows 命名共享内存、双路配对环 | 原始 BGR8 图像、类别 ID、CameraInfo，20 Hz |
| LiDAR | UE 雷达生成 → Windows XYZ 共享内存环 | PointCloud2，20 Hz |
| IMU | UE 原生 ImuSimple 输出 → Windows 历史共享内存，约 100 ms 缓冲；RPC 适配器保留供对比 | Imu，100 Hz |
| GPS | AirSim RPC `getGpsData` | NavSatFix，10 Hz |
| GNSS 原始观测 | RPC 真值驱动原工程随机模型，明确标记 synthetic | 自定义观测，10 Hz |
| 真值与时钟 | RPC 状态时钟对运动学查询夹取时间 | Odometry、TF、/clock，20 Hz |
| 5G | RPC 真值 + 独立连接的异步 LOS 射线查询 + 原工程随机模型 | 自定义观测，10 Hz |

相机与雷达此前已经采用共享内存。ROS 2 进程间仍使用 DDS；2026-11-10 起默认使用 UDP 配置，绕过本机 Fast DDS 原生访问冲突；旧 SHM + UDP XML 保留供对比。这与 UE 图像/雷达环是两层不同的传输，UE 共享内存仍然启用。当前实现不宣称跨进程零复制。见 [Fast DDS 崩溃排查与修复](FastDDS崩溃排查与修复.md)。

## 定位证据与修复

1. 直接观测共享内存头，相机源约 19.98 Hz，但原 ROS 话题只有 16 Hz。性能分析发现 `Image(data=bytes)` 经生成的 Python 构造器和 setter 重复分配大数组，另有 `ndarray.tobytes()` 临时缓冲。现在填充消息公开的 `data` 数组，一次复制，避免这些冗余复制；完整图像尺寸、像素和时间戳不变。
2. 原读者只取 latest，处理稍慢便跳过仍保留在环中的新帧。现在相机、雷达按顺序读取仍驻留的下一帧；首次连接从最新帧开始，超过环容量时跳到最早仍驻留帧并记录跳过数量。不会重发旧帧补频率，不能靠缓冲弥补长期算力不足。
3. 曾有两个 UE 游戏实例写入同名 `AirSimLidar_Lidar1`，短测帧号异常跳变。已停止额外独立游戏实例，保留主离屏仿真及 UE 编辑器。新增启动前游戏实例检查，以及按 AirSim endpoint/vehicle/kind 的 Windows 命名互斥量，阻止重复采集节点跨 ROS 域启动。
4. 5G 每隔一秒的多基站 LOS 查询原来串行阻塞发布。现由后台线程、独立 RPC 连接更新真实 LOS 快照；初次真实查询完成前不发布，快照过期时明确报错，不把未知状态当作 LOS。模型直接读取该快照，避免第二层缓存延长有效期。快照刷新沿用原配置 `los.update_interval_s`，允许最大年龄为 `max(2.5 × interval, 0.5)` 秒；运动学、距离和随机观测仍按当前 10 Hz 采集时间计算。LOS 本身不是 10 Hz 新射线测量。
5. IMU 使用 1.25 ms 重试间隔与 100 Hz 发布门控，只接受新的源时间戳；采集进程请求 Windows 1 ms 调度，关闭 Windows 11 对该进程忽略计时精度的节能策略，IMU 使用 Above Normal 优先级。没有使用 High/Realtime 优先级、CPU 绑核或特定 GPU 配置。
6. 高频计时使用 `time.perf_counter()`。原 Windows `time.monotonic()` 的约 15.6 ms 粒度不适合判断 100 Hz 间隔。完整验收采用独立 IMU 进程，避免观察者在构造、拷贝、检查大图时阻塞自己的 IMU Python 回调。阈值、原始采集时间戳与空点云检查保持原定义。
7. UE 的 GPU 读回槽按槽位遍历，完成顺序可能不是采集顺序。相机在 Python 中按原始采集时间排序，默认 `camera.reorder_window_s=0.2`。这增加约 200 ms 的相机发布延迟并使用少量额外帧内存，以保留乱序帧而不是修改时间戳；超过排序窗口的乱序仍明确报错。启动/停止时缓冲内的少量边界帧不代表长期降频。算法应使用消息时间戳进行融合；若任务对显示延迟要求更严，可在测量后缩小窗口，或在 UE 中修正 GPU 槽调度。
8. IMU RPC 采集放在独立线程，避免该节点的 ROS executor 诊断/调度延迟影响采样；发布线程只读取真实新样本，定时精度、源频率和系统负载仍会影响最坏间隔。关闭节点时先结束线程再释放 RPC。

新增诊断字段：`recent_hz`（最近约 10 秒）、`capture_p95_ms`、`emit_p95_ms`、`source_skipped_frames`。捕获耗时包含正常等待新帧的时间，不应直接与处理耗时相加来推断每帧开销。`wall_hz` 仍是本次进程启动以来的平均值。

## 启动与验证

首次配置将 `config/machine.example.json` 复制成 `config/machine.local.json`，填写本机依赖环境、pixi、UE4Editor、uproject 路径。该文件已加入 gitignore，不随源码迁移。本机路径已填写。

环境变量 `EVTOL_PIXI_WORKSPACE`、`EVTOL_PIXI_EXE`、`EVTOL_UE_EDITOR`、`EVTOL_UE_PROJECT` 可覆盖对应本机配置；`activate.ps1 -PixiWorkspace ... -PixiExe ...` 和 `start_ue.ps1 -Editor ... -Project ...` 的显式参数优先。运行代码无须匹配原机器的 D 盘目录。

```powershell
# 在 ROS 工程根目录。已有仿真时不再启动 UE。
.\scripts\start_ue.ps1 -Settings src\evtol_simulation\config\airsim.source_2k.json
# 等待 UE 就绪，另开终端：
.\scripts\start_simulation.ps1
# 另开终端：
.\scripts\start_visualization.ps1 -Viewer foxglove
```

默认启动双路 2K。640×480 档须同时使用 `airsim.20hz.json` 和 `start_simulation.ps1 -Config src/evtol_simulation/config/simulation.yaml`。不要重复启动同名采集。原 `ros2 launch ...` 仍可用，但便捷脚本额外检查了本机的 UE 游戏实例。

发布运行后，先观察源和发布端，再进行完整验收：

```powershell
. .\scripts\activate.ps1
python scripts\probe_pipeline.py --duration 10
.\scripts\validate.ps1 -Config src\evtol_simulation\config\simulation.source_2k.yaml -Duration 60
```

若要独立保存报告：

```powershell
python scripts\validate_live.py --config src/evtol_simulation/config/simulation.source_2k.yaml --duration 60 --warmup 10 --output reports/my_acceptance.json
```

报告 `frequency_passed` 表示所有平均频率通过；`passed` 及 `result` 还要求时间连续性、数据质量、诊断、TF 和同步全部通过。不能把前者当成完整验收通过。相机 20 Hz、IMU 100 Hz 等门槛未降低；最低频率仍为目标的 97.5%，最大间隔仍为 4 个周期，最大相机/雷达差仍为 50 ms。

历史 UE 测量出现过空雷达帧，直接读取共享内存也可观察到。雷达按实际采样时间差计算转角和射线数，再裁剪水平 FOV；仿真过慢或调度抖动时可能没有有效命中。最新版本已修改并重编译 UE 的固定步长调度和雷达热路径，保持原扫描网格与射线模型，最终窗口无前向空帧。Python 仍保留真实空数据，不用旧帧替换；其它场景无命中时仍应报告并诊断。

## 迁移到另一台 Windows 11

1. 复制本 ROS 工程源码，以及完整匹配的 UE 4.27 工程、场景资产和 AirSim 插件。不要仅复制本工程而遗漏 UE 扩展，也不要只复制 DLL 到不同 UE 版本。
2. 新机安装 Visual Studio 2022 C++ 工具链（ROS）、UE 4.27 所需的 C++ 工具链/SDK、Visual C++ 运行库、pixi 和 Foxglove。按 ROS 2 Jazzy 官方 Windows 安装文档完成系统前置依赖；硬件需有运行双路 2K 仿真的余量。
3. 本工程 `environment/pixi.toml`、`pixi.lock` 固定了现有 Python 3.12.3、NumPy、OpenCV 等依赖；`ros_release.json` 记录验证过的 ROS Windows 归档名和 SHA-256。通过官方 ROS 发布源取得匹配归档，或复制原机归档。bootstrap 会校验哈希，避免混用 Python/ROS ABI。

```powershell
# 路径按新机实际情况填写；依赖工作区须是新目录。
.\scripts\bootstrap_windows.ps1 -RosArchive E:\Installers\ros2-jazzy-20260128-windows-release-amd64.zip -PixiWorkspace E:\Deps\ros2_jazzy
```

此步骤需要网络获取锁定依赖；`bootstrap_windows.ps1` 不会改写已有 pixi manifest。若已安装兼容依赖，直接填写本机配置即可。归档或锁文件不同应先验证兼容性，不要关闭检查强行混用。

4. 编辑 `config/machine.local.json` 中 UE 路径。若完全复制了原配置，检查 UE 传感器源码是否一致：

```powershell
.\scripts\ue_sensor_support.ps1 -PluginDirectory E:\UEProjects\MyProject\Plugins\AirSim
```

`ue_support/` 收录了当前运行插件的相机读回、共享内存、雷达及必要依赖源码快照与 SHA-256 清单。它是匹配版本的恢复材料，不是任意 AirSim 分支的通用补丁。若恢复文件，应先检查差异，再使用同一命令的 `-Apply` 参数；脚本先备份受影响文件。按 UE 项目的编译流程构建 `MyProjectEditor Win64 Development`，确认对应共享内存扩展进入实际加载的插件。

5. 在新路径重新编译 ROS 工作区：

```powershell
.\scripts\build.ps1
.\scripts\install_visualization.ps1
.\scripts\test.ps1
```

不要迁移旧 `build/`、`install/`、`log/` 或 `.pixi/` 来代替重建，它们含有绝对路径及平台构建产物。若复制的是整个目录，可先在新机移走这些旧构建目录。Windows 正在运行的 exe 不能被构建覆盖；编译前停止采集及可视化脚本。

6. 运行预检、源计数检查及 60 秒真实仿真验收。迁移脚本/路径检查已在本机验证；尚未在第二台电脑执行完整安装，其他机器的实时吞吐量需以该机实测为准。
