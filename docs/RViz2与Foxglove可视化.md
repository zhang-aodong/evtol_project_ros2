# Windows 11：RViz2 排查与 Foxglove 可视化

适用工程：`D:\ZAD_Files\evtol_files\evtol_project_ros2`，Windows 原生 ROS 2 Jazzy。

## 本次排查结论（2026-11-05）

- 真实 AirSim 数据正常流动。最新诊断在测量前等待 DDS/TF 发现，8 秒内收到 161 帧原始雷达；155 次按雷达采集时间查询 `map -> drone1/lidar_link` 全部成功，未发现时钟回退。
- RViz 精简配置、独立 ROS 域的官方默认配置以及 Qt desktop OpenGL/DPI 设置均已尝试；窗口截图仍为白色，**尚未确认恢复实际渲染**。窗口抓取也可能无法捕获原生 OpenGL 表面；当前屏幕抓取呈现锁屏背景，无法用它完成最终图形验收。已检测到 NVIDIA、Intel 和 ToDesk 虚拟显示适配器，不能仅凭这些信息断定具体驱动故障。
- 排查开始时存在两个 UE 游戏实例，后其中一个退出。雷达插件使用固定共享内存名 `AirSimLidar_Lidar1`，同一时间只应运行一个写入该名称的实例；重复实例可能造成数据和时钟混用。本次最新测量不存在 TF 不同步，不应通过修改传感器时间戳或伪造静态机体 TF 来消除旧报错。
- 两路 2048×1536、RGB8、20 Hz 图像，仅像素就约 360 MiB/s，直接交给界面显示会带来大量拷贝和渲染负担。因此新增独立 `evtol_visualization` 消费者包，原始传感器发布逻辑和频率不变。
- 已安装并构建 Windows 原生 Foxglove SDK 桥接，**实际 WebSocket/CDR 接收和图像解码测试通过**。尚未在 Foxglove 应用内完成肉眼图形验收；不要把协议测试等同于 GUI 渲染验收。

## 新增组件

```text
原 UE/AirSim -> 六组采集节点 -> 原始 ROS 2 话题 -> 算法 / 原始 rosbag
                                  |
                                  +-> preview_node -> RViz2
                                            |
                                            +-> foxglove_node -> 本机 Foxglove
```

`preview_node` 默认最多 5 Hz，图像宽度不超过 640，点云最多 16000 点。RGB 使用面积缩小和 JPEG；语义图使用最近邻缩小和 PNG，避免产生新标签颜色。点云保留原字段和坐标系。所有预览均沿用原始 `header.stamp`、`frame_id`，不对原始话题降频。

`foxglove_node` 使用官方 `foxglove-sdk==0.30.3`，直接转发 ROS 序列化 CDR 和完整 `ros2msg` schema，包括自定义 GNSS/5G 消息。只向指定话题开放读取；不开放控制指令、参数修改和服务调用。桥接将 `/clock` 转发为 Foxglove 时间，并缓存、为后连接客户端重发 `/tf_static`。

| 用途 | 话题 |
| --- | --- |
| RViz RGB Image | `/drone1/preview/rgb/image` |
| RViz Semantic Image | `/drone1/preview/semantic/image` |
| Foxglove RGB Image | `/drone1/preview/rgb/image/compressed` |
| Foxglove Semantic Image | `/drone1/preview/semantic/image/compressed` |
| 两种界面点云 | `/drone1/preview/lidar/points` |
| 坐标变换与时间 | `/tf`、`/tf_static`、`/clock` |
| 真值/IMU/GNSS/5G | 原有 `/drone1/ground_truth/odometry`、`/drone1/imu/data`、`/drone1/gnss/fix`、`/drone1/gnss/raw_observations`、`/drone1/five_ga/observations` |

## 环境安装与构建

本机已完成；首次在其他机器配置时执行：

```powershell
cd D:\ZAD_Files\evtol_files\evtol_project_ros2
.\scripts\install_visualization.ps1
```

该脚本调用工程的 `activate.ps1`，将 Foxglove SDK 和诊断用 `websockets==15.0.1` 安装在工程 `.visualization_deps`，仅构建 `evtol_visualization`。不需要 WSL、rosbridge 或修改全局 conda/pixi 依赖。OpenCV、NumPy、ROS 消息依赖来自现有 ROS 环境。

依赖已安装、仅修改源码时，也可执行：

```powershell
. .\scripts\activate.ps1
$env:COLCON_EXTENSION_BLOCKLIST = 'colcon_core.event_handler.summary'
colcon build --merge-install --packages-select evtol_visualization
. .\install\local_setup.ps1
```

每个新终端都应激活环境。直接在 `(base)` 终端运行 `rviz2` 找不到命令，只说明该终端未加载 ROS 环境。

## 保持一个仿真和一套采集节点

**如果仿真和采集已经运行，不要重复执行本节启动命令，直接启动下面的可视化脚本。** 停止旧的白屏 RViz 窗口可减少额外订阅；不要同时启动多个预览脚本。

需要从头启动时，终端 A：

```powershell
cd D:\ZAD_Files\evtol_files\evtol_project_ros2
.\scripts\start_ue.ps1 -Settings src\evtol_simulation\config\airsim.source_2k.json
```

等待 UE/RPC/共享内存就绪后，终端 B：

```powershell
cd D:\ZAD_Files\evtol_files\evtol_project_ros2
. .\scripts\activate.ps1
ros2 run evtol_simulation preflight --config src\evtol_simulation\config\simulation.source_2k.yaml
ros2 launch evtol_bringup simulation.launch.py config:=src/evtol_simulation/config/simulation.source_2k.yaml rviz:=false
```

640×480 档使用 `airsim.20hz.json` 与 `simulation.yaml`，两端配置必须配对。UE 重启或 Reset 导致仿真时间回退后，重启采集与可视化，清除旧时间缓存。

## 优先尝试轻量 RViz

终端 C：

```powershell
cd D:\ZAD_Files\evtol_files\evtol_project_ros2
.\scripts\start_visualization.ps1 -Viewer rviz
```

脚本自动激活环境、启动预览节点，加载 `src/evtol_bringup/rviz/preview.rviz`。默认 10 FPS、传感器 Best Effort QoS、深度 1，只开启 RGB 和雷达。视角跟随机体，避免飞机离开初始视野。退出 RViz 后清理本次预览节点。使用鼠标的 Move Camera/Interact 工具调整视角。

点云仍报变换错误时，先关闭当前脚本，再用传感器本地坐标系测试：

```powershell
.\scripts\start_visualization.ps1 -Viewer rviz -LocalFrame
```

此配置固定坐标为 `drone1/lidar_link`，用于检查雷达局部点云，不表示全局地图。它能显示、`map` 配置不能显示时，再检查 TF 和时间。连背景/网格都白时，优先检查图形环境。

图形环境处理顺序：

1. 在已登录、未锁屏的本机桌面测试，比较 ToDesk/远程会话与本地显示。
2. Windows 设置 → 系统 → 显示 → 图形，为 `C:\pixi_ws\ros2-windows\lib\rviz2\rviz2.exe` 选择高性能 GPU，再重启 RViz。
3. 仍白屏时，可尝试 `-SoftwareOpenGL`。这只请求 Qt 使用软件 OpenGL，Ogre 是否支持仍取决于当前构建，不能保证修复。
4. 若仍失败，使用 Foxglove；不必修改仿真采集节点。

当前 RViz 构建不支持 `--opengl210`，不要使用该参数。

## Foxglove 实时可视化

1. 从 [Foxglove 官网](https://foxglove.dev/download) 安装当前版 Windows 桌面应用，或使用 Chrome 打开 [Foxglove Web](https://app.foxglove.dev/)。按应用提示登录。推荐先用桌面版，避免浏览器本地网络访问权限的影响。
2. 保持采集运行，关闭 RViz 预览脚本后，在终端 C 执行：

```powershell
cd D:\ZAD_Files\evtol_files\evtol_project_ros2
.\scripts\start_visualization.ps1 -Viewer foxglove
```

3. 在 Foxglove 选择 **Open connection → Foxglove WebSocket**，输入 `ws://127.0.0.1:8765`，点击 Open。当前 SDK 使用 `foxglove.sdk.v1` 子协议，需使用新版 Foxglove；本项目不是 rosbridge，不要选择 rosbridge 连接。
4. 添加 **3D** 面板，设置显示坐标系/Display frame 为 `map`，打开 `/tf` 和 `/tf_static` 变换，以及 `/drone1/preview/lidar/points` 点云。点大小可从 2 px 开始、历史保留时间设为 0。调整视角；若飞机已经飞远，让相机跟随 `drone1/base_link`。
5. 添加两个 **Image** 面板，分别选择上表中的 RGB、语义 `.../compressed` 话题。
6. 添加 **Plot** 面板，将 `/drone1/imu/data` 的角速度、加速度字段从 Topics 拖入；从 `/drone1/ground_truth/odometry` 拖入位置、速度字段。添加 **Raw Messages** 面板查看 GNSS、5G 自定义观测。
7. 将布局保存为本地布局，之后重复使用。退出时在终端按 Ctrl+C，脚本清理本次预览节点。桥接默认仅监听本机。

如当前端口已被本次测试或旧桥接占用，直接连接现有端口，或先停止旧脚本。`-Port 8766` 可改用其他端口，但不要为此同时运行重复预览节点。工程不自动下载安装桌面应用，也未对你的 Foxglove 布局进行图形验收。

官方参考：[Python SDK 连接步骤](https://docs.foxglove.dev/docs/getting-started/python)、[ROS 2 数据与 MCAP](https://docs.foxglove.dev/docs/connecting-to-data/frameworks/ros2)。官方 Linux `apt install ros-...-foxglove-bridge` 命令不能在 Windows PowerShell 直接使用；这里提供的是工程内的原生 Windows SDK 适配器。

## 一键数据链路诊断

保持 Foxglove 脚本运行，另开终端：

```powershell
cd D:\ZAD_Files\evtol_files\evtol_project_ros2
. .\scripts\activate.ps1
$env:PYTHONPATH = "$PWD\.visualization_deps;$env:PYTHONPATH"
python scripts\diagnose_visualization.py --duration 8 --websocket ws://127.0.0.1:8765
```

报告写入 `reports/visualization_diagnostic.json`，验证按采集时间的 TF、时钟回退、WebSocket 广告、CDR 解码、JPEG/PNG 解码、点云大小以及晚加入的静态 TF。它不启动采集，不操作无人机。省略 `--websocket` 可只检查 ROS 链路。

本次结果保存在 `reports/visualization_20261105.json`：12 个话题均接收到实际数据；8 秒内 IMU 797 条，GNSS/5G 各 80 条，TF 159 条、静态 TF 1 条，RGB/语义/点云预览分别 35/36/35 帧。预览约 4.3–4.5 Hz，是“最多 5 Hz”的丢帧式预览策略，不是原始传感器验收频率。最后一帧 RGB 约 24 KB、语义约 9 KB、点云约 109 KB（均含 CDR 消息字段），具体负载随场景变化。

`scripts/check_tools.py` 以往只证明 RViz 启动并订阅，不能证明其窗口显示正常。本次 WebSocket 测试也不代替图形验收。完整原始频率验收继续使用 `scripts/validate.ps1`；历史 60 秒传感器通过结果不表示高负载图形/录包同时运行也一定达标。

## 录包与离线查看

原始算法数据仍录原始话题。已有原始录制方式保持不变。想制作轻量可视化包时，在预览脚本运行期间执行：

```powershell
. .\scripts\activate.ps1
ros2 bag record -s mcap -o bags/visualization --qos-profile-overrides-path src/evtol_bringup/config/bag_qos.yaml /clock /tf /tf_static /drone1/preview/lidar/points /drone1/preview/rgb/image/compressed /drone1/preview/semantic/image/compressed /drone1/imu/data /drone1/gnss/fix /drone1/gnss/raw_observations /drone1/five_ga/observations /drone1/ground_truth/odometry
```

Ctrl+C 停止后，把 `.mcap` 文件直接拖入 Foxglove 查看，无需实时桥接。这是预览包；要训练、离线算法和正式采集，应另录完整原始传感器话题。每次选择新的输出目录。回放包时应停止真实采集，避免多个 `/clock` 和相同话题发布者。
