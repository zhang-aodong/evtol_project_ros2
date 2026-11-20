# 双相机 ONNX、路径规划与分层预警

> 当前默认视场和导航已更新：每路 70°、偏航 ±15°、总覆盖约 100°，并启用采样巡航。见 [最新配置与实测说明](相机100度与采样匀速飞行.md)。下文 120°、±45° 和 32 机成绩属于此前版本。

本次实现位于 `evtol_project_ros2`。传感器采集、目标检测、雷达定位、预警、导航、控制和独立评估分节点运行；障碍机真值仅用于评估，不进入目标检测和目标定位链路。自机状态暂使用真值 ENU 位姿，起点是本次会话初始位置，之后可替换为融合定位接口 `/drone1/state/odometry`。

## 当前传感器与模型

| 项目 | 当前配置 |
| --- | --- |
| 左前相机 | `front_left`，偏航 -45°，1536×1536，水平视场 120° |
| 右前相机 | `front_right`，偏航 +45°，1536×1536，水平视场 120° |
| 总水平覆盖 | 约 210°，中间约 30° 重叠 |
| 图像采集/检测上限 | 20 Hz；实际频率受 UE 与整机负载影响 |
| 点云 | 水平 360°；`NavigationLidar` 为 128 线、垂直 180°、20 Hz、230400 点/秒、100 m |
| 检测模型 | `models/drone_detection_b2_1536.onnx`，固定 batch=2、FP16、输入 `[2,3,1536,1536]` |
| 推理后端 | ONNX Runtime CUDAExecutionProvider；要求 GPU，拒绝静默改用 CPU |
| 可视化预览 | 双图横向拼接，默认 5 Hz、总宽 1536；原始每路推理仍为 1536 |

两台普通透视相机不能覆盖完整前向半球顶部和底部。按照已确认的要求，盲区及遮挡目标仍在主统计分母内；几何视场内计数只作诊断。水平 210° 覆盖不代表完整半球覆盖。

当前在线算法是**目标检测框 + 实测激光雷达投影**，不运行语义分割模型。左相机仍可采集语义真值掩码供 YOLO 数据导出使用，这与在线语义分割推理不同。`surround` 和 `models/semantic_runtime` 是兼容保留的历史名称，不代表六路相机或语义推理。

基础参数在 `config/targeting.yaml`；UE 模板在 `src/evtol_simulation/config/airsim.navigation_2k.json`。控制台启动时根据界面配置生成本次 UE 和 ROS 参数快照，实际参数以 `data/console_runs/` 中对应会话快照为准。旧 version=2 控制台配置载入后自动升级为双相机和前半球统计，保留用户的航点、障碍机和 UE 路径；本次部署没有改写原 `config/console.local.json`。

## 两路预警的含义

### 独立雷达近距预警

话题 `/drone1/warning/lidar`，JSON。

1. 从位姿对齐的点云中估计正下方局部地面/屋顶，计算实测 AGL；不是简单使用相对起点 ENU 高度。
2. 只有 **AGL > 5 m** 才启用近距预警。
3. 以本机为球心，5 m 内出现任意有效、非自身回波即预警，包含侧面、后方、上方和下方；自身排除半径默认 0.3 m。
4. 地面估计未知、点云或状态过期时显示无效原因，不显示为安全。正下方有目标时，最近向下表面可能是目标而非真实地面，这是当前 AGL 估计的局限。

这一通道不区分建筑、无人机等语义。原始点云避障一直运行，不受“超过 5 m 才启用雷达预警”的显示门限限制。

### 视觉—激光融合预警

话题 `/drone1/warning/fusion`，JSON。

检测框通过相机内外参关联实测激光点，经过遮挡、聚类和尺寸约束得到 ENU 表面位置，再多目标跟踪估计速度。没有激光支持的二维框不输出虚构三维位置；没有使用障碍机真值补目标。

目标检测和定位不以 10 m 截断，保留雷达量程及实际可观测范围内的目标。**当前融合预警区域**是半径 10 m、沿机体前向的完整半球：

`||目标位置 - 自机位置|| ≤ 10 m` 且 `相对位置 · 机体前向 ≥ 0`。

机体前向由实时姿态计算，转向会改变统计半球。ENU +Y 并非永远是机头方向。

对新鲜的无人机轨迹发布：距离、ENU 位置、绝对速度、相对速度、接近速度、预测最近距离/时刻、未来距离和预计进入半球时间。默认匀速预测 3 s、步长 0.5 s。预测不会扩大“当前 10 m 半球”的预警统计范围；速度突变和大机动时预测有误差。测量超过 0.3 s 即失效，不通过延长历史保持来提高成绩。

## 航点和避障

任务仍按输入顺序到达 ENU 三维航点。默认局部规划 `horizontal_first_astar`：先在同高度尝试八邻域 A* 水平绕行，失败再尝试 26 邻域三维 A*。竖直代价鼓励优先水平绕行，但允许上下避障。

异步规划返回后会用最新点云再次检查路线；速度层继续检查点云、融合目标动态包络、加速度和制动距离。融合目标的当前位置及短时预测位置也参与路线障碍构造。这是有时间预算的局部规划，并非全局最优或完整时空规划；狭窄空间、完全封闭区域和观测不足时允许安全停止，不能承诺任意障碍环境必然到达。

界面中的航点、速度、飞行剖面、障碍机数量和路径功能继续使用。试验支持 32 机，配置容量上限 64 机；容量上限不代表所有机器上都能保持实时。

## Windows 11 启动和 Foxglove

在项目根目录的 PowerShell 中运行：

```powershell
.\scripts\start_console.ps1 -NoBrowser -BridgePort 8766
```

`-NoBrowser` 仅禁止自动打开浏览器。管理器不会自动启动飞行。打开 Foxglove 桌面软件，重新加载/重启软件以加载更新扩展；添加 **EVTOL Simulation Console** 面板，通过 Foxglove WebSocket 连接 `ws://127.0.0.1:8766`。也可使用本地网页 `http://127.0.0.1:9040`。

1. 在 UE 页填写本机的 UE4Editor、uproject 和设置模板路径，选择可视化或后台运行。
2. 在障碍机、本机飞行、避障实验页配置参数，保存。确认双相机为 120°/1536、融合预警和实验统计均为 10 m。
3. 启动 UE，待 AirSim 就绪后启动 ROS。需要真实飞行时启用执行指令和障碍机开关。
4. 避障实验页查看双路预警、AGL、真实数量、正确报警数量、及时成功率、漏警和精确率；监控页查看节点耗时和超时原因。
5. Foxglove 3D 面板固定坐标为 `map_enu`，添加 `/evaluation/markers`、`/drone1/perception/markers`、`/drone1/navigation/lidar/points`、`/drone1/navigation/waypoint_path`、`/drone1/navigation/traveled_path`。图像选择 `/drone1/perception/surround/detections_preview/compressed`。
6. 结束时先正常停止 ROS 和 UE；发生关闭卡住时使用已有强制结束按钮，只结束管理器拥有的进程。

更改相机分辨率、视场或障碍机生成配置后必须重启 UE；不能仅重启 ROS。导入配置仅修改草稿，保存并重启后生效。导出草稿、已保存配置及本次运行配置是三个不同操作，请按需选择。

## 运行环境、编译与迁移

ROS 环境与 GPU 模型环境分开。ROS 使用 Windows ROS2 Jazzy；GPU Python 路径在 `config/machine.local.json` 的 `ModelPython` 或环境变量 `EVTOL_MODEL_PYTHON` 中设置。当前机器 GPU Python 为 `D:\Software\miniconda\envs\yolo-26\python.exe`，迁移时必须改为目标机器的路径。

ONNX Runtime 使用 CUDA 12 / cuDNN 9；当前机器 PyTorch 为 `2.11.0+cu128`。Torch 只在 ONNX 链路中提供 CUDA DLL、GPU 像素预处理及设备缓冲，不加载 PT 模型做推理。安装包版本见根目录 `requirements-onnx.txt`；随项目保留的 `models/onnx_runtime` 是 **CPython 3.11 / Windows x64** 运行依赖，目标 GPU Python 应匹配。CUDA 驱动和 GPU 环境仍需在目标机器安装，不能仅复制项目就替代系统驱动。

若需要重建运行依赖，在匹配的 GPU Python 环境中执行 `python -m pip install -r requirements-onnx.txt`，并安装兼容的 CUDA PyTorch 和 OpenCV、NumPy。RTX 5090 需要支持该显卡的 CUDA/PyTorch 版本；不要在 ROS Python 环境里混装模型依赖。

使用目标机器实际 GPU Python，启动 UE 前先检查：

```powershell
$gpuPython = 'D:\Software\miniconda\envs\yolo-26\python.exe'
& $gpuPython .\scripts\check_detection_onnx.py
```

应显示 CUDAExecutionProvider、FP16 及 `[2,3,1536,1536]`，否则先解决 GPU 环境。纯黑输入只验证执行链路，不验证检测质量。

停止本项目 ROS 后编译：

```powershell
.\scripts\build.ps1
.\scripts\build_console_panel.ps1
```

前端首次构建需要 Node/npm 依赖（在 `foxglove/evtol-console` 执行 `npm ci`）；随后重新运行 `start_console.ps1` 将构建扩展安装到本机 Foxglove 用户目录。原 UE 工程应保留此前已适配的共享内存和 ROI 激光插件；本轮没有增加需要重新编译 UE C++ 的接口。

模型重新训练后可导出：

```powershell
& $gpuPython .\scripts\export_detection_onnx.py --weights models/drone_detection.pt
& $gpuPython .\scripts\check_detection_onnx.py
```

导出产生 ONNX 和同名 JSON 哈希清单；当前类映射固定模型 class 0=无人机，标准类别为 6，必须保持训练数据类别一致。改变输入尺寸或相机数量必须同步重新导出、修改配置和校验，当前控制台只接受 1536 推理尺寸。YOLO26 end-to-end 导出没有额外 NMS，历史 `iou` 配置不作为此次 ONNX 调参手段。

## 实验口径与结果

主指标是**每次目标进入完整 10 m 前半球后 0.5 s 内实际成功报警的比例**。独立评估读取真实障碍机数量与位置，直接评价 `/drone1/warning/fusion` 的报警，另行保留检测率，避免把检测率当作报警率。

实时“真实数量/正确报警数量”按有效时刻统计；“及时预警成功率”按交会事件统计，两者分母不同。真值不全或姿态缺失的历元标记无效并报告覆盖率；没有目标时比率为 N/A，不计作 100%。退出滞回为 2 m / 1 s，证据有效期 0.3 s、位置匹配门限 3 m，所有试验保持同一口径。

第一次 32 机试验报告 `reports/perception_dual_front_onnx/result.json`：51 次有效交会，及时报警 27 次、迟到 11 次、漏报 13 次，**52.94%，尚未达到 90%**。检测及时率 58.82%，逐历元检测召回 45.58%，有效计分覆盖率 94.60%。几何 FOV 计数等于全部计分真值计数，说明该轮失败不能主要归因于几何盲区；仍有可见无人机未出框，以及部分融合结果过期。

保存预览的离线阈值诊断只能辅助检查，图像已降采样和 JPEG 压缩，不能代替原始图像精度验收。降低阈值能增加部分框，但不能恢复所有明显漏检；当前未为追求数字而放宽阈值或延长证据窗口。

回归测试：97 项通过；Foxglove UI 测试通过；ROS 包构建通过。报告见 `reports/dual_tests.log`、`reports/dual_final_ui.log`、`reports/dual_final_build.log`。ONNX 双图离线基准完整处理约 13.21 ms，对比 PT 20.31 ms，约 1.54 倍；实际 UE+32机负载下不能直接将此数值换算成发布频率。

第二轮时序优化验证结果另存 `reports/perception_dual_front_onnx_v2/result.json`：54 次有效交会中 37 次及时、7 次迟到、10 次漏报，**68.52%，仍未达到 90%**。逐历元检测召回 58.31%，有效计分覆盖率 92.55%。两轮轨迹受闭环避障影响，不是逐帧相同回放，不能把差值全部归因于某一项优化。两轮报告均保留。

第二轮原始相机归档仍有 35 次 completion reorder 超时，归档平均约 17.16 Hz；雷达、推理和归档的频率应分别检查。控制推理直接读取共享内存最新图像，不等待归档排序，但不能据此声称本次所有传感器已稳定满足目标频率。共享内存读取和完整算法负载下仍需继续优化 UE 回读及调度。

复现实验（关闭当前管理器拥有的 UE/ROS 后，在已激活 ROS 终端中）：

```powershell
. .\scripts\activate.ps1
$env:PYTHONPATH="$PWD\.visualization_deps;$PWD\.algorithm_deps;$env:PYTHONPATH"
python .\tests\probe_perception_recall.py --seconds 120 --label dual_front_recheck --dual-front
```

该试验独立 ROS 域 223，32 架障碍机分两层高度从 20 m 飞到 7 m 并循环，本机执行 10 m 悬停任务和实时避障；结束后清理自有进程并恢复原会话原点，校验用户配置未改写。不同实验使用不同 label 保存结果。建议后续分别测试 4/16/32 机、左右侧向进入、迎面接近、不同高度、遮挡及空场，至少重复多个独立场景，保留全部成功和失败事件。

当前仍需针对新的 120° 广角和俯视角收集训练数据、独立验证集，改进视觉漏检后再次验收。协议中的预警准确率、漏警率、误警率应使用预先冻结的风险/安全事件标签和完整混淆矩阵分别验收；本轮半球及时预警率不等同于这些合同指标全部达标，也不能从单次匹配精确率推导“误警率小于 1%”。
