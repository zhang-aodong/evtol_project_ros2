# 运行时漏检排查与修复（2026-11-19）

## 验收口径

主指标：每次障碍机进入以本机为中心、半径 20 m 的完整三维球后，在 0.5 s 内得到正确检测的事件比例。保持原有 0.3 s 真实测量证据窗口、3 m 几何关联门限；真值不进入视觉、融合或避障算法。分母包含全部方向的障碍机，不因相机遮挡、朝向或高度而删减。另报告逐时刻检出率、精确率、有效真值覆盖率、碰撞和飞行状态。

有限场景达到 95% 不等于任意地图、天气、遮挡和速度均保证 95%。多次进入来自相同障碍机，事件存在相关性。统计报告保留失败测试，不使用峰值或选取最好的一帧进行验收。

## 已确认的原因

1. 原有共同前向视场之外的目标占绝大多数，全向 20 m 验收与单个前向相机不匹配。
2. 原模型侧面、俯视小无人机训练不足。图像中看得到，不代表分割有 class 6；增大视场也会降低每架目标的像素数。
3. 旋转雷达射线稀疏且固定角网格，小目标可能长期落在网格间隙。
4. 等待雷达/位姿时，单槽待处理图像被后续图像覆盖；ROI 扫描时间晚于曝光时间，又被误用作下一张图所需原始扫描的下界。
5. 目标在曝光与雷达采集之间移动，单纯补偿本机姿态不能补偿目标运动，严格旧像素匹配丢失深度。
6. 多次 RPC 获取无原生时间戳的位姿、逐架真值，在负载下出现过宽采样区间；IMU 落后历史队列后也缺少自行恢复能力。
7. 新图像回退到较早的原始雷达深度时，跟踪器因时间倒退清空身份及速度估计；融合现在只输出时间递增的测量。
8. 起飞低速同时限制了紧急避让候选，附近行人的不确定性包络可能挡住全部候选。紧急脱离现在可以使用既有限速内的速度，并保留真实加速度、响应延迟、原始点云和起飞禁止向下运动的约束；没有安全候选仍停止。
9. 直接复用上一帧视觉区域预取射线，在本机倾斜转弯时可能错过新区域；对膨胀区域均匀抽样还可能跳过小目标本体。默认根据实测本机姿态反向补偿光学射线，并优先覆盖视觉组件内的细分光学射线，再分配周边运动搜索射线。串行等待当前帧 ROI 的对照测试延迟更大，未采用。

## 最终数据链路

UE 六方向 RGB → 共享内存 → GPU YOLO26 语义分割 → 无损 class-ID PNG ROS 话题 → 多视图融合 → 原测量/跟踪话题 → 现有避障。

NavigationLidar 继续经共享内存提供真实完整扫描；小目标由视觉 mask 请求额外有预算限制的 ROI 真实射线。UE 返回射线 first-hit XYZ 和采集时间，不返回目标身份或真值坐标。最多 4096 条射线/周期，按真实位姿转 ENU，再与视觉类别融合。原前向 2K 相机、原始雷达和安全点云链路保留。

匹配的像素搜索范围受时间差、最大目标速度、实际深度和最大像素数限制；仍要求真实视觉类别组件、真实深度点和三维聚类。不能将无视觉分类的雷达簇直接计为正确检出的无人机。超过时效、没有位姿包围或没有深度均显式丢弃并报告。

新增原生本机状态接口只提供本机位置、姿态、速度；它实现当前用户指定的“暂用真值本机定位”。障碍机真值批量接口只在独立评估节点使用。后续融合定位可通过已有 state.source=fused 接口替换本机状态。

## 配置和可视化

配置入口：`config/targeting.yaml` 的 `surround`。可设置 enabled、weights、width/height、horizontal_fov_deg、imgsz、max_hz、max_age_s、min_mask_pixels、min_lidar_points、scan_window_s、roi_lidar、roi_search_pixels、max_target_speed_mps、max_motion_pixels。不要通过增加 max_age_s 或实验 evidence_s 美化检出率。

环视模型：`models/surround_semantic.pt`，类别表和 SHA256 在同目录 JSON。六路 768×768 RGB，100° 视场，推理上限 6 Hz；融合使用保留目标类别的 2×栅格降采样，减少几何计算而保留小目标。原 `models/best.pt` 保留；`front_diagnostic_inference: false` 默认关闭重复前向 GPU 推理，前向原始数据仍发布。需要单独比较前向分割时可显式开启该诊断选项，并重新检查实时性。

模型环境通过 `config/machine.local.json` 的 ModelPython 配置；CUDA 推理使用 float16，CPU 回退会明确报告，实时性能需重新验收。`roi_prefetch: true` 默认将上一帧视觉区域经实测姿态旋转补偿，曝光后立即请求真实射线，与当前 GPU 推理并行；缺少近期真实姿态时跳过预取，回到当前帧请求。当前帧没有视觉分类证据时，射线本身不会产生无人机目标。射线坐标均来自真实几何测量，无真值身份或目标位置输入。

本机状态和独立障碍机真值分别以 50 Hz 采样，计分仍为 20 Hz。评估订阅队列保留 64 个真实样本以承受短暂回调阻塞；仍使用不超过 0.15 s 的真值插值间隔，不外推，也不增加检测证据保留时间。

Foxglove 可添加 Image 面板查看 `/drone1/perception/surround/rgb/compressed` 和 `/drone1/perception/surround/semantic_preview/compressed`。3D 面板固定坐标系 `map_enu`，查看 `/drone1/perception/markers`、`/drone1/perception/lidar/points` 及六路 `/drone1/perception/roi_lidar/surround_<方向>/points`。控制台继续显示 `/evaluation/detection_metrics` 的实时数量、检出率及及时事件成功率。Raw Messages 查看 `/diagnostics`，关注 target_localizer 的丢帧原因、image_to_output_ms、roi_lidar_points、per_view、compute_ms 和 request_compute_ms。

## 部署与迁移

1. 关闭本项目 UE、仿真 ROS 和旧控制台管理器。
2. 在 PowerShell 项目根目录执行 `scripts/activate.ps1`，按项目既有构建流程编译 ROS 包。
3. 执行 `scripts/ue_sensor_support.ps1 -PluginDirectory '<迁移后 UE 项目>/Plugins/AirSim' -Apply`，随后 `scripts/build_ue.ps1`。UE 4.27 项目必须重新编译，单拷 Python 无法启用新增原生接口。
4. `config/machine.local.json` 设置目标机器的 UnrealEditor、UnrealProject、ModelPython 等路径；复制模型及 semantic_runtime，并安装原有 ROS/Python/CUDA 依赖。
5. 重新启动原 `scripts/start_console.ps1`，刷新 Foxglove 后从面板启动 UE 和仿真。旧管理器已加载的 Python 模块不会随文件更新自动重载。

控制台自动生成相匹配的 UE 相机参数和采集配置。若手动启动 UE，须使用本次生成的 airsim.json；旧 settings 没有六方向相机时会明确报错，不能用空图代替。

## 可重复验收

激活 ROS 环境并安装项目后，运行 `python tests/probe_perception_recall.py --seconds 150 --label my_acceptance --radial-holdout`。该脚本使用独立 ROS domain 223，自动启动和清理自己创建的 UE/ROS，保留用户配置及原点文件；运行前应停下其他使用同一 AirSim 端口的仿真。

另去掉 `--radial-holdout` 验证用户当前路线。结果位于 `reports/perception_<label>/result.json`、telemetry.json 和对应 `data/console_runs/ros_*/evaluation/range_*`。原始 metrics、matches、encounters 全部保留。主指标必须结合实际升空、至少 20 个完成事件、有效真值覆盖率和精确率一起检查，样本不足不判通过。

本次离线训练 mask 与 RGB 严格同采集时间，训练/验证采用连续时间块划分。最终验收改变航向、速度、初始半径和高度，不将训练采集运行计为独立验收。


## 本轮真实验收结果

**结论：尚未稳定达到 95% 验收要求。** 本次交付包含已部署的真实修复、原始日志与可重复验收脚本；不能将单次较高结果宣传为整体通过。

| 测试 | 及时成功/完成事件 | 及时率 | 时刻召回 | 精确率 | 真值覆盖 | 说明 |
|---|---:|---:|---:|---:|---:|---|
| surround_final_radial_v2 | 124/128 | 96.88% | 85.96% | 99.98% | 95.11% | 历史测试，本机大部分在地面，不作为有效验收 |
| surround_prefetch_holdout | 116/119 | 97.48% | 82.79% | 99.76% | 90.61% | 历史较高及时率，但真值覆盖不足、本机未到目标高度 |
| surround_monotonic_original | 1/1 | 100.00% | 92.05% | 77.88% | 98.25% | 用户原航线可升空；仅1次事件，样本不足 |
| surround_escape_holdout | 128/148 | 86.49% | 81.08% | 99.80% | 94.78% | 起飞脱离修复后，仍有迟检 |
| surround_targeted_holdout | 109/152 | 71.71% | 50.74% | 99.84% | 96.11% | 串行测距对照，不采用 |
| surround_rotated_holdout | 131/151 | 86.75% | 69.16% | 99.83% | 96.29% | 姿态补偿版，仍有迟检 |
| surround_optimized_holdout | 126/146 | 86.30% | 67.76% | 99.97% | 93.79% | 最终代码，32机，28→9m，方位偏移7° |
| surround_optimized_verification | 123/144 | 85.42% | 73.93% | 99.91% | 95.82% | 最终代码，32机，30→10m，方位偏移19° |

最终代码已通过 **84 项相关回归测试**，包括六方向覆盖、真实视觉与深度双重证据、射线预算、姿态补偿、时间递增、起飞紧急避让、地面约束、历史 IMU 恢复和控制台配置。构建与测试日志位于 `reports/perception_optimized_*`。

## 仍未解决的限制及下一步

1. 密集小目标仍存在首次分割缺失、深度未命中及融合过期；当前图像到输出可能超过 300 ms，使 0.5 s 首次发现窗口只剩较少的有效机会。单纯修改置信度、延长历史或扩大关联门限不能作为解决方案。
2. 原训练补采图为 512×512，训练使用 imgsz=768；插值训练不能补出原图不存在的目标细节。环视实际采集为 768×768。下一步应补采原生 768/更高像素的侧向、俯仰、倾斜、遮挡和远距小无人机，按独立场景划分训练/验证/最终测试，比较事件指标后再替换模型。离线 class-6 像素召回约90.3%，不能当作事件95%的证据。
3. 真值全部32架同时出现采样缺口，表明负载下仍有整批消息/本机状态中断；最终报告保留缺口，不将未知帧标为检测成功。下一步可将原生kinematics写入带序号的共享内存环形历史，以恢复真实采样，继续保持0.15 s最大插值间隔与0.4 s评估缓冲。
4. 相机原20 Hz链路本轮平均约18–19 Hz，雷达约19.8 Hz、IMU约99.5 Hz；相机偶见重排窗口超限。未宣称全部传感器频率严格达标。

## 文件与恢复

- 中文说明：`docs/perception_recall_20261119.md`。全部成功与失败测试汇总：`docs/perception_recall_acceptance_20261119.json`。
- 最终原始结果：`reports/perception_surround_optimized_holdout/result.json`、`reports/perception_surround_optimized_verification/result.json`；各文件的 run 字段指向独立评估原始 metrics/matches/encounters。
- 源码及模型 SHA256：`docs/perception_recall_deployment_20261119.json`。旧文件备份：`backups/perception_recall_20261119_140958`。
- 用户的 console.local.json、原航点和 models/best.pt 保留；测试 finally 恢复 session_origin，并验证用户配置 hash。每次结果的 owned_remaining 应为空。
- 已完成本机 UE 插件补丁与重新编译，复制到另一台机器仍需按前文步骤应用并编译插件；本轮新增原生接口是项目依赖。

日常启动：运行 `./scripts/start_console.ps1 -NoBrowser -BridgePort 8766`，Foxglove 连接 `ws://127.0.0.1:8766`，由现有控制台按钮分别启动 UE 和 ROS。启动新的管理器前必须先关闭已有管理器，避免端口和控制权冲突。
