# UE 窗口中的障碍机检测框

`evtol_targeting.ue_detection_boxes` 是独立的只读可视化节点。订阅
`/drone1/perception/tracks`，筛选 `class_id: 6`（无人机），用 AirSim
`simPlotLineList` 在 UE 世界中画绿色 12 边立体框，并标注跟踪 ID 和状态。
障碍机位置来自视觉与雷达定位、跟踪结果，不从障碍机真值话题生成。

## 启动

在 `evtol_project_ros2` 根目录编译（已部署机器仅需修改代码后重编译）：

```powershell
. .\scripts\activate.ps1
colcon build --merge-install --executor sequential --packages-select evtol_targeting evtol_bringup
. .\install\local_setup.ps1
```

新开终端启动有窗口的 UE：

```powershell
.\scripts\start_ue.ps1 -Visible
```

等待 UE 加载完成后，按原流程启动实验：

```powershell
.\scripts\start_experiment.ps1
```

实验 launch 默认自动启动框节点；上述命令维持原有影子模式。
需要实际执行航点任务仍使用原来的 `-Execute`、`-Actors` 等参数，绘图不需要它们。
已有实验正在运行但还没有框节点时，可另开终端只启动绘图：

```powershell
.\scripts\start_ue_boxes.ps1
```

不要同时启动两份绘图节点；实例锁会拒绝重复启动。
若 UE 已手动启动且有可见游戏窗口，直接启动实验/绘图节点即可，无需重启 UE。
不带 `-Visible` 的 `start_ue.ps1` 仍保持原来的后台渲染行为。
`-Visible -WindowWidth 1280 -WindowHeight 720` 仅设置窗口大小，不降低传感器图像分辨率。
配置的 `ViewMode` 应为 `SpringArmChase` 等显示模式，不能为 `NoDisplay`。

## 配置

`config/targeting.yaml` 的 `ue_detection_boxes` 字段控制显示：

```yaml
ue_detection_boxes:
  enabled: true
  input_topic: perception/tracks
  class_ids: [6]
  rate_hz: 10.0
  color_rgba: [0.0, 1.0, 0.0, 1.0]
  line_thickness: 3.0
  line_lifetime_s: 0.2
  min_size_enu_m: [1.0, 1.0, 0.5]
  padding_m: 0.15
  show_labels: true
  label_scale: 1.0
  message_timeout_s: 0.5
  measurement_timeout_s: 1.0
  clock_timeout_s: 0.5
  max_boxes: 16
  rpc_timeout_s: 0.5
  source_coordinates: vehicle_local
  ego_actor_name: ''
  global_ned_offset_m: null
```

相对话题名自动加配置的 namespace；也支持完整绝对话题名。
输入必须是 `evtol_perception_interfaces/TargetArray`，frame 为 `map_enu`，
session 与 `/simulation/reference_origin` 一致。可改订阅
`perception/targets_measurements`，显示未经跟踪预测的定位测量。
默认包含 tentative/confirmed/predicted 轨迹，最多预测到跟踪器的存活时间上限。

尺寸取 `max(Target.size, min_size_enu_m) + 2 * padding_m`。
当前定位结果仅包含雷达可见点范围，没有可靠目标姿态，因此这是沿 ENU 轴对齐的
AABB 检测框，并非完整机体尺寸或随目标姿态旋转的 OBB。

## 坐标与迁移

输入是以本次主机起点为原点的 ENU 米坐标。节点先加回
`ReferenceOrigin.source_origin_enu`，再转换轴向：`N=ENU.y, E=ENU.x, D=-ENU.z`。
AirSim 绘图需要全局 NED，而当前采集真值是车辆局部 NED，因此还要加局部到全局偏移。
节点只读取主机 `simGetVehiclePose` 和 `simGetObjectPose` 校准这个偏移，
不能简单用 settings 的 X/Y/Z 代替，因为 UE pawn 带有地面偏移。
校准用前后两次主机局部位置夹取全局采样；位移超过 5 cm 时重试。
默认开始实验前主机静止即可自动完成，移动过快时诊断提示并等待重试。
主机 actor 名默认等于 AirSim vehicle 名；自定义场景可用 `ego_actor_name` 指定。
也可显式配置已知的 `global_ned_offset_m: [N, E, D]`。
只有采集源本身已是全局 NED 转成的 ENU 时才设置 `source_coordinates: global_ned`。
更换场景/出生点必须重新校准或更新显式偏移，不能照搬本机校准值。

无需修改或重编译 UE 插件，无新增第三方 Python 依赖；复制项目、编译 Python 包并按
原项目迁移文档配置 `machine.local.json` 即可。绘制使用 UE debug lines，保留正常深度遮挡；
目标在镜头外或被物体遮挡时，框也可能不可见。

## 时效、诊断与排查

绘图使用独立 RPC 工作线程，10 Hz 批量发送所有目标，每次最多 16 个框。
不发布飞行指令、不解锁、不移动物体，也不调用清除所有持久标记的全局接口。
无新检测、空数组、检测测量过期、时钟停更、时钟倒退或会话切换时停止绘图；
此前的框按 0.2 秒 UE 世界时间寿命消失。UE 完全暂停时，寿命计时也暂停，恢复后过期。
停止绘图节点使用其终端 `Ctrl+C`，不影响实验其他节点。

```powershell
. .\scripts\activate.ps1
ros2 topic hz /drone1/perception/tracks
ros2 topic echo /simulation/reference_origin --once
ros2 topic echo /diagnostics
```

`/diagnostics` 中 `/drone1/ue_detection_boxes` 报告 `draw_calls`、`boxes`、
`errors`、`last_error`、`input_topic` 与会话；`waiting_or_no_fresh_detection`
表示当前没有可绘制的新鲜输入，`drawing` 表示有可绘制目标，`rpc_error` 表示连接或校准异常。
有 tracks 消息不代表一定有无人机目标，需要检查数组中是否存在 class 6。
如果只有后台 UE，应停止原 UE 再加 `-Visible` 启动，不能同时运行两份共享内存写入者。

## 验证

自动测试覆盖非零 ENU 原点/全局偏移、12 边几何、类别筛选、最小尺寸、批量绘图、
无效坐标/会话/时间戳、数据和时钟超时、空检测清除、原点校准失败重试，
以及 RPC 调用只包含主机坐标校准与绘图。
窗口验证可用隔离 ROS 域的测试检测消息检查完整订阅到 UE 绘图链路；
测试框仅用于显示验证，不是模型精度或实际检测结果的证据。

2026-11-10 本机验证：相关功能包编译成功，完整 89 项自动测试通过。
隔离 ROS 域 224 的人工检测消息触发 19 次真实 UE 绘图调用，空检测后停止绘制。
当前场景校准偏移为 [0, 0, 0.1464958191] 米，不应复制为其他场景的固定参数。
结果见 reports/ue_boxes_live.json，测试日志见 reports/ue_boxes_unit.err.log。
本机 computer-use 原生管道不可用，未完成 UE 窗口截图或目视验收；
接口调用成功不等于已经验证窗口显示效果。