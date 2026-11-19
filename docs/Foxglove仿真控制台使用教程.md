# Foxglove 仿真控制台使用教程

详细日志、主机／障碍机状态和独立强制结束按钮已更新，见 [控制台详细日志与强制结束](控制台详细日志与强制结束.md)。

2026-11-14 的多机出生碰撞、起飞失败和控制 RPC 超时修复，见
[多机起飞失败修复与验证](多机起飞失败修复20261114.md)。障碍机现支持首航点空中布设与真实地面起飞两种初始化方式。

## 1. 已实现的内容

控制台提供“运行总览、UE 启动、障碍机、本机飞行、监控 / 日志”五个页面。
同一套界面既可以作为 Foxglove 扩展面板使用，也可以在本机浏览器打开。

- 一个 PowerShell 入口启动常驻管理程序、Foxglove 数据桥接和轻量预览节点。
- UE 与 ROS 实验分别启动、分别停止；UE 支持后台渲染和可见窗口。
- 设置 UE Editor 程序、工程和基础 AirSim 配置路径。
- 编辑障碍机数量、名称、出生位置、速度、航点与循环次数。
- 本机支持 ENU 航点 + 统一速度、速度—高度剖面两种模式。
- 剖面支持表格输入、增删排序、点击绘图添加点、拖动点、即时折线和航点预览。
- 实际飞行的速度—高度曲线叠加显示，并展示飞行阶段、子进程和诊断信息。

所有任务仍由现有 ROS 2 安全规划器和唯一主机控制网关执行。传感器节点继续负责
采集与发布，不承担界面、任务配置或进程管理。

## 2. 第一次配置与编译

本机原有 ROS 2 Jazzy / Pixi、UE 4.27、AirSim 插件、共享内存采集和模型环境继续沿用。
先按原项目环境文档配置 `config/machine.local.json`，确认原实验可以运行。
本控制台源码及依赖为项目内的 Python 包，不要求安装 Qt。

第一次取得源码后，在项目根目录运行：

```powershell
cd D:\ZAD_Files\evtol_files\evtol_project_ros2
. .\scripts\activate.ps1
colcon build --merge-install --executor sequential --packages-select evtol_targeting evtol_safety evtol_visualization evtol_console
. .\install\local_setup.ps1
```

当前交付已经编译。更换电脑时先完整编译已有消息和其他功能包（可使用原来的
`scripts/build.ps1`），再按上述方式构建控制台。

已经提供编译好的 Foxglove 扩展：

```text
foxglove/evtol-console/evtol.evtol-console-0.1.0.foxe
foxglove/evtol-console/dist/extension.js
```

日常运行不需要 Node.js。只有修改界面源码后重新打包、或运行界面开发测试时需要 Node.js / npm：

```powershell
.\scripts\build_console_panel.ps1
```

该脚本用锁文件安装官方 npm 依赖并重新生成 `.foxe`。源代码入口是
`foxglove/evtol-console/src/index.ts`，界面代码是
`src/evtol_console/evtol_console/web/app.js`；Foxglove 与浏览器共享这份界面代码。

## 3. 日常只运行这一个脚本

```powershell
cd D:\ZAD_Files\evtol_files\evtol_project_ros2
.\scripts\start_console.ps1
```

脚本会自动：

1. 激活项目 ROS 2 环境。
2. 将本项目编译好的面板复制到当前用户的
   `%USERPROFILE%\.foxglove-studio\extensions\evtol.evtol-console-0.1.0`。
3. 启动管理程序、支持控制服务的 Foxglove 桥接和预览节点。
4. 等 HTTP 服务就绪后打开 `http://127.0.0.1:9040`，作为相同功能的本地控制界面。

此时 UE、采集、算法、飞行控制均未启动，等待界面操作。

不自动打开浏览器：

```powershell
.\scripts\start_console.ps1 -NoBrowser
```

端口或扩展安装的选项：

```powershell
.\scripts\start_console.ps1 -Port 9041 -BridgePort 8766 -SkipPanelInstall
```

默认桥接端口为 8765。如果已有旧的只读桥接占用端口，脚本会自动选取 8766..8799 的
空闲端口；**Foxglove 应连接终端打印的实际端口**。不会关闭已有桥接。
HTTP 管理端口已占用时会拒绝重复启动；不要对同一仿真运行两份管理程序。

此入口已经包含桥接，不需要再运行 `start_visualization.ps1 -Viewer foxglove`。
轻量预览常驻，但原始相机图像、激光雷达、IMU 等采集配置不因界面而降低规格。

## 4. 在 Foxglove 中打开完整面板

1. 启动或重新加载 Foxglove 桌面程序，让它识别刚安装的扩展。
2. 选择 **Open connection → Foxglove WebSocket**。
3. 输入终端打印的地址，例如 `ws://127.0.0.1:8765`。
4. **Add panel → EVTOL Simulation Console**。
5. 放大面板，或者把控制台放在一个 Tab 中，将 Image、3D、Plot 等放在另一个 Tab 中。
6. 保存 Foxglove 布局，下次继续使用同一布局即可。

网页版 Foxglove 可以将 `.foxe` 文件拖入可视化页面后添加面板。
当前 Foxglove 官方文档注明本地扩展安装可能需要 Developer seat，具体取决于版本和许可。
如果当前账户不能加载自定义扩展，直接打开本地控制台 URL；界面和后端完全相同，
同时仍可以用 Foxglove 内置面板看传感器数据。不能把“本地网页可用”误认为
“已经绕过 Foxglove 的扩展许可限制”。

使用较新的 Foxglove 客户端连接项目当前安装的 SDK。该 SDK 协商
`foxglove.sdk.v1` 系列协议；仅支持旧 `foxglove.websocket.v1` 的客户端会握手失败。

## 5. 设置 UE 并启动

打开“UE 启动”：

- **UE Editor 程序路径**：例如
  `D:\Software\Epic\UE_4.27\Engine\Binaries\Win64\UE4Editor.exe`。
- **UE 工程路径**：例如
  `D:\Software\UEproject\MyProject 4.27\MyProject 4.27\MyProject.uproject`。
- **AirSim 基础配置**：默认项目内的 `src/evtol_simulation/config/airsim.navigation_2k.json`。
- **运行方式**：可视化窗口或后台渲染。
- **窗口尺寸**：只影响 UE 主窗口，不改变采集相机分辨率。

先按需要编辑障碍机数量、出生位置，再点“保存全部参数”，最后点“启动 UE”。
总览会显示启动中，AirSim RPC 就绪后显示“已就绪”。UE 启动最长等待 120 秒；
启动失败会显示错误及日志路径。不会在按钮点击后直接假定启动成功。

后台模式使用 `-RenderOffscreen`，继续渲染相机与雷达采集所需场景。
可视化模式使用正常游戏窗口；基础配置如果为 `NoDisplay`，本次生成配置自动改为
`SpringArmChase`。管理程序需运行在当前登录用户会话，不要放到 Windows Session 0 服务中。

## 6. 设置障碍机

“障碍机”页面的数量决定本次 AirSim settings 中配置的障碍机数量，支持 0..32 架。
这只是配置上限，不承诺 32 架时仍达到此前少量车辆下验收的性能。

每架可以设置：

| 字段 | 含义 |
|---|---|
| 名称 | AirSim vehicle 名，不能与主机重名，也不能重复 |
| UE 出生位置 N/E/D | 相对于 UE 全局 NED 原点，米；D 向下为正 |
| ENU 航点 E/N/U | 相对于本次主机初始点；U 向上为正 |
| 速度 | 该障碍机运动速度 |
| 循环次数 | 0 持续循环；正整数为路径执行次数 |

障碍机不是主机，修改其路径不改变主机航点。
“初始化方式”默认使用首航点空中布设，稳定后等待本机起飞，再执行后续航线。
这只在实验初始化时放置飞机；若需要从地面真实起飞，选择地面模式，并确保出生点有地面支撑、没有场景碰撞。
单点路径到达后悬停；多点路径按循环次数执行。行人沿用原有 `config/actors.yaml` 的路径，
本版本可以开关行人控制，尚未增加行人逐点编辑表格。

首次生成的障碍机草稿使用明确的 ENU 示例航点，不把旧配置中的 FRD/NED 数值假冒为 ENU。
原 `actors.yaml` 不会被覆盖，请在界面确认新航点后保存。

## 7. 本机：航点模式

“本机飞行”选择“ENU 航点 + 飞行速度”。逐行设置 `[E,N,U]`，支持增删和上移排序。
例如：

```text
[0, 0, 10]
[0, 100, 10]
[0, 100, 0]
```

表示升到初始点上方 10 m，向北到 100 m，然后在该处落地。
飞行速度是期望巡航上限；接近航点、制动、转向和避障时允许降低。
末点为地面并启用“末点在地面时自动降落”时，由现有降落检查和网关执行落地流程。

## 8. 本机：速度—高度剖面

规则严格按此次需求实现：

1. 横轴为三维地速大小，单位 m/s；纵轴为 ENU 高度，单位 m。
2. 按输入顺序连接点，不能按速度大小排序。
3. 相邻点高度不同：仅改变高度，E/N 保持不变，生成垂直升降航段。
4. 相邻点等高：沿所设 ENU 航向水平前进，距离取终点行的“水平距离”，默认 100 m。
5. 航向 0° 为北，90° 为东。所有名义航点位于这个方向确定的同一竖直平面。
6. 起点、终点速度必须为 0；非零长度航段不能两端速度都为 0。

输入一个点立即更新图中的点和连线；可以在表格输入，也可启用“点击图中添加点”。
拖动蓝色点会同步改变速度和高度，表格跟随更新。编辑时会即时显示转换的 ENU 航点。
图中蓝线是参考剖面，绿线是实际飞行记录。

对应此前 300 m 起飞、水平 100 m、下降的例子：

| 点 | 速度 m/s | 高度 m | 水平距离 m | 航段 |
|---|---:|---:|---:|---|
| 1 | 0 | 0 | 忽略 | 起点 |
| 2 | 4 | 300 | 忽略 | 垂直上升 |
| 3 | 7 | 300 | 100 | 水平向前 |
| 4 | 7 | 200 | 忽略 | 垂直下降 100 m |
| 5 | 0 | 0 | 忽略 | 垂直减速下降至地面 |

航向 0° 时，转换得到：

```text
[0,   0,   0]
[0,   0, 300]
[0, 100, 300]
[0, 100, 200]
[0, 100,   0]
```

此示例应将总速度上限改为至少 7 m/s；保存时仍会检查当前激光雷达范围、制动能力和
延迟预算是否允许该速度，不能通过界面静默绕过安全限制。

航段内使用沿航段的实际位置进度计算速度参考：

```text
v(s)² = v_start² + (v_end² - v_start²) × s / L
a = (v_end² - v_start²) / (2L)
```

即名义上按距离进行匀加速 / 匀减速；不是根据墙上时间强推目标。
界面同时显示理想持续时间作为参考，不要求用户补充飞行时间。
加速度超出设置时会拒绝配置。

水平与垂直之间的直角转向不可能保持非零速度瞬时完成。因此当前实现会在转向点
减速、到达并稳定后进入下一段；同向共线且非零速度的中间点支持通过。
起飞、降落接管、控制误差和避障也会让实际曲线偏离理想折线。
**该功能是带速度参考的航点任务，不是保证每个转角严格命中速度的轨迹优化器。**
避障优先级高于剖面跟踪：必要时可离开名义平面绕行或停车，仍以航点到达和数据时效为约束。

地面起始的剖面必须先垂直爬升，且首个上升点不低于配置的安全起飞高度。
如果首点本身在空中，先按原来的起飞 / 航点流程到达该高度，再进入后续剖面。

## 9. 启动 ROS 实验

UE 显示“已就绪”后，在“运行总览”设置：

- **执行飞行控制**：默认关闭，主机只观察，不解锁。开启后启动 ROS 会执行主机任务。
- **启动障碍机 / 行人控制**：独立开关，开启后它们会按配置运动；主机观察模式不代表
  障碍机也停止运动。
- **评估**：发布目标真值并记录定位误差。
- **新 ENU 会话**：启动前将旧原点存入本次运行目录，下一份主机真值确定新原点。

保存后点击“启动 ROS 实验”。它会启动六组传感器、状态适配、轨迹、导航雷达、视觉、
目标定位、安全规划、主机网关和 UE 检测框；障碍机、评估按开关启动。
“进程运行中”不等于模型已经加载完毕：请同时看监控中的诊断和飞行阶段，
模型加载阶段可能需要原有的启动时间预算。
执行模式会等待视觉节点实际完成推理并稳定报告正常后，才启动能够解锁主机的控制网关；
等待超时会停止本次启动并保留日志，不会因按钮请求已接收而直接解锁。

如果 UE 是手动启动的，可以直接启动 ROS 来连接，但车辆名称和出生位置必须与界面一致。
外部 UE 不属于本控制台，关闭按钮不会终止它。

UE 运行中可以保存不影响 UE settings 的航点、速度和 ROS 开关；
改变车辆数量、出生位置、UE 路径或显示模式时需先关闭 UE。
ROS 运行中不能直接保存新的启动配置，需要先停止 ROS。正在编辑的草稿不会被每秒刷新的状态覆盖。

## 10. 停止与退出

1. 点击“停止 ROS”，管理程序先调用主机停止服务和场景演员清理服务，再结束自己启动的节点。
2. 障碍机如果配置 `land_on_stop` 会执行原有降落清理，可能需要等待。
3. ROS 停止后可单独“关闭 UE”，或者用“停止全部”。
4. “取消当前启动”可以取消 UE 等待或 ROS 启动中的操作。
5. 不再使用时，在入口终端按 `Ctrl+C`，管理程序中断等待并结束自有进程，不再等待逐架降落。

关闭等待过久时，点击窗口顶部“强制结束 ROS + UE”。该按钮在普通关闭执行期间仍可用，
跳过降落清理，保留管理程序和桥接。只处理当前控制台启动的进程；手动启动的外部 UE 不会被结束。

主机停止沿用现有控制器语义：取消任务、请求悬停并释放 API 控制；不把停止 ROS
描述为“已自动降落”。若主机尚在空中，应根据场景需要先完成降落任务。
异常清理的原因会记录在管理日志中。不要用按进程名全局结束 Python 的方式关闭控制台。

## 11. 文件、接口与迁移

```text
src/evtol_console/                 管理程序、配置校验、进程所有权、静态界面
src/evtol_targeting/flight_profile.py  纯剖面编译与速度包络
src/evtol_safety/profile.py        剖面任务与现有避障 / 航点接口
foxglove/evtol-console/            扩展入口、锁文件、编译产物、开发测试
config/console.local.json         本机保存的控制台配置（首次保存时生成）
data/console_runs/ue_*/           本次 UE settings、UE 日志
data/console_runs/ros_*/          本次 targeting / actors / console 快照与逐节点日志
```

基础 `targeting.yaml`、`actors.yaml`、`airsim.navigation_2k.json` 不被控制台覆盖。
换电脑时重新配置机器环境和 UE 路径，不要把旧机器绝对路径当作可迁移默认值。
运行快照含机器绝对路径，主要用于本次运行审计；跨机重跑应在新机器重新保存和生成快照。

Foxglove 原生 JSON 服务：`/simulation/console`，请求示例：

```json
{"action":"status","payload":{}}
```

支持 `status`、`preview`、`save`、`start_ue`、`stop_ue`、`start_ros`、`stop_ros`、
`stop_all`、`cancel`。启动 / 停止请求立即返回 operation_id，最终成败需要轮询状态，
不能把“已接受请求”当作“启动完成”。不接受任意 shell 命令。

此外提供真实 ROS 2 `std_srvs/Trigger` 服务：

```text
/simulation/console/start_ue
/simulation/console/stop_ue
/simulation/console/start_ros
/simulation/console/stop_ros
/simulation/console/stop_all
/simulation/console/cancel
```

ROS 2 状态话题 `/simulation/console/status` 为 `std_msgs/String`，内部 JSON 包含配置、
操作、进程、诊断和遥测；Foxglove 内的同名状态通道直接使用 JSON 编码。
HTTP 备用接口是本机 `/api/status` 和 `/api/command`。服务均只监听 127.0.0.1，
本版本不提供跨机器远程启动接口。

## 12. 常见问题与验证

- **面板不可见**：重新加载 Foxglove；检查本地扩展许可；或使用本机网页。
- **面板提示数据源不支持服务调用**：连接的是旧只读桥接或文件回放，应连接入口脚本打印的新桥接地址。
- **编辑了参数但按钮不能启动**：先保存；运行中的 ROS 需要先停止才能应用新配置。
- **更改数量后仍是旧车辆**：必须用控制台生成的 settings 重新启动 UE。
- **看不到 UE 窗口**：确认选择“可视化窗口”；后台模式不会弹出可见窗口。
- **实际速度与参考线不完全一致**：查看转向减速、降落、避障、传感器包络与控制状态。
- **启动失败**：查看“最近操作”的具体错误和监控页逐节点日志路径。

开发验证命令：

```powershell
. .\scripts\activate.ps1
python -m unittest discover -s tests -v
cd foxglove\evtol-console
npm ci
npm run typecheck
node tests\ui.mjs
cd ..\..
$env:PYTHONPATH="$PWD\.visualization_deps;$env:PYTHONPATH"
python tests\probe_console_live.py
# 以下会真实启动 UE 和 ROS，均固定为观察模式、不启动障碍机控制：
python tests\probe_console_live.py --live --background
python tests\probe_console_live.py --live
```

界面测试覆盖表格编辑、剖面绘图、点击添加、拖动点和服务请求；
通信验证使用真实 Foxglove WebSocket 服务请求。真实仿真验证结果保存在
`reports/console_live_background.json`、`reports/console_live_visible.json`。
当前自动化桌面通道不可用，尚未完成 Foxglove 窗口截图与目视排版验收；
不能把构建、DOM 测试、服务和进程验证冒充为桌面目视验证。

### 2026-11-11 验证结果与当前限制

- 4 个相关功能包构建通过；104 项 Python 回归通过；界面 DOM 编辑、绘图、拖动和提交测试通过。
- TypeScript 类型检查和 Foxglove 扩展打包通过；一键入口、面板自动安装及真实 WebSocket 服务调用通过。
- 后台和可见窗口两种 UE 模式的启动、ROS 观察栈、真实 ENU 遥测、停止 ROS、关闭 UE 均通过。
- Ctrl+C 退出后，管理程序统一清理预览子进程；测试 UE 和 ROS 已清理，原有外部桥接未关闭。
- **新增剖面的完整实飞闭环尚未通过。** 独立低速测试（上升 3 m、水平 4 m、下降）在起飞阶段
  返回 AirSim 控制 RPC `timed out`。增加真实推理就绪等待后仍能复现，同时日志出现雷达／状态超时。
  目前不能将剖面参考生成和单元测试通过等同于实际飞行验收通过，也不能声称已经验证高速剖面或新增障碍场景。
  不应通过关闭数据时效检查来强制完成该测试。

该问题的复现日志为 `reports/console_profile.out.log`、`reports/console_profile.err.log`，
逐节点日志位于其中对应的 `data/console_runs/ros_*/`。独立复现入口是
`python tests/probe_profile_flight.py`，**此命令会真实解锁并控制仿真主机**，仅在没有其他实验运行时执行。
它使用独立配置、结束后清理自己的 UE／ROS，并恢复原 ENU 原点文件，不覆盖用户航点。

完整验证记录见 [Foxglove 控制台验证记录](Foxglove控制台验证记录.md)。

官方参考：

- https://docs.foxglove.dev/docs/extensions/local-development
- https://docs.foxglove.dev/docs/extensions/extension-api
- https://docs.foxglove.dev/docs/visualization/panels/service-call
