# Windows Fast DDS 崩溃排查与绕过方案

日期：2026-11-10。适用项目：`evtol_project_ros2`，Windows ROS 2 Jazzy。

## 已确认的故障

最近实验在 GNSS 节点首先出现 `3221225477`（十六进制 `0xC0000005`），随后 launch 终止全部节点。Windows Application 日志 Event ID 1000 确认故障模块为 `C:\pixi_ws\ros2-windows\bin\fastrtps-2.14.dll`，版本 2.14.5.0，偏移 `0x1af616`。2026-11-08 相机节点的崩溃也记录在同一个 DLL 和偏移。因此不能将这些日志直接归因于相机渲染负载或 GNSS 模型代码。

这是原生 Fast DDS 层的访问冲突，具体内部触发机制尚未通过转储分析确定。Python 普通异常捕获无法恢复该进程。缺少导航雷达、UE 黑屏等是之前的不同问题，不能混为一谈。`SIGINT not supported ... SIGTERM` 是 Windows 的连锁退出日志；日志目录符号链接权限警告不导致本次崩溃。

## 项目修改

- 默认 DDS XML 为 `src/evtol_bringup/config/fastdds_udp.xml`，只使用 UDPv4，关闭 DDS 层 SHM。
- UE → 采集节点的图像和双雷达共享内存协议、尺寸、频率、时间戳保持原有配置。
- 原 `fastdds.xml` 保留，不覆盖外部自定义 XML。激活时打印实际 DDS XML 和 ROS 域。
- 两个 launch 的节点输出改为 `both`，后续 traceback 与 native faulthandler 输出同时进入终端及 ROS 日志目录。
- `scripts/diagnose_dds.py` 启动全部采集和算法节点，网关固定为 shadow，不启用 actors，不解锁、不起飞、不重置原点；分别保存每个子进程日志及订阅统计，结束时只清理它创建的进程树。运行前先停止已有实验，避免传感器互斥锁冲突。

选择仅影响当前终端及其子进程。已有 ROS 节点不会自动切换传输，需重新启动；UE 不需因 DDS 修改而重启。若已有自定义 XML，使用 `. .\scripts\activate.ps1 -DdsTransport udp` 显式切换，后续启动脚本会保留该终端的选择。

## 本机验证

证据：`reports/dds_udp_domain0.json`，各节点日志为 `reports/dds_udp_domain0_*.log`。保留用户已经运行的 UE，使用 ROS 域 0、原双路 2048×1536 配置。14 个子进程全部完成 20 秒预热和 60 秒测量，`node_exits: {}`，结果 `PASS_STABILITY`。

| 实际接收话题 | 60 秒测得频率 |
| --- | --- |
| RGB BGR8，9437184 字节/帧 | 19.318 Hz |
| 语义 BGR8，同尺寸 | 19.266 Hz |
| 前向雷达 | 19.904 Hz |
| 导航雷达 | 19.921 Hz |
| IMU | 95.045 Hz |
| GNSS Fix / 原始观测 | 10.000 / 10.000 Hz |
| 5G-A | 9.933 Hz |
| ENU 自身位姿 | 19.919 Hz |
| 视觉语义推理结果 | 16.674 Hz |

该结果只证明当前机器、当前静止场景、该窗口内的稳定性和数据送达，不能证明原生缺陷已根治、长期无崩溃或所有传感器频率达标。例如原 RGB/语义 20 Hz 的 97.5% 下限为 19.5 Hz，IMU 100 Hz 的下限为 97.5 Hz，本次仍未通过这些严格频率指标。没有减少图像尺寸或伪造时间戳。本次不执行飞行，之前的绕障实测记录另见三维航点文档。

## 使用

已经启动 UE 且 AirSim 41451 正在监听时，不要重复执行 `start_ue.ps1`。ROS 退出不会自动关闭 UE，`AirSim port 41451 is already in use` 是防止重复启动的保护提示。

```powershell
cd D:\ZAD_Files\evtol_files\evtol_project_ros2
. .\scripts\activate.ps1 -DdsTransport udp
# 可选：先验证；需停止已有实验。此命令结束后清理自己的 shadow 节点。
python scripts\diagnose_dds.py --domain 0 --seconds 60
# 无人机在地面且新实验需要重建初始原点时：
.\scripts\start_experiment.ps1 -NewSession -Execute
```

Foxglove、rosbag、命令行查询终端均重新执行 `. .\scripts\activate.ps1 -DdsTransport udp`，并使用相同 `ROS_DOMAIN_ID`。脚本默认域 0；使用其他域时显式传入 `--domain`。

如需对比旧 DDS SHM 配置，在停止所有 ROS 节点后执行 `. .\scripts\activate.ps1 -DdsTransport shm`。该配置在本机发生过原生崩溃，不作为推荐运行方式；恢复 UDP 使用 `-DdsTransport udp`。

出现新崩溃时，先找第一个退出的节点，读取新的 ROS 日志目录及该子进程 traceback，再查看 Windows 应用事件；不要只依据最后一屏 SIGTERM 推断原因。UDP 仍崩溃时需进一步分析转储或验证兼容的中间件升级方案。
