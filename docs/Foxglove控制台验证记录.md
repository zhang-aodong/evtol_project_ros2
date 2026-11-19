# Foxglove 控制台验证记录

日期：2026-11-11。环境：Windows 11、ROS 2 Jazzy / Pixi、UE 4.27、项目现有 AirSim 插件。

| 检查 | 结果 | 证据 |
|---|---|---|
| targeting / safety / visualization / console 构建 | PASS | `reports/console_build.log`、`reports/console_build_final.log` |
| Python 回归 | 104 项通过 | `reports/console_tests.err.log` |
| 表格编辑、障碍机数量、剖面绘图、点击／拖动、保存与启动提交 | PASS | `foxglove/evtol-console/tests/ui.mjs` |
| TypeScript 类型检查与扩展打包 | PASS | `.foxe` 与 `dist/extension.js` 已生成 |
| 一键启动入口、HTTP、面板自动安装 | PASS | HTTP 200；安装的 extension.js 与工程构建 SHA256 一致 |
| 真实 Foxglove JSON 服务 status / preview | PASS | `tests/transport.mjs`，非模拟 context |
| 后台 UE、ROS 观察栈、ENU 遥测、停止／关闭 | PASS | `reports/console_live_background.json` |
| 可见 UE、ROS 观察栈、ENU 遥测、停止／关闭 | PASS | `reports/console_live_visible.json` |
| Ctrl+C 清理预览与管理程序 | PASS | 修复 PowerShell 中断时的预览残留，改由 manager 持有并清理 |
| 低速剖面完整实飞 | FAIL | `reports/console_profile.err.log`，起飞阶段控制 RPC 超时 |
| Foxglove 和 UE 桌面目视验收 | 未完成 | 当前 computer-use 窗口通道不可用 |

两种生命周期验证固定主机 `execute=False`、场景控制 `actors=False`，未进行飞行。
低速剖面测试单独打开了执行，使用 0 障碍机、3 m 上升、4 m 前进、下降落地的配置。
第一次在模型加载期起飞后出现超时；增加成功推理等待后仍在起飞阶段超时。
控制器报告 `phase=failed`、`reason=timed out`，未记录 PASS，也没有放宽数据时效或制动约束。

因此当前可以验收控制台、任务生成、进程生命周期及接口，不能验收新增剖面的完整飞行性能。
参考速度是航段速度目标，水平／垂直转角减速、落地接管和避障都会改变实际曲线。
图上 nominal 路径处于同一竖直平面；安全规划器可在避障时离开该平面。

入口与使用顺序见 [使用教程](Foxglove仿真控制台使用教程.md)。
