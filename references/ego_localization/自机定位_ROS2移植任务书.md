# 自机多源融合定位接入 ROS 2 主框架

**给实施 AI 的任务**　请在 `zhang-aodong/evtol_project_ros2` 中实现自机定位。先读本文件及压缩包中的 `offline_reference/`，再核对当前检出的源码。完成下述第一版代码、启动配置、测试和验收说明。仓库核对版本为 [c4a5a8b](https://github.com/zhang-aodong/evtol_project_ros2/tree/c4a5a8ba578f9694a18efdeed0959e6ee8f31419)。如分支已更新，以实际发布端与订阅端为准。

## 1. 现有算法怎样用

原工程是 C++17/Ceres 离线融合器。它读一段传感器数据，构造 IMU 预积分、GNSS 和 5G-A 等观测因子，完成整段优化。交付的是修正过 GNSS 虚拟卫星建锚、支持不提供真值文件的 `truth_free_loader_trial` 源码副本。

| 原工程位置（相对 offline_reference） | 移植时用它核对什么 |
|---|---|
| `src/ceres_fusion.cpp`、`include/offline_fusion/imu_covariance.hpp` | IMU 噪声、零偏及协方差传播；检索 `FiveGRangeCost`，核对逐站测距残差与权重 |
| `src/initialization.cpp`、`src/dataset.cpp` | 旧版姿态初始化、NED 坐标、GNSS 建锚及 5G 字段。ROS 端以消息头和当前场景配置替代 CSV 时间轴与历史地理常数 |
| `src/vision.cpp`、`src/lidar.cpp` 与 `tests/` | 保存已有相机、点云前端和回归样例，供后续单独验证 |
| `evidence/`（压缩包根目录下） | 无真值文件验证及同一条旧航线的分阶段离线误差。D0 为原始仿真 5G，D1/D2 为不同级别的合成退化 |

第一版交付两种可切换的在线解：**IMU + GNSS fix 基线**、**IMU + GNSS fix + 逐基站 5G-A 测距**。建议沿用主框架 Python/NumPy 和 ament_python，用 15 维误差状态卡尔曼滤波器（ESKF）逐条处理观测。ESKF 和 fix 位置更新属于新增实现，旧 Ceres 解保留为离线对照。一次全段优化会利用未来观测，循环调用离线程序也无法据此验证在线延迟。

## 2. 接线和坐标

新建 `src/evtol_localization/`，节点命名 `ego_localization`。把接收 ROS 消息、转换坐标及发布结果的代码与可独立测试的估计器分开。沿用项目的 namespace 配置，下表以 drone1 为例。

| 方向 | 接口 | 用途 |
|---|---|---|
| 输入 | `/drone1/imu/data`，`sensor_msgs/msg/Imu`，目标 100 Hz | FLU 机体系角速度、加速度和 ENU 参考姿态 |
| 输入 | `/drone1/gnss/fix`，`sensor_msgs/msg/NavSatFix`，目标 10 Hz | 经纬高、有效状态及近似 ENU 位置协方差 |
| 输入 | `/drone1/five_ga/observations`，`evtol_sim_interfaces/msg/FiveGObservations`，目标 10 Hz | 每条链路给出基站坐标、测距、标准差、信号强度与有效标志 |
| 输入 | `/simulation/reference_origin`，`evtol_perception_interfaces/msg/ReferenceOrigin` | transient-local 订阅，接收已冻结的会话 ID 和原始 map 起点 |
| 输出 | `/drone1/localization/odometry`，`nav_msgs/msg/Odometry` | 时间取已处理的状态历元；父系 `map_enu`，子系 `drone1/base_link`；姿态表示 FLU 到 ENU 的旋转 |
| 输出 | 定位诊断 | 至少区分等待初始化、正常更新、仅惯性传播、状态过期，记录最近接收和实际采用的观测时间及拒绝原因 |

**先解决地理基准，再融合 5G。** 当前采集后端用 AirSim `getHomeGeoPoint` 获取原始地图的经纬高，`ReferenceOrigin` 只给出笛卡尔坐标。推荐由采集端一次发布并录制相同来源的地理基准（供晚订阅者获取），也可采用随场景归档且可核验的显式配置。定位节点只消费 ROS 话题或配置。缺少匹配基准时显示等待原因。

1. 用同一地理基准将 fix 通过 WGS‑84/ECEF 换到原始 map ENU，核对高度基准，再计算 `p_session = p_map - source_origin_enu`。对非零出生点做一次实测。会话轴始终是东、北、上；初始机头朝向不参与该平移。
2. 采集端发布的 `base_station_enu` 已经在原始 map ENU 中，每个站只减一次 `source_origin_enu`。用一个非零起点的几何样例检查平移前后预测距离相等。IMU 已完成 FRD→FLU 转换。
3. `Odometry.pose` 给机体原点的位置与姿态，`twist.linear = R_body_to_enu.T @ v_enu`，角速度用机体轴，协方差随相应坐标变换。主框架下游会自行把 twist 转回世界系。
4. 会话原点当前由状态适配器根据仿真首帧真值创建，这是共享坐标约定。滤波器不订阅真值、下游 `state/odometry` 或动态 `map_enu → base_link` TF，不用原点消息的初始真值姿态初始化。会话变化或时钟回退时清空状态、缓存与待处理更新；原点就绪后重新初始化。

## 3. 估计器的最低实现

- **初始化与传播。** 名义状态为位置、速度、姿态、陀螺和加速度计零偏。用有效 fix 初始化位置，静止条件满足时初始化速度及可辨识的零偏。首版允许用已发布且有效的仿真 IMU 姿态初始化航向，记录该依赖；姿态不可用且航向未定时报告未就绪。姿态初始化后用去偏角速度传播，用机体系比力旋转到 ENU 后加重力；静置测试应得到近零速度变化。按采样间隔离散化过程噪声，输出有效协方差。IMU 消息的零协方差表示未知，噪声由配置提供。
- **GNSS 更新。** 依据状态、有限性、协方差及归一化创新门限接受位置观测。配置测量方差下限。失锁时暂停 GNSS 更新；恢复时先检查创新并报告接受或拒绝。若协方差发散导致需要重新初始化，显式报告时间与位置跳变量。
- **5G-A 更新。** 每条有效链路使用 `measured_range_m`。在同一 ENU 中，预测距离为 `norm(p_antenna - p_anchor)`；按项目安装参数处理杆臂，默认零值应有配置依据。测距方差取 `max(range_sigma_m, sigma_floor_m)^2`，检查 `measurement_valid`、有限正距离、标准差及创新，并支持 `prs_rsrp_dbm` 门限。按 `gnb_id` 记录接受/拒绝。RTT 与由它得到的测距只计一次；`is_los` 属于仿真链路标签，若用于降权需配置并在报告中说明。单站只有一个距离约束，多站几何差时如实保留不确定度。
- **异步更新。** 统一使用 `header.stamp`。现有 IMU 配置带约 0.1 s 缓冲，原生 GPS 另有延迟。用有界历史状态、IMU 与观测缓存处理乱序，可回到观测历元更新后重传播至最新历元；重放时包含该区间已接受的其他观测。也可先采用有界延迟的按时排序并实测状态年龄。相同传感器、同一条样本只处理一次，超出时间窗明确丢弃并计数。内部校正不能让发布的状态时间倒退，也不能靠墙钟改写旧样本时间。传感器 QoS 与仓库发布端一致。

## 4. 接入主框架和验收

1. 在 `config/targeting.yaml` 增加定位开关和参数，在 `src/evtol_bringup/launch/target_localization.launch.py` 增加独立的自机定位进程，沿用 `target_experiment.launch.py` 的调用链。现有名为 `localizer` 的进程定位障碍目标，需继续运行。支持先旁路运行，即 `state.source: truth` 时也可并行产生估计轨迹；选 `fused` 时确保估计器已启动。保留原默认启动行为。
2. 在当前实验栈中，`state.fused_topic` 已指向 `/drone1/localization/odometry`。切换 `state.source: fused` 后，现有 `evtol_targeting.state_node` 统一转发 `/drone1/state/odometry` 并独占动态 TF；保持采集侧 `publish_truth_tf: false`。滤波器停止或超时后不得重发旧位姿维持有效状态。适配器的 `state/status` 主要按到达时间判断新鲜度，另要报告“仿真当前时刻减状态时间戳”。核对实际配置的 `state.timeout_s`（当前为 0.3 s）。
3. 增补现有录包白名单及 QoS，收录输入、地理基准、会话原点、融合输出和对照真值。提供只启动估计和评价节点的回放方式；使用录包时间轴，同名源和动态 TF 保持唯一。Foxglove 中叠加融合轨迹与 `/drone1/ground_truth/local_odometry`；旁路期间 `traveled_path` 仍取自主状态，不能将其当作融合结果。
4. 先做可重复的小测试：静止不漂移的理想输入、朝北飞行时轴和速度符号、非零会话原点下的 5G 距离、异常链路拒绝、乱序/重复消息、会话切换后重新初始化。再做旁路 AirSim 或真实采集 bag 验证。用同一包比较 GNSS+IMU 和增加 5G 的两组，分别在 GNSS 正常与人为屏蔽的 20 s 窗口运行；若沿用旧试验则采用相对起始时间 40–60 s。屏蔽只作用于估计器输入，保留真值供独立评价。
5. 在共同真值历元对齐估计，报告覆盖率、三维 RMSE、P95、中断末端误差和恢复时的校正跳变，以及状态频率、状态年龄 P95/最大值、观测采用数。评价使用会话 ENU 原始对齐，不再用真值拟合轨迹。确认切换 fused 后下游收到的确为融合输出、关闭估计器后状态超时、无双重 TF。在只有 mock 的机器上交付软件测试记录与可执行的 AirSim 验收命令，实测栏留待运行。旧离线 RMSE 不充当在线通过阈值。

**请交付**修改文件清单、Windows 构建与启动命令、truth 旁路和 fused 两种配置、测试结果及实际运行条件，并说明旧工程复用了哪些观测模型、哪些是新增在线实现。无需修改任务规划或障碍目标检测算法。

## 5. 后续移植顺序

如果项目要求复现原有 GNSS 观测级解算，将其作为单独模式。当前 `GnssObservation.msg` 有 PRN、伪距、相位、多普勒、方位角、仰角与 SNR，但没有卫星 ECEF 坐标。可由仿真发布端把与本次星座一致的卫星状态按 PRN 暴露出来；先验证单历元伪距与接收机钟差，再接入融合。原工程借 GNSS fix 构造虚拟卫星的 SPP-like 方法仅用于历史复现。不要把它的解与同源 fix 当作两份独立量测。两种 GNSS 模式分别报告。

相机和 LiDAR 前端在基线通过后按独立开关加入。相机用当前 CameraInfo、外参和采样时间，LiDAR 核对逐帧点云坐标及外参；只将通过质量检验的相对运动约束送入估计器。旧航线未验证二者稳定提高绝对位置精度。各做一次同包消融，通过后再纳入默认配置。


## 6. Camera + IMU 独立定位模式（随包补充）
   
请增加可独立启动的单目视觉惯性里程计（VIO）模式。输入只有同一台前向相机的图像、CameraInfo、相机与机体的静态外参，以及 IMU 角速度和加速度。模式验收要求关闭 GNSS、5G 和真值输入后仍能估计局部轨迹。下述模块补充前文的 GNSS+IMU 和 5G 模式。
   
1. 先阅读 camera_imu_reference/visual_imu_fixed_rotation.py 与 visual_imu_stable_preintegration.py。前者以陀螺积分约束帧间旋转，由图像匹配估计平移方向；后者在关键帧之间直接积分 IMU，用视觉方向和惯性约束求局部位置、速度。随包附历史视觉边、关键帧时间网格、稳定版输出及独立评价结果。离线脚本依赖原航线数据路径；包内没有原始图像和 IMU 数据全集。换机时替换数据入口。
2. 在线版按图像采样时刻截取覆盖整个关键帧区间的 IMU，使用当前相机内外参；用实测静止条件初始化重力和零偏，初始局部航向可置零。替换实验中写死的初始化时段及相机参数。视觉低视差、跟踪失败或 IMU 缺样时记录状态，停止采用失效视觉方向。采用有界滑窗与边缘化或等价的有界状态估计，禁止使用当前输出历元以后的观测。
3. 建议独立发布 /drone1/localization/vio/odometry，使用专属局部坐标系，例如 vio_odom。仅靠相机和 IMU 时，局部原点与初始航向由估计器定义。若主框架需要 map_enu 状态，必须先通过独立外部观测确定平移和航向。若与 GNSS/5G 联合，优先在同一估计器中加入视觉约束；共享同一批 IMU 的两路估计存在相关性，不能当作独立观测重复融合。
4. 先用同包比较 IMU-only 与 Camera+IMU，报告相同区间内 1 s、5 s 相对位移 RMSE、覆盖率、视觉约束接受数、状态年龄和最长中断。指标定义对齐随包历史评价，另列只对齐平移和航向、固定尺度后的轨迹误差。再验证 GNSS 中断前后视觉约束的贡献。此次附件给出的是离线原型和在线移植设计，实时运行须在目标框架另行验收。
