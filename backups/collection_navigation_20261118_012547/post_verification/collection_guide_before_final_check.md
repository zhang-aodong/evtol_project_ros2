# 数据采集与训练数据导出

## 1. 功能与边界

`evtol_collection` 是独立的数据采集包，不负责控制飞行、目标推理或生成传感器数据。支持两种采集模式：

| 模式 | 输入 | 保存内容 |
| --- | --- | --- |
| ROS 2 bag | 界面勾选的已发布 ROS 话题 | 标准 MCAP 或 SQLite3 bag，包含消息类型和 QoS，可用 `ros2 bag info/play` |
| 视觉训练数据 | 原始 RGB 和 UE 类别语义真值 | 图像、原始类别 PNG 掩码、YOLO 检测框和/或分割轮廓、时间戳与标签来源 |

采用用户确认的**语义真值派生标签**方案。标签不是视觉模型的预测结果，也不是逐架无人机实例真值。每个同类 8 连通区域生成一个框和一个外轮廓：同类飞机接触或重叠时可能合并，一架飞机被遮挡成多个不相连区域时也可能拆分。检测框表示可见部分，不推测被遮挡的完整外形。YOLO 外轮廓不能精确保留孔洞；原始 PNG 掩码保留原类别和孔洞，适合语义分割监督。

当前 Windows 实现每个 ROS 域、同一 Windows 登录会话只允许一个命令行/控制台采集进程；两种模式依次使用。无需修改或重新编译 UE 插件。采集依赖现有仿真发布器正常运行。

## 2. 编译与启动

在工程根目录的 PowerShell 执行：

```powershell
. .\scripts\activate.ps1
colcon build --merge-install --executor sequential --event-handlers console_direct+ --packages-select evtol_collection evtol_console
. .\install\local_setup.ps1
.\scripts\start_console.ps1
```

新机器先按原有环境教程安装 ROS 2 Jazzy Windows、Pixi 依赖和工程其他包；新包使用 `rclpy`、`sensor_msgs`、`std_msgs`、`std_srvs`、`rosbag2_py`、`rosidl_runtime_py`、NumPy、OpenCV、PyYAML，以及工程自带的 `evtol_simulation`。MCAP/SQLite3 存储插件须存在于 ROS 环境。该功能不要求训练环境或 GPU。

Foxglove 桌面软件连接终端显示的 `ws://127.0.0.1:8765`（端口冲突时以终端实际输出为准），添加 **EVTOL Simulation Console** 面板。已打开旧面板时重载 Foxglove；启动脚本会安装最新本地扩展。也可在浏览器打开 `http://127.0.0.1:9040` 使用同一界面。

先在控制台启动 UE 和 ROS 仿真，然后进入 **数据采集** 页。仅启动 UE 不会自动发布 ROS 图像。

## 3. Foxglove 操作

### 录制 bag

1. 设置输出根目录，默认相对于工程根目录的 `data/collections`；也可填写绝对路径。
2. 点击刷新话题，筛选并勾选需要保存的话题；支持输入完整绝对话题名。只保存所选话题，未出现的话题会等待，20 秒仍不存在则报错。
3. 选择 MCAP（默认）或 SQLite3，设置分卷大小、队列容量、是否使用仿真时钟。仿真建议开启仿真时钟，并勾选 `/clock`、`/tf`、`/tf_static` 和相关坐标原点话题，方便回放。
4. 开始 bag 采集，查看消息计数、状态、队列丢弃和绝对输出目录。
5. 点击正常停止并保存，等状态变成 `completed` 后再关闭程序。标准 bag 位于本次会话的 `rosbag2` 子目录。

原始 2K 图像占用很大。可选择 `/drone1/camera/rgb/image_realtime` 降低录制负载，但它是预览图，不用于下述精确真值配对。`split_size_mb` 分卷按字节近似触发，单条大消息可能超过边界；0 表示不分卷。

### 采集视觉训练数据

1. 保证 `/drone1/camera/rgb/image_raw` 和 `/drone1/camera/semantic/class_ids` 正常发布。
2. 设置 namespace（默认 `drone1`）、采样频率、类别、检测/分割导出类型和 `train/val/test`。默认只选类别 6（无人机），两种 YOLO 标签均导出。
3. 开始数据集采集，状态最初为 `waiting_frames`，收到同步数据后为 `recording`。长时间没有新配对会重新显示等待，不会伪造图像。
4. 查看已保存帧、标签区域数、未配对数、采样跳过数、队列丢弃数。正常停止并等待收尾。

RGB 和掩码必须有**完全相同的采集时间戳、frame_id 和尺寸**，不使用最近邻强行配对。采样频率按原始仿真采集时间节流，上限受源数据频率、DDS 和磁盘能力约束；5 Hz 是采集目标，不是提高源帧率。采样跳过是主动降采样，不是丢包。

没有选定类别的帧也会保存为负样本（空 YOLO 文本）。小区域剔除仅影响 YOLO 标签，原始掩码仍保留全部类别。注意小目标若低于面积阈值，可能成为被忽略的目标；远距离实验建议降低阈值并复查标签。

配置可从文件导入、导出，也可保存到独立的 `config/collection.local.json`。开始采集使用当前参数快照；修改或保存配置不会改变已运行采集器。仿真航点和原有 `console.local.json` 不受影响。

## 4. 命令行入口

不使用面板时，在已运行仿真、相同 ROS_DOMAIN_ID 的环境下执行：

```powershell
.\scripts\start_collection.ps1 -Mode bag -Config config\collection.example.json -Duration 30
.\scripts\start_collection.ps1 -Mode dataset -Config config\collection.example.json -OutputRoot 'D:\training_data' -Duration 60
```

`-Duration 0` 持续采集；时长是实际经过的秒数。按一次 Ctrl+C 正常停止并等待队列排空。亦可在另一个已激活环境的终端执行：

```powershell
ros2 service call /simulation/collection/stop std_srvs/srv/Trigger '{}'
```

停止服务不可达时，可在**当前会话目录**创建空文件 `stop.request`；采集器会检测该标记并正常收尾。不要在命令行采集期间从面板再启动第二个采集器。

## 5. 可调参数

示例文件为 `config/collection.example.json`，导入时要求字段完整、结构与示例一致。

| 字段 | 范围/默认 | 说明 |
| --- | --- | --- |
| `output_dir` | `data/collections` | 会话输出根目录 |
| `bag.topics` | 1–256 个唯一绝对话题 | 待录制的话题 |
| `bag.storage_id` | `mcap` / `sqlite3` | bag 存储格式 |
| `bag.use_sim_time` | `true` | bag 接收时间使用 ROS 仿真时钟；消息自身 header 不修改 |
| `bag.queue_mb` | 16–1024，默认 256 | 本地序列化消息等待队列内存上限 |
| `bag.split_size_mb` | 0–65536，默认 2048 | 分卷 MB；0 不分卷 |
| `dataset.namespace` | `drone1` | RGB/真值话题前缀，不是模型 namespace |
| `dataset.rate_hz` | 0.1–30，默认 5 | 按采集时间进行节流 |
| `dataset.class_ids` | 1–6，默认 `[6]` | 需要导出 YOLO 的原始类别及顺序 |
| `dataset.labels` | `detection`、`segmentation` 至少一项 | YOLO 输出类型；原始语义掩码始终保存 |
| `dataset.split` | `train` / `val` / `test` | 整段会话所属数据划分 |
| `dataset.min_area_px` | 1–100000，默认 20 | 最小连通区域像素数 |
| `dataset.polygon_epsilon_px` | 0–10，默认 1 | 外轮廓简化误差，单位像素 |
| `dataset.jpeg_quality` | 50–100，默认 95 | JPEG 图像质量；掩码始终为无损 PNG |
| `dataset.queue_frames` | 1–32，默认 8 | 写盘等待帧对数上限，同时有字节限制 |
| `dataset.pair_capacity` | 2–32，默认 12 | 时间戳配对缓存上限 |

类别对应：0 背景、1 飞机、2 建筑、3 行人、4 地面、5 跑道、6 无人机、255 未知/忽略。PNG 是单通道原始类别 ID，直接用图片查看器看通常接近黑色，需要调色板显示；**不要把原始掩码重新压成 JPEG**。

YOLO ID 根据 `class_ids` 的顺序从 0 连续编号。例如 `[6]` 时 `0=无人机`；`[3,6]` 时 `0=行人,1=无人机`。映射写入 `label_schema.json` 与 YAML，训练时不可混用不同映射的会话。

## 6. 文件格式、回放与训练

```text
data/collections/
  <会话名>.config.json         # 独立配置快照
  <会话名>.source_run.json     # 控制台发起时额外保存的仿真配置快照
  <会话名>/
    collection.json           # 完整性、参数和统计
    status.json               # 最新运行状态
    rosbag2/                  # bag 模式：metadata.yaml 与 mcap/db3 分卷
    label_schema.json         # 数据集模式：类别映射与派生标签限制
    frames.jsonl              # 每帧采集时间戳、坐标frame、目标区域等
    semantic/images/train/*.jpg
    semantic/masks/train/*.png
    detection/images/train/*.jpg
    detection/labels/train/*.txt
    segmentation/images/train/*.jpg
    segmentation/labels/train/*.txt
```

各任务有 `dataset.yaml`，并有 `val/test` 目录。图像文件名包含会话名和采集纳秒时间，掩码/标签同名。检测文本格式为 `class cx cy width height`，分割为 `class x1 y1 x2 y2 ...`，坐标均归一化。当前为兼容标准 YOLO 目录在三种任务下各保存一份图像，预估磁盘容量时需要计入重复图像。

bag 检查与回放：

```powershell
. .\scripts\activate.ps1
ros2 bag info '.\data\collections\<bag会话名>\rosbag2'
ros2 bag play '.\data\collections\<bag会话名>\rosbag2' --clock
```

回放前停止实时 ROS 发布器，避免同域出现两份源数据。`--clock` 由播放器生成时间；bag 中若也录入 `/clock`，回放时应按所用 Jazzy CLI 的 `--topics` 显式选择需要的数据话题，排除录入的 `/clock`，避免两个时钟源。RViz/Foxglove 固定坐标系和使用仿真时钟的节点也应与回放匹配。

训练时不要将同一视频的相邻帧随机分到训练集和验证集。分别改变场景、路径、距离、光照和遮挡采集 `train`、`val`，可额外采集 `test`；保持类别映射一致。组合独立会话：

```powershell
python -m evtol_collection.build_index --task detection --sessions 'D:\training_data\<train会话>' 'D:\training_data\<val会话>' --output 'D:\training_data\indexes\drone_detect_v1'
python -m evtol_collection.build_index --task segmentation --sessions 'D:\training_data\<train会话>' 'D:\training_data\<val会话>' --output 'D:\training_data\indexes\drone_segment_v1'
```

索引不复制图像；必须有非空 train 和 val，拒绝未正常完成或映射不一致的会话。在单独的 Ultralytics 训练环境中，例如：

```powershell
yolo detect train model=yolo11n.pt data='D:\training_data\indexes\drone_detect_v1\dataset.yaml' imgsz=1024 epochs=100
yolo segment train model=yolo11n-seg.pt data='D:\training_data\indexes\drone_segment_v1\dataset.yaml' imgsz=1024 epochs=100
```

这是通用 Ultralytics 格式示例，不代表已经训练模型。原工程的自定义语义模型可能有自己的训练代码；使用其训练流程时优先读取 `semantic/masks` 的原始 PNG，并配置同样的类别映射和 ignore=255，不能假定 YOLO 外轮廓可直接用于原 vendor 语义训练器。

## 7. 停止、完整性和性能

正常停止先停止订阅入队，排空本地写盘队列，再关闭 bag/索引并保存元数据。`completed` 且 `collection.json` 中 `complete=true, finalized=true` 表示正常收尾；`empty=true` 表示没有保存有效数据。`error` 请查看 error 字段及控制台日志。正常停止最多等待约 30 秒，磁盘或网络目录阻塞时可能失败。强制结束 ROS/UE 是应急手段，不保证正在写入的 bag 能恢复，不能把强制结束当作保存按钮。

队列丢弃为 0 仅表示**本地采集器等待写盘时没有因队列满而丢弃**，不证明 DDS、UE 或发布器全链路无丢帧。未配对数用于判断 RGB/真值同步问题；结合 received、paired、sampled_out、保存帧数一起分析。时钟回退会停止采集，避免重置后的数据混入同一会话；重启仿真前请先正常停止采集。大量原始图像/点云会增加 DDS、CPU 和磁盘负载，可减少话题、降低采样率或换本地 SSD，不会通过重复帧伪造频率。

迁移 Windows 11 机器时复制工程及所需数据，按机器配置重新激活和编译；相对输出路径会相对新工程根目录解析。会话内部使用相对任务目录，组合训练索引引用绝对图像路径，因此移动数据后需重新运行 `build_index`。无需硬编码本机用户目录或 UE 安装路径。

## 8. 本机验证

自动回归 154 项通过，包含标准 MCAP/SQLite3 写入关闭后读回、历史静态话题、精确图像配对、标签、空负样本、独立训练索引；面板类型检查、交互测试与扩展打包通过。

真实 UE 短测：15 对 RGB/原始掩码、30 个无人机连通区域标签；RGB/真值各接收 100 帧，精确配对 100，主动降采样 85，写盘队列丢弃和未配对均为 0。MCAP 选择 4 个话题共保存并读回 580 条消息（含 `/tf_static` 历史消息），完整收尾。具体报告见 `reports/collection_live_verification.json`，叠加预览为 `reports/collection_truth_preview.jpg`。这证明本机短采集流程可用，不等同于长时间满负载性能保证。
