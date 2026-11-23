/* Shared UI: runs inside the Foxglove extension or the same-origin local page. */
export function previewProfile(profile) {
  const heading=Number(profile.heading_deg)*Math.PI/180;
  let e=0,n=0;
  return profile.points.map((p,i,all)=>{
    if(i && Math.abs(Number(p.height_m)-Number(all[i-1].height_m))<1e-6){
      e+=Math.sin(heading)*Number(p.distance_m);n+=Math.cos(heading)*Number(p.distance_m);
    }
    return [e,n,Number(p.height_m)];
  });
}
const esc=v=>String(v??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const clone=v=>JSON.parse(JSON.stringify(v));
const css=`
*{box-sizing:border-box} .evtol{height:100%;min-height:500px;display:flex;flex-direction:column;background:#111823;color:#dbe5f2;font:13px/1.5 "Segoe UI","Microsoft YaHei",sans-serif}
.evtol header{display:flex;align-items:center;gap:12px;padding:13px 18px;background:#1a2433;border-bottom:1px solid #334254}.evtol h1{font-size:18px;margin:0}.evtol small{color:#91a4ba}.evtol .grow{flex:1}.evtol .badge{font-size:11px;border:1px solid #48617b;padding:3px 8px;border-radius:12px}
.evtol nav{display:flex;gap:3px;padding:8px 12px;background:#151e2b;flex-wrap:wrap}.evtol button{cursor:pointer;border:1px solid #40536b;border-radius:5px;background:#28394e;color:#eaf3ff;padding:7px 12px;font:inherit}.evtol button:hover{background:#3b5675}.evtol button:disabled{opacity:.4;cursor:default}.evtol button.primary{background:#167b88;border-color:#2296a4}.evtol button.danger{background:#693746;border-color:#985164}.evtol nav button.active{background:#304d70;border-color:#5a8ab7}
.evtol main{overflow:auto;flex:1;padding:15px;min-height:0}.evtol section[hidden]{display:none}.evtol .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(290px,1fr));gap:14px}.evtol .card{background:#192434;border:1px solid #324357;border-radius:7px;padding:15px;margin-bottom:13px}.evtol h2{font-size:15px;margin:0 0 12px;color:#ecf4ff}.evtol h3{font-size:13px;margin:12px 0 8px;color:#a5cfe3}.evtol label{display:flex;flex-direction:column;gap:5px;margin-bottom:11px;color:#acbfd3}.evtol input,.evtol select{background:#0f1825;color:#e0ebf7;border:1px solid #3d5068;border-radius:4px;padding:7px;min-width:0;width:100%;font:inherit}.evtol input[type=checkbox]{width:auto}.evtol .check{flex-direction:row;align-items:center;gap:8px}.evtol .row{display:flex;align-items:end;gap:10px;flex-wrap:wrap}.evtol .row label{flex:1;min-width:100px}.evtol .buttons{display:flex;gap:8px;flex-wrap:wrap;margin:10px 0}.evtol p{margin:8px 0;color:#9fb4ca}.evtol .hint{color:#e8bd76}.evtol .table-wrap{overflow:auto}.evtol table{width:100%;border-collapse:collapse;font-size:12px}.evtol th,.evtol td{text-align:left;padding:6px;border-bottom:1px solid #304052;white-space:nowrap}.evtol th{color:#9bb4ce}.evtol td input{min-width:75px;max-width:150px}.evtol td button{padding:3px 7px}.evtol svg{display:block;width:100%;min-height:250px;background:#101b29;border-radius:5px;touch-action:none}.evtol .status-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:10px}.evtol .metric{padding:12px;background:#101b29;border:1px solid #354a60;border-radius:6px}.evtol .metric b{display:block;font-size:20px;color:#87d6ce}.evtol pre{white-space:pre-wrap;word-break:break-word;max-height:260px;overflow:auto;background:#0f1824;padding:10px;color:#b5c8db}.evtol footer{padding:9px 15px;background:#192434;border-top:1px solid #35465a;display:flex;gap:10px;align-items:center}.evtol .notice{flex:1;overflow-wrap:anywhere}.evtol .error{color:#ff9b9b}.evtol .ok{color:#8ed4b6}.evtol .path{overflow-wrap:anywhere;font:12px Consolas,monospace}.evtol .subtle{font-size:12px;color:#92a5bb}.evtol .split{display:grid;grid-template-columns:minmax(280px,1fr) minmax(280px,1fr);gap:14px}@media(max-width:800px){.evtol .split{grid-template-columns:1fr}.evtol header small{display:none}}
`;

export function mountConsole(host, transport) {
  let draft=null,state=null,disposed=false,dirty=false,polling=false,previewTimer=null,dragging=null,activeTab='overview',collectionDraft=null,collectionTopics=[],clientEvents=[],observedOperations=new Map();
  const shell=document.createElement('div');shell.className='evtol';
  shell.innerHTML=`<style>${css}</style><header><h1>EVTOL 仿真控制台</h1><small>Windows · ROS 2 · UE / AirSim</small><span class="grow"></span><button class="danger" data-op="force_stop_all">强制结束 ROS + UE</button><span class="badge" id="connection">正在连接</span></header>
  <nav>${[['overview','运行总览'],['ue','UE 启动'],['actors','障碍机'],['mission','本机飞行'],['experiment','避障实验'],['localization','自机定位'],['collection','数据采集'],['monitor','监控 / 日志']].map(([id,t])=>`<button data-tab="${id}">${t}</button>`).join('')}</nav>
  <main><div class="buttons"><button id="import-config">从文件导入配置</button><input id="config-file" type="file" accept=".json,application/json" hidden><button id="export-config">导出当前编辑配置</button><button id="export-saved">导出已保存配置</button><button id="export-run">导出本次运行配置</button></div>
  <section data-page="overview"><div class="status-grid" id="metrics"></div><div class="grid" style="margin-top:14px"><div class="card"><h2>独立启动与停止</h2><p>① 编辑参数并保存 → ② 启动 UE，等待 AirSim 就绪 → ③ 启动 ROS 实验。</p><div class="buttons"><button class="primary" data-op="start_ue">启动 UE</button><button data-op="stop_ue">关闭 UE</button><button class="primary" data-op="start_ros">启动 ROS 实验</button><button data-op="stop_ros">停止 ROS</button><button class="danger" data-op="stop_all">停止全部</button><button data-op="cancel">取消当前启动</button></div><h3>ROS 启动选项</h3><label class="check"><input type="checkbox" data-bind="run.execute">执行飞行控制（关闭时仅观察）</label><label class="check"><input type="checkbox" data-bind="run.actors">启动障碍机 / 行人控制</label><label class="check"><input type="checkbox" data-bind="run.evaluate">发布目标真值并记录定位评估</label><label class="check"><input type="checkbox" data-bind="run.new_session">启动 ROS 时建立新的 ENU 原点会话</label><p class="hint">停止 ROS 会请求控制器停止和障碍机清理；关闭 UE 必须先停止 ROS。界面停止按钮仅管理本控制台启动的进程。</p></div><div class="card"><h2>本次运行</h2><div id="summary"></div><h3>最近操作</h3><div id="operations"></div></div></div></section>
  <section data-page="ue" hidden><div class="card"><h2>UE 程序与工程</h2><label>UE Editor 程序路径（.exe）<input data-bind="ue.editor" placeholder="D:\\Software\\Epic\\UE_4.27\\Engine\\Binaries\\Win64\\UE4Editor.exe"></label><label>UE 工程路径（.uproject）<input data-bind="ue.project"></label><label>AirSim 基础配置（项目相对路径或完整路径）<input data-bind="ue.settings"></label><div class="row"><label>运行方式<select id="visible"><option value="true">可视化窗口</option><option value="false">后台渲染</option></select></label><label>窗口宽度<input type="number" data-bind="ue.width" min="320" max="7680"></label><label>窗口高度<input type="number" data-bind="ue.height" min="240" max="4320"></label></div><p>相机和雷达仍使用原有共享内存设置。后台模式继续生成传感器数据；窗口尺寸不改变采集图像的分辨率。</p><div class="buttons"><button data-save class="primary">保存全部参数</button><button data-op="start_ue">启动 UE</button><button data-op="stop_ue">关闭 UE</button></div></div></section>
  <section data-page="actors" hidden><div class="card"><h2>障碍机场景</h2><div class="row"><label>障碍机数量<input id="actor-count" type="number" min="0" max="64"></label><label>障碍机初始化<select data-bind="actors.initialization"><option value="first_waypoint">首航点空中布设并悬停（推荐多机实验）</option><option value="ground">配置出生点地面起飞（需要无碰撞地面）</option></select></label><label class="check"><input type="checkbox" data-bind="actors.pedestrians_enabled">同时启用原有行人配置</label></div><p>航点使用本机初始点为原点的 ENU 米坐标。空中布设只在实验开始前设置障碍机首航点并清零速度，飞行途中不重定位；本机起飞后才开始障碍机航线。首航点仍须避开场景模型。地面模式要求出生点位于有效地面。数量或出生位置改变后需重启 UE 和 ROS。</p></div><div id="actor-list"></div><button data-save class="primary">保存全部参数</button></section>
  <section data-page="mission" hidden><div class="card"><h2>本机飞行模式</h2><div class="row"><label>模式<select data-bind="mission.mode" id="flight-mode"><option value="waypoints">ENU 航点 + 飞行速度</option><option value="profile">速度—高度剖面</option></select></label><label>航点模式速度（m/s）<input type="number" data-bind="mission.speed_mps" min="0.05" step="0.1"></label><label>总速度上限（m/s）<input type="number" data-bind="mission.max_speed_mps" min="0.1" max="30" step="0.1"></label><label>最大加速度（m/s²）<input type="number" data-bind="mission.max_acceleration_mps2" min="0.05" max="5" step="0.1"></label></div><label class="check"><input type="checkbox" data-bind="mission.land_at_last">末点在地面时自动降落（关闭则停在交接高度悬停）</label></div>
  <div id="waypoint-editor" class="card"><h2>本机 ENU 航点</h2><div id="ego-points"></div><button id="add-ego">＋ 添加航点</button></div>
  <div id="profile-editor"><div class="split"><div class="card"><h2>有序剖面点</h2><label>水平航向（°，0=北，90=东）<input id="heading" data-bind="mission.profile.heading_deg" type="number" step="1"></label><div id="profile-points"></div><div class="buttons"><button id="add-profile">＋ 添加剖面点</button><button id="example-profile">加载示例</button></div><p>每行顺序连接。等高：水平向前飞行该行的距离；高度变化：只垂直升降，该行水平距离忽略。起点和终点速度必须为 0。</p><p class="hint">横轴速度是三维地速大小。水平 / 垂直转向点会按安全要求减速，避障也可能改变实际剖面。</p></div><div class="card"><h2>速度—高度预览</h2><div class="row"><label class="check"><input id="click-add" type="checkbox">点击图中添加点</label><label>新增等高段默认距离（m）<input id="default-distance" type="number" value="100" min="0.05"></label></div><svg id="profile-chart" viewBox="0 0 680 380" aria-label="速度高度图"></svg><p>蓝线：输入剖面；绿线：实际飞行。点按输入顺序连接，可拖动蓝色点，数值表同步更新。</p><div id="profile-description" class="subtle"></div></div></div><div class="card"><h2>转换后的 ENU 航点</h2><p>航点模式采用 2 m 采样、3 m 前视跟踪、2 Hz 路径规划；近距避障 4 m，直线中间点不停站，终点和急转弯减速。危险或数据过期时仍会制动。</p><div id="compiled-route"></div><div id="validation" class="hint"></div></div></div>
  <div class="buttons"><button class="primary" data-save>校验并保存全部参数</button><button id="reload">重新读取已保存参数</button></div></section>
  <section data-page="experiment" hidden>
  <div class="card"><h2>激光—视觉融合与评估参数</h2><p>文件导入只载入草稿，保存后下一次启动生效。改变视场、数量或出生点需重启 UE。范围检测率与风险预警正确率分别评价；主统计为以机体前向为轴的 10 m 完整半球，遮挡和相机盲区不剔除。三相机偏航 −60° / 0° / +60°，默认每路 70°、水平合计覆盖约 190°；相邻重叠 10°。默认图像 1152×1536、上仰 12°，增大垂直覆盖；上下盲区仍计入完整半球漏警，三路批量 ONNX CUDA 推理，上限 15 Hz；相机采集仍为 20 Hz。</p>
  <div class="row"><label>每路水平视场 °（合计≈本值+30°）<input type="number" min="20" max="120" data-bind="sensors.horizontal_fov_deg"></label><label>推理尺寸（32 的倍数）<input type="number" min="320" max="1536" step="32" data-bind="sensors.imgsz"></label><label>每路宽度<input type="number" data-bind="sensors.width" min="1024" max="2048" step="32"></label><label>每路高度<input type="number" data-bind="sensors.height" min="1024" max="2048" step="32"></label><label>检测频率上限 Hz<input type="number" data-bind="sensors.max_hz" min="1" max="20"></label><label class="check"><input type="checkbox" data-bind="dynamic.enabled">启用融合动态避障</label></div>
  <details open><summary>检测与事件统计参数</summary><div id="experiment-options" class="grid"></div></details>
  <details open><summary>起飞后近距雷达紧急避障</summary><p>独立于视觉检测，默认 2 m 全向点云兜底。控制器进入 flying 后启用；地面、起飞阶段及最后降落交接不启用主动脱离。参数保存后重启 ROS 生效。</p><label class="check"><input type="checkbox" data-bind="raw_emergency.enabled">启用起飞后雷达兜底</label><div id="raw_emergency-options" class="grid"></div><div id="raw-emergency-status" class="status-grid"></div></details>
  <details open><summary>独立雷达与融合预警参数</summary><div id="warnings-options" class="grid"></div></details><details><summary>动态避障与包络参数</summary><div id="dynamic-options" class="grid"></div></details>
  <button data-save class="primary">校验并保存全部参数</button></div>
  <div class="card"><h2>检测链路与侧向保护</h2><div class="status-grid" id="perception-health"></div><p>视觉融合参与目标检测评分；全周雷达未分类目标独立参与避障，不计作视觉检出。超时结果不回填成功率。</p></div>
  <div class="card"><h2>实时范围检测与进入事件</h2><p id="evaluation-validity">等待评估</p><div class="status-grid" id="evaluation-metrics"></div><p id="evaluation-details"></p><svg id="evaluation-chart" viewBox="0 0 680 200" aria-label="范围检测成功率"></svg></div>
  <div class="card"><h2>范围内真值与检测匹配</h2><div id="evaluation-targets"></div><p>3D 面板固定坐标设为 map_enu，添加 /evaluation/markers；蓝色半球线框为预警统计范围，绿色为检出、红色为漏检真值框。</p></div>
  <div class="card"><h2>双路实时预警</h2><div id="dual-warnings" class="status-grid"></div><div id="warning-targets"></div><p>雷达通道：实测离地高度超过 5 m 后，周围 5 m 内任意有效非自身回波预警。融合通道：前向 10 m 完整半球；位置/速度预测不使用障碍机真值。超时数据标为无效，不显示为安全。</p></div><div class="card"><h2>动态风险与冻结事件预警统计</h2><div id="warning-metrics" class="status-grid"></div><pre id="warning-details"></pre><p>真值 CPA 为当前运动趋势诊断，不用于闭环后反算协议准确率。填写 warning_events 冻结风险/安全交会标签后，才显示事件混淆矩阵。</p></div>
  </section>
  <section data-page="localization" hidden>
  <div class="card"><h2>自机多源融合定位</h2><div class="row">
  <label class="check"><input type="checkbox" data-bind="localization.enabled">启用自机定位</label>
  <label>在线模式<select data-bind="localization.mode"><option value="gnss_imu">GNSS + IMU</option><option value="gnss_imu_5g">GNSS + IMU + 逐站 5G-A</option></select></label>
  <label>下游状态来源<select data-bind="localization.state_source"><option value="truth">真值控制，融合旁路评价</option><option value="fused">融合定位控制</option></select></label>
  <label class="check"><input type="checkbox" data-bind="localization.evaluate">独立真值误差评价</label></div>
  <p>坐标为会话 ENU，原点来自统一会话约定。滤波器只消费 IMU、GNSS 和可选 5G-A，不消费真值位姿。第一版航向初始化依赖仿真 IMU 的有效姿态；Camera+IMU 留作后续。定位开启后重新启动 ROS 生效。</p>
  <div class="row"><label>排序等待 s<input type="number" step="0.01" data-bind="localization.reorder_s"></label><label>状态过期 s<input type="number" step="0.01" data-bind="localization.stale_s"></label><label>GNSS 标准差下限 m<input type="number" step="0.05" data-bind="localization.gnss_sigma_floor_m"></label><label>5G 标准差下限 m<input type="number" step="0.05" data-bind="localization.fiveg_sigma_floor_m"></label><label>5G 最低 RSRP dBm<input type="number" data-bind="localization.fiveg_min_rsrp_dbm"></label></div>
  </div><div class="card"><h2>飞行中卫星异常与恢复</h2>
  <div class="row"><label class="check"><input type="checkbox" data-bind="localization.gnss_fault.enabled">启用 GNSS 异常测试</label><label>时间基准<select data-bind="localization.gnss_fault.time_base"><option value="flight">本次飞行进入起飞阶段</option><option value="session">会话原点时刻（适合离线回放）</option></select></label><label>未列出时段<select data-bind="localization.gnss_fault.default_mode"><option value="normal">正常</option><option value="dropout">中断</option></select></label></div>
  <p>每段使用 [start_s, end_s)，模式 normal=正常、dropout=屏蔽、no_fix=失锁、bias=ENU 位置偏置；时段必须有序且不重叠。默认 0–40 s 正常、40–60 s 中断、60 s 后自动恢复。仅影响估计器 GNSS 输入，原始话题与真值保留。</p>
  <textarea id="gnss-fault-segments" rows="9" spellcheck="false" style="width:100%;background:#0f1825;color:#dbe5f2"></textarea><p id="gnss-fault-validation"></p>
  <div class="buttons"><button id="gnss-fault-example">加载 40–60 s 中断示例</button><button class="primary" data-save>保存全部参数</button></div>
  </div><div class="card"><h2>独立定位误差（与真值同时间戳对齐）</h2><div class="status-grid" id="localization-metrics"></div>
  <div class="row"><label>曲线时间窗口<select id="localization-window"><option value="60">最近 60 秒</option><option value="300">最近 5 分钟</option><option value="0">全部已缓存（最多 10 分钟）</option></select></label><div class="buttons"><button id="localization-pause">暂停曲线</button><button id="localization-clear">清空曲线</button><button id="localization-export">导出误差 CSV</button></div></div>
  <p id="localization-curve-status" role="status">等待独立误差评价数据</p>
  <div id="localization-charts" style="display:grid;grid-template-columns:minmax(0,1fr);gap:24px">${[['3d','三维位置误差','#87d6ce'],['x','X / 东向误差','#78b7ff'],['y','Y / 北向误差','#f0ba68'],['z','Z / 高度误差','#cc9bff']].map(([key,title,color])=>`<div><h3 style="color:${color}">${title} <span data-localization-value="${key}">—</span></h3><svg id="localization-chart${key==='3d'?'':'-'+key}" data-error-chart="${key}" viewBox="0 0 680 240" preserveAspectRatio="none" aria-label="${title}曲线" role="img" style="height:clamp(300px,40vh,480px);min-height:300px"></svg><small data-localization-hover="${key}">移动鼠标查看采样时刻与误差</small></div>`).join('')}</div>
  <p>X、Y、Z 为同一真值历元的“估计 − 真值”，保留正负号；三维误差为 √(X²+Y²+Z²)，单位 m。横轴为距会话起点的仿真时间（s），不使用真值拟合轨迹。数据过期时保留历史并停止追加，采样中断处断线。曲线约 10 Hz，缓存最多 6000 点；完整逐历元误差仍保存于评价文件。</p>
  <h3>恢复、采用观测与逐基站状态</h3><pre id="localization-details"></pre>
  <p>3D 面板叠加 /drone1/localization/path 与 /drone1/localization/truth_path；Plot 可使用 /drone1/localization/error_3d 和 /drone1/localization/error_enu.vector.x/y/z。traveled_path 始终代表下游所选状态。</p></div></section>
  <section data-page="collection" hidden>
  <div class="card"><h2>独立数据采集</h2><p>先启动 ROS 传感器发布，再按需开始采集。两种模式每次创建独立目录，可在仿真运行中开始和停止，不改变飞行任务。</p>
  <label>保存根目录（项目相对路径或完整路径）<input id="collect-output" data-collect="output_dir" placeholder="data/collections"></label>
  <div class="buttons"><button id="collect-save">保存采集配置</button><button id="collect-import">导入采集配置</button><input id="collect-file" type="file" accept=".json,application/json" hidden><button id="collect-export">导出采集配置</button></div></div>
  <div class="split"><div class="card"><h2>ROS 2 bag：选择话题</h2>
  <div class="row"><label>存储格式<select data-collect="bag.storage_id"><option value="mcap">MCAP（ROS 2 bag）</option><option value="sqlite3">SQLite3（.db3）</option></select></label><label>分卷大小 MB（0=不分卷）<input type="number" data-collect="bag.split_size_mb" min="0" max="65536"></label></div>
  <label class="check"><input type="checkbox" data-collect="bag.use_sim_time">使用仿真时间（需 /clock，仿真重置会停止录制）</label>
  <div class="buttons"><button id="collect-refresh">刷新已发布话题</button><button id="collect-select-visible">选择全部显示话题</button><button id="collect-select-none">清空选择</button></div>
  <input id="collect-topic-filter" placeholder="筛选话题，例如 camera、lidar"><div id="collect-topics" style="max-height:320px;overflow:auto;margin:10px 0"></div>
  <div class="row"><label>手动添加绝对话题<input id="collect-custom-topic" placeholder="/drone1/imu/data"></label><button id="collect-add-topic">添加</button></div>
  <p id="collect-topic-count"></p><p>建议保留 /clock、/tf、/tf_static 和会话原点。只录制实际选择的话题；原图和多传感器高频录制需要足够磁盘吞吐。</p><button id="collect-bag" class="primary">开始录制 ROS 2 bag</button></div>
  <div class="card"><h2>视觉训练数据：UE 语义真值</h2><div class="row"><label>采样频率 Hz<input type="number" data-collect="dataset.rate_hz" min="0.1" max="30" step="0.1"></label><label>数据划分<select data-collect="dataset.split"><option value="train">train（训练）</option><option value="val">val（验证）</option><option value="test">test（测试）</option></select></label></div>
  <label>相机命名空间<input data-collect="dataset.namespace"></label><div id="collect-classes" class="row"></div>
  <label class="check"><input id="collect-detection" type="checkbox">YOLO 目标检测框</label><label class="check"><input id="collect-segmentation" type="checkbox">YOLO 分割轮廓</label>
  <div class="row"><label>最小连通区域像素<input type="number" data-collect="dataset.min_area_px" min="1" max="100000"></label><label>轮廓简化误差 px<input type="number" data-collect="dataset.polygon_epsilon_px" min="0" max="10" step="0.1"></label><label>JPEG 质量<input type="number" data-collect="dataset.jpeg_quality" min="50" max="100"></label></div>
  <p>始终保存原始类别 ID 的无损 PNG 掩码及配对 RGB。YOLO 类别自动映射为连续 ID，附 dataset.yaml 和逐帧索引。整次采集放在同一 split；验证集和测试集请另开独立场景采集。</p>
  <p class="hint">标签来自 UE 类别语义真值，不使用模型预测。检测框按同类连通区域生成，接触或重叠的同类目标可能合并；YOLO 外轮廓不表达孔洞，精确像素以原始 PNG 为准。</p><button id="collect-dataset" class="primary">开始采集 YOLO + 语义真值</button></div></div>
  <div class="card"><h2>采集进度与落盘状态</h2><div id="collect-status" class="status-grid"></div><p id="collect-path" class="path"></p><pre id="collect-details"></pre><button id="collect-stop" class="danger">停止采集并保存</button><p>等待“completed / 已收尾”再移动或回放数据。强制结束会绕过保存流程，可能留下未完成的 bag；采集状态会明确标为中断。</p></div>
  </section>
  <section data-page="monitor" hidden><div class="card"><h2>ROS 进程与日志路径</h2><div id="processes"></div></div><div class="card"><h2>传感器与算法诊断</h2><div id="diagnostics"></div></div><div class="card"><h2>详细状态</h2><pre id="runtime-details"></pre><h2>障碍机运行状态</h2><div id="actor-states"></div></div></section>
  </main><details open class="card" style="margin:0 12px;flex-shrink:0"><summary>实时操作与运行日志（所有页面可见）</summary><div class="row"><label>筛选来源 / 关键字<input id="log-filter" placeholder="例如 actors、gateway、ERROR"></label><label class="check"><input id="log-errors" type="checkbox">仅警告 / 错误</label><label class="check"><input id="log-follow" type="checkbox" checked>自动滚动</label><button id="export-logs">导出当前日志</button></div><pre id="logs" style="height:155px;max-height:30vh;margin:0"></pre><small id="log-path"></small></details><footer><span class="notice" id="notice">等待管理程序</span><span id="dirty" class="hint"></span><button data-save class="primary">保存参数</button></footer>`;
  host.appendChild(shell);
  const $=s=>shell.querySelector(s),$$=s=>Array.from(shell.querySelectorAll(s));
  const notice=(message,error=false)=>{ $('#notice').textContent=message;$('#notice').className='notice '+(error?'error':'ok'); };
  function clientLog(message,level='INFO'){
    clientEvents.push({wall:Date.now()/1000,time:new Date().toLocaleTimeString(),source:'界面',level,message});
    clientEvents=clientEvents.slice(-200);renderLogs();
  }
  function renderLogs(){
    const filter=$('#log-filter').value.toLowerCase();
    const lines=[...(state?.logs||[]),...clientEvents].sort((a,b)=>(a.wall||0)-(b.wall||0));
    const shown=lines.filter(l=>(!$('#log-errors').checked||['ERROR','WARN'].includes(l.level))&&(!filter||JSON.stringify(l).toLowerCase().includes(filter)));
    $('#logs').textContent=shown.map(l=>`${l.time} [${l.level||'INFO'}] [${l.source||'manager'}] ${l.message}`).join('\n');
    if($('#log-follow').checked)$('#logs').scrollTop=$('#logs').scrollHeight;
    $('#log-path').textContent=state?.log_file?'完整管理日志：'+state.log_file:'';
  }
  $('#log-filter').oninput=renderLogs;$('#log-errors').onchange=renderLogs;
  $('#export-logs').onclick=()=>{
    const blob=new Blob([JSON.stringify({exported_at:new Date().toISOString(),state,clientEvents},null,2)],{type:'application/json'});
    const url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download='evtol-console-log.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
    clientLog('已导出当前日志与状态快照');
  };
  function download(value,name){
    const blob=new Blob([JSON.stringify(value,null,2)],{type:'application/json'});
    const url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
  }
  $('#import-config').onclick=()=>$('#config-file').click();
  $('#config-file').onchange=async()=>{
    const file=$('#config-file').files?.[0];if(!file)return;
    try{if(file.size>1048576)throw new Error('配置文件不能超过 1 MB');
      const data=JSON.parse(await file.text()),previous=draft;
      draft=await call('import_config',data);mark();populate();updateStatus();
      notice(`配置已载入草稿：版本 ${draft.version}，障碍机 ${draft.actors.count} 架；尚未保存，不影响正在运行的实验`);
      clientLog('文件导入校验成功：'+file.name+'；未执行启动或保存');
    }catch(e){notice('导入失败：'+e.message,true);clientLog('导入失败：'+e.message,'ERROR');}
    finally{$('#config-file').value='';}
  };
  $('#export-config').onclick=async()=>{try{download(await call('export_config',{config:draft}),'evtol-experiment-draft.json');notice('已导出当前草稿，文件不代表本次实际运行参数');}catch(e){notice(e.message,true);}};
  $('#export-saved').onclick=async()=>{try{download(await call('export_config'),'evtol-experiment-saved.json');}catch(e){notice(e.message,true);}};
  $('#export-run').onclick=async()=>{try{download(await call('export_run'),'evtol-experiment-run.json');}catch(e){notice(e.message,true);}};
  const optionNames={region:'统计区域（front_hemisphere）',lidar_radius_m:'雷达近距预警半径 m',lidar_enable_agl_m:'雷达预警启用离地高度 m',fusion_radius_m:'融合预警半球半径 m',prediction_horizon_s:'距离/速度预测时域 s',prediction_step_s:'预测采样步长 s',max_measurement_age_s:'预警测量有效期 s',target_success_rate:'及时预警成功率目标 0..1',radius_m:'预警统计半径 m',association_gate_m:'表面中心到机体原点匹配门限 m',evidence_s:'实际观测证据有效期 s',window_s:'滑窗长度 s',timely_s:'及时检出时限 s',exit_margin_m:'退出滞回距离 m',exit_hold_s:'退出保持时间 s',buffer_s:'评估最大缓冲 s',truth_max_gap_s:'真值最大插值跨度 s',truth_hz:'真值/评估目标频率 Hz',publish_hz:'指标显示发布频率 Hz',class_ids:'评估类别 ID（当前只支持 [6]）',target_size_m:'评估机体尺寸 ENU m',ego_radius_m:'自机包络半径 m',risk_clearance_m:'风险表面安全间距 m',risk_horizon_s:'评估风险预测时域 s',horizon_s:'动态候选预测时域 s',step_s:'动态分段检查步长 s',clearance_m:'动态表面安全余量 m',min_size_m:'障碍机最小尺寸 ENU m',sigma_scale:'不确定度膨胀倍数',max_age_s:'融合测量最大年龄 s',max_velocity_std_mps:'速度不可信门限 m/s',max_targets:'最大融合目标数',degraded_speed_mps:'融合掉线时速度上限 m/s',score_only_flying:'仅在网关 flying 阶段计分（起飞/预置只显示）'};
  Object.assign(optionNames,{escape_speed_mps:'脱离速度上限 m/s',self_exclusion_m:'自身回波排除半径 m',max_range_rate_mps:'径向接近速度上限 m/s',range_history_s:'径向距离历史窗口 s'});
  function optionEditors(){for(const section of ['experiment','dynamic','warnings','raw_emergency']){
    const container=$('#'+section+'-options');container.innerHTML=Object.entries(draft[section]).filter(([k])=>k!=='enabled').map(([key,value])=>`<label>${esc(section==='raw_emergency'&&key==='radius_m'?'触发距离 m（机体原点至回波）':optionNames[key]||key)}<input data-option="${section}.${key}" type="${typeof value==='boolean'?'checkbox':Array.isArray(value)||typeof value==='string'?'text':'number'}" step="0.05" value="${esc(Array.isArray(value)?JSON.stringify(value):value)}" ${value===true?'checked':''}></label>`).join('');
    container.querySelectorAll('input').forEach(el=>el.oninput=()=>{try{set(el.dataset.option,el.type==='checkbox'?el.checked:el.type==='number'?(el.value===''?null:Number(el.value)):(typeof draft[section][el.dataset.option.split('.')[1]]==='string'?el.value:JSON.parse(el.value)));}catch(e){notice('数组请输入 JSON，例如 [1,1,0.5]',true);}});
  }}
  const metricHistory=[];
  function evaluation(t){
    const diagnostic=suffix=>Object.entries(t.diagnostics||{}).find(([name])=>name.endsWith('/'+suffix))?.[1];
    const surround=diagnostic('surround_vision'),visual=surround||diagnostic('vision'),fusion=diagnostic('target_localizer'),side=diagnostic('lidar_obstacles'),safety=diagnostic('safety');
    const ageValue=(entry,key)=>entry&&Date.now()/1000-entry.received<2?entry.values?.[key]??'—':'—';
    const pipeline=t.detection_metrics?.pipeline;
    const rawFresh=!!state.ros_running&&Date.now()/1000-(t.mission_received_wall||0)<2;
    const raw=rawFresh?t.mission?.safety_data?.lidar_emergency:null;
    $('#raw-emergency-status').innerHTML=[['雷达兜底状态',!raw?'等待新鲜状态':!raw.armed?'未启用（'+(raw.reason||'非飞行阶段')+'）':raw.active?'近距避障中':'已启用 / 范围内无回波'],['最近回波 m',raw?.nearest_m?.toFixed(2)??'—'],['径向接近速度 m/s',raw?.max_closing_mps?.toFixed(2)??'—'],['约束方向数',raw?.cells??'—'],['当前动作',raw?.action??'—']].map(([k,v])=>`<div class="metric"><small>${esc(k)}</small><b>${esc(v)}</b></div>`).join('');
    $('#perception-health').innerHTML=[['路径规划 Hz',t.mission?.safety_data?.planning_hz??'—'],['点云应急范围 m',t.mission?.safety_data?.raw_emergency_radius_m??'—'],['融合避障邻域 m',t.mission?.safety_data?.fused_avoidance_radius_m??'—'],['点云应急介入',t.mission?.safety_data?.raw_emergency_active?'是':'否'],['近距融合目标',t.mission?.safety_data?.dynamic_targets_local??'—'],['视觉推理 ms',ageValue(visual,'inference_ms')],['视觉结果年龄 ms',ageValue(visual,'output_age_ms')],['融合结果年龄 ms',ageValue(fusion,'output_age_ms')],['视觉无人机检测框',ageValue(visual,'drone_boxes')],['推理后端',ageValue(visual,'backend')],['雷达未分类点簇（仅诊断）',ageValue(side,'active_tracks')],['雷达运动轨迹（仅诊断）',ageValue(side,'moving_tracks')],['累计几何匹配目标次',pipeline?.matched_frame_targets??'—'],['超时帧 / 已评估帧',pipeline?`${pipeline.late_frames} / ${pipeline.frames}`:'—']].map(([k,v])=>`<div class="metric"><small>${esc(k)}</small><b>${esc(v)}</b></div>`).join('');
    const m=t.detection_metrics,age=Date.now()/1000-(t.detection_metrics_received_wall||0),fresh=!!state.ros_running&&age<2;
    $('#evaluation-validity').textContent=!m?'等待独立评估节点':!fresh?'指标已过期，当前数字仅为最后记录':!m.valid?'不可评估：真值不完整，缺失 '+JSON.stringify(m.missing):`${m.scoring?'正式计分':'预览（未计入累计）'} · ${m.region==='front_hemisphere'?'前向半球':'全周球'} ${m.radius_m} m · 统计仿真时刻 ${m.stamp_s.toFixed(2)} s · 评估延迟 ${m.evaluation_delay_s?.toFixed(2)} s`;
    const percent=v=>v==null?'N/A':(v*100).toFixed(1)+'%';
    const cards=m?.valid?[['真实障碍机',m.truth_count],['正确检出',m.detected_count],['漏检',m.missed_count],['误检',m.false_positive_count],['当前范围成功率',percent(m.recall)],['最近滑窗成功率',percent(m.window_recall)],['本次累计成功率',percent(m.cumulative_recall)],['检测精确率',percent(m.precision)],['共同视场成功率',percent(m.fov_recall)],['及时检出事件成功率',percent(m.timely_success_rate)]]:[];
    $('#evaluation-metrics').style.opacity=fresh?'1':'.4';$('#evaluation-metrics').innerHTML=cards.map(([k,v])=>`<div class="metric"><small>${esc(k)}</small><b>${esc(v)}</b></div>`).join('');
    $('#evaluation-details').textContent=m?`真值已知 ${m.known}/${m.expected}；进入事件 ${JSON.stringify(m.encounters||{})}；有效/无效历元 ${m.totals?.valid||0}/${m.totals?.invalid||0}；观测证据有效期 ${m.evidence_s||'—'} s。正式计分阶段由“仅飞行时计分”参数控制。`:'';
    const rows=(m?.targets||[]).filter(r=>r.in_range);
    $('#evaluation-targets').innerHTML=table(['真值机名','距离 m','检出','共同视场','轨迹 ID','观测年龄 s','进入事件'],rows.map(r=>`<tr><td>${esc(r.name||r.identity)}</td><td>${r.distance_m.toFixed(2)}</td><td>${r.detected?'成功':'漏检'}</td><td>${r.in_common_fov?'是':'否'}</td><td>${r.track_id??'—'}</td><td>${r.observation_age_s?.toFixed(2)??'—'}</td><td>${esc(r.event)}</td></tr>`));
    if(m&&fresh&&metricHistory.at(-1)?.stamp!==m.stamp_s){if(metricHistory.at(-1)?.session!==m.session_id)metricHistory.length=0;metricHistory.push({stamp:m.stamp_s,value:m.valid?m.recall:null,session:m.session_id});if(metricHistory.length>300)metricHistory.shift();}
    const first=metricHistory[0]?.stamp||0,last=metricHistory.at(-1)?.stamp||1;
    const points=metricHistory.filter(p=>p.value!=null).map(p=>`${40+(p.stamp-first)/Math.max(1,last-first)*620},${170-p.value*140}`).join(' ');
    $('#evaluation-chart').innerHTML=`<text x="4" y="25" fill="#a5cfe3">100%</text><text x="4" y="175" fill="#a5cfe3">0%</text><line x1="40" y1="170" x2="660" y2="170" stroke="#40536b"/><polyline points="${points}" fill="none" stroke="#87d6ce" stroke-width="2"/>`;
    $('#warning-details').textContent=JSON.stringify(t.warning_metrics||{status:'等待风险诊断'},null,2);
    const report=t.warning_metrics,hw=report?.hemisphere,lw=report?.lidar,fw=report?.fusion;
    const warningFresh=!!state.ros_running&&Date.now()/1000-(t.warning_metrics_received_wall||0)<2;
    const values=[['预警统计',warningFresh&&hw?.valid?'有效':'过期 / 真值不完整'],['前向半球真实障碍机',hw?.truth_count??'—'],['正确预警目标',hw?.detected_count??'—'],['当前预警召回率',percent(hw?.recall)],['累计预警召回率',percent(hw?.cumulative_recall)],['及时预警事件成功率',percent(hw?.timely_success_rate)],['及时漏警率',percent(report?.hemisphere_miss_rate)],['预警精确率',percent(hw?.precision)],['目标',percent(report?.target_success_rate)],['离地高度 m',lw?.agl_m?.toFixed(2)??'未知'],['雷达近距预警',!warningFresh||!lw?.valid?'数据无效':!lw.enabled?'未达启用高度':lw.warning?'预警':'范围内无回波'],['5 m 内回波数',lw?.point_count??'—'],['融合预警',!warningFresh||!fw?.valid?'数据无效':fw.warning?'预警':'未发现范围内目标']];
    $('#dual-warnings').style.opacity=warningFresh?'1':'.4';
    $('#dual-warnings').innerHTML=values.map(([k,v])=>`<div class="metric"><small>${esc(k)}</small><b>${esc(v)}</b></div>`).join('');
    $('#warning-targets').innerHTML=table(['跟踪 ID','距离 m','目标速度 m/s','接近速度 m/s','预测最近距离 m','预计进入 s','当前预警'],(fw?.targets||[]).map(d=>`<tr><td>${d.track_id}</td><td>${d.distance_m.toFixed(2)}</td><td>${Math.hypot(...d.velocity_enu_mps).toFixed(2)}</td><td>${d.closing_speed_mps.toFixed(2)}</td><td>${d.cpa_distance_m.toFixed(2)}</td><td>${d.predicted_entry_s??'—'}</td><td>${d.warning?'是':'否'}</td></tr>`));
    const w=t.warning_metrics?.labeled_event_metrics;
    $('#warning-metrics').innerHTML=w?[['预警精确率',percent(w.precision)],['风险召回率',percent(w.recall)],['漏警率',percent(w.miss_rate)],['误警率 FPR',percent(w.false_alarm_rate)],['误报占预警比',percent(w.false_discovery_rate)],['事件计数',JSON.stringify(w.counts)]].map(([k,v])=>`<div class="metric"><small>${esc(k)}</small><b>${esc(v)}</b></div>`).join(''):'<p>尚未配置冻结事件标签，不计算协议级比率。</p>';
  }
  const mark=()=>{dirty=true;$('#dirty').textContent='未保存';};
  const get=(path)=>path.split('.').reduce((o,k)=>o?.[k],draft);
  const set=(path,value)=>{const parts=path.split('.');let o=draft;for(const k of parts.slice(0,-1))o=o[k];o[parts.at(-1)]=value;mark();};
  async function call(action,payload={}){
    const response=await transport({action,payload});
    if(!response?.ok)throw new Error(response?.error||'管理程序未返回成功结果');
    return response.data;
  }
  function table(headers,rows){return `<div class="table-wrap"><table><thead><tr>${headers.map(h=>`<th>${esc(h)}</th>`).join('')}</tr></thead><tbody>${rows.join('')}</tbody></table></div>`;}
  function pointEditor(container,points,onChange){
    container.innerHTML=table(['序号','东 E / m','北 N / m','高 U / m','操作'],points.map((p,i)=>`<tr><td>${i+1}</td>${p.map((v,j)=>`<td><input type="number" value="${esc(v)}" step="0.1" data-index="${i}" data-axis="${j}"></td>`).join('')}<td><button data-remove="${i}">删除</button><button data-up="${i}">↑</button></td></tr>`));
    container.querySelectorAll('input').forEach(input=>input.oninput=()=>{points[+input.dataset.index][+input.dataset.axis]=input.value===''?null:Number(input.value);mark();onChange?.();});
    container.querySelectorAll('[data-remove]').forEach(b=>b.onclick=()=>{points.splice(+b.dataset.remove,1);mark();pointEditor(container,points,onChange);onChange?.();});
    container.querySelectorAll('[data-up]').forEach(b=>b.onclick=()=>{const i=+b.dataset.up;if(i){[points[i-1],points[i]]=[points[i],points[i-1]];mark();pointEditor(container,points,onChange);onChange?.();}});
  }
  function actors(){
    $('#actor-count').value=draft.actors.count;
    $('#actor-list').innerHTML=draft.actors.agents.map((a,i)=>`<div class="card" data-agent="${i}"><h2>障碍机 ${i+1}</h2><div class="row"><label>车辆名称<input data-agent-key="name" value="${esc(a.name)}"></label><label>速度（m/s）<input type="number" step="0.1" data-agent-key="speed_mps" value="${a.speed_mps}"></label><label>循环次数（0=持续循环）<input type="number" data-agent-key="repeat" value="${a.repeat}"></label></div><h3>UE 出生位置（全局 NED：北、东、下，m）</h3><div class="row">${a.spawn_ned_m.map((v,j)=>`<label>${['N','E','D'][j]}<input type="number" data-spawn="${j}" value="${v}" step="0.1"></label>`).join('')}</div><h3>ENU 运动航点</h3><div data-agent-points></div><button data-add-agent>＋ 添加航点</button></div>`).join('');
    $$('#actor-list [data-agent]').forEach(box=>{
      const a=draft.actors.agents[+box.dataset.agent];
      box.querySelectorAll('[data-agent-key]').forEach(el=>el.oninput=()=>{a[el.dataset.agentKey]=el.type==='number'?(el.value===''?null:Number(el.value)):el.value;mark();});
      box.querySelectorAll('[data-spawn]').forEach(el=>el.oninput=()=>{a.spawn_ned_m[+el.dataset.spawn]=el.value===''?null:Number(el.value);mark();});
      pointEditor(box.querySelector('[data-agent-points]'),a.waypoints_enu_m);
      box.querySelector('[data-add-agent]').onclick=()=>{a.waypoints_enu_m.push(clone(a.waypoints_enu_m.at(-1)||[0,30,10]));mark();actors();};
    });
  }
  let scales={v:10,h:20};
  function chart(){
    if(!draft)return;
    const p=draft.mission.profile.points,actual=state?.telemetry?.profile_actual||[];
    const values=[...p,...actual].filter(x=>Number.isFinite(Number(x.speed_mps))&&Number.isFinite(Number(x.height_m)));
    const maxV=Math.max(4,...values.map(x=>Number(x.speed_mps)))*1.15,maxH=Math.max(10,...values.map(x=>Number(x.height_m)))*1.15;
    if(!dragging)scales={v:maxV,h:maxH};
    const x=v=>58+Number(v)/scales.v*592,y=h=>330-Number(h)/scales.h*300;
    let out='';
    for(let i=0;i<=5;i++){const v=scales.v*i/5,h=scales.h*i/5;out+=`<path d="M${x(v)} 30V330 M58 ${y(h)}H650" stroke="#2a3d51"/><text x="${x(v)}" y="350" text-anchor="middle" fill="#9fb4ca" font-size="12">${v.toFixed(1)}</text><text x="50" y="${y(h)+4}" text-anchor="end" fill="#9fb4ca" font-size="12">${h.toFixed(1)}</text>`;}
    out+='<text x="345" y="374" text-anchor="middle" fill="#c3d9eb" font-size="13">速度 / m·s⁻¹</text><text x="15" y="175" text-anchor="middle" transform="rotate(-90 15 175)" fill="#c3d9eb" font-size="13">高度 / m</text>';
    if(actual.length>1)out+=`<polyline points="${actual.map(a=>`${x(a.speed_mps)},${y(a.height_m)}`).join(' ')}" fill="none" stroke="#78ce9d" stroke-width="2" opacity=".75"/>`;
    out+=`<polyline points="${p.map(a=>`${x(a.speed_mps)},${y(a.height_m)}`).join(' ')}" fill="none" stroke="#69b9ef" stroke-width="2.5"/>`;
    p.forEach((a,i)=>out+=`<circle data-point="${i}" cx="${x(a.speed_mps)}" cy="${y(a.height_m)}" r="6" fill="#70c5fb" stroke="#173b58" stroke-width="2" style="cursor:grab"/><text x="${x(a.speed_mps)+9}" y="${y(a.height_m)-8}" fill="#d7ebff" font-size="12">${i+1}</text>`);
    $('#profile-chart').innerHTML=out;
  }
  function profileTable(){
    const p=draft.mission.profile.points;
    $('#profile-points').innerHTML=table(['点','速度 m/s','高度 m','水平距离 m','类型 / 操作'],p.map((a,i)=>`<tr><td>${i+1}</td>${['speed_mps','height_m','distance_m'].map(k=>`<td><input type="number" min="0" step="0.1" data-profile-index="${i}" data-profile-key="${k}" value="${esc(a[k])}" ${k==='distance_m'&&(!i||Math.abs(a.height_m-p[i-1].height_m)>1e-6)?'disabled':''}></td>`).join('')}<td><span data-kind="${i}">${!i?'起点':Math.abs(a.height_m-p[i-1].height_m)>1e-6?'垂直':'水平'}</span> <button data-delete-profile="${i}">×</button><button data-up-profile="${i}">↑</button></td></tr>`));
    $$('#profile-points input').forEach(el=>el.oninput=()=>{p[+el.dataset.profileIndex][el.dataset.profileKey]=el.value===''?null:Number(el.value);mark();refreshPreview();});
    $$('#profile-points input').forEach(el=>el.onchange=()=>profileTable());
    $$('[data-delete-profile]').forEach(b=>b.onclick=()=>{p.splice(+b.dataset.deleteProfile,1);mark();profileTable();refreshPreview();});
    $$('[data-up-profile]').forEach(b=>b.onclick=()=>{const i=+b.dataset.upProfile;if(i){[p[i-1],p[i]]=[p[i],p[i-1]];mark();profileTable();refreshPreview();}});
  }
  function refreshPreview(){
    chart();
    try{
      const route=previewProfile(draft.mission.profile);
      $('#compiled-route').innerHTML=table(['序号','E / m','N / m','U / m'],route.map((p,i)=>`<tr><td>${i+1}</td>${p.map(v=>`<td>${Number(v).toFixed(2)}</td>`).join('')}</tr>`));
    }catch(e){$('#compiled-route').textContent=String(e);}
    clearTimeout(previewTimer);
    previewTimer=setTimeout(async()=>{
      try{const p=await call('preview',draft);if(disposed)return;$('#validation').textContent='几何与参数校验通过；保存时还会校验传感器制动包络。';if(p.profile)$('#profile-description').textContent=`理想剖面累计时长 ${p.profile.nominal_duration_s.toFixed(1)} s（不含转向减速、起降接管和避障）`;}catch(e){if(!disposed)$('#validation').textContent=String(e.message||e);}
    },300);
  }
  function populate(){
    $$('[data-bind]').forEach(el=>{const v=get(el.dataset.bind);if(el.type==='checkbox')el.checked=v;else el.value=v;});
    $('#gnss-fault-segments').value=JSON.stringify(draft.localization?.gnss_fault.segments||[],null,2);
    optionEditors();$('#log-filter').value=draft.ui.log_filter;$('#log-errors').checked=draft.ui.log_errors;$('#log-follow').checked=draft.ui.log_follow;$('#default-distance').value=draft.ui.profile_default_distance_m;
    $('#visible').value=String(draft.ue.visible);actors();pointEditor($('#ego-points'),draft.mission.waypoints_enu_m);profileTable();mode();refreshPreview();
  }
  function mode(){const profile=draft.mission.mode==='profile';$('#waypoint-editor').hidden=profile;$('#profile-editor').hidden=!profile;}
  function updateStatus(){
    const s=state,t=s.telemetry||{},position=t.position_enu_m||[0,0,0];
    const fresh=s.ros_running && Date.now()/1000-(t.received_wall||0)<2;
    $('#connection').textContent='管理程序已连接';
    const cards=[['UE / AirSim',s.airsim_ready?'已就绪':s.ue_owned?'加载中':'未连接'],['ROS 实验',s.ros_running?'进程运行中':'已停止'],['实际地速',!fresh||t.speed_mps==null?'—':t.speed_mps.toFixed(2)+' m/s'],['ENU 高度',fresh&&t.position_enu_m?position[2].toFixed(2)+' m':'—'],['飞行阶段',t.flight?.phase||'—']];
    $('#metrics').innerHTML=cards.map(([k,v])=>`<div class="metric"><small>${k}</small><b>${esc(v)}</b></div>`).join('');
    $('#summary').innerHTML=`<p>已保存模式：${s.config.mission.mode==='profile'?'速度—高度剖面':'ENU 航点'}</p><p>障碍机：${s.config.actors.count} 架 · 控制：${s.config.run.execute?'执行':'观察'}</p><p class="path">${esc(s.active_run||'尚未启动本次 ROS 实验')}</p><p>本机 ENU：${position.map(v=>Number(v).toFixed(2)).join(', ')}</p><p>${esc(t.mission?.reason||'等待任务状态')}</p>`;
    $('#operations').innerHTML=s.operations.slice(-5).reverse().map(o=>`<p class="${o.state==='failed'?'error':''}">${esc(o.action)} · ${esc(o.state)} ${esc(o.message)} · ${Math.max(0,((o.finished_wall||Date.now()/1000)-(o.started_wall||Date.now()/1000))).toFixed(1)} s</p>`).join('')||'<p>暂无操作</p>';
    $('#processes').innerHTML=table(['进程','PID','状态','日志文件'],Object.values(s.processes).map(p=>`<tr><td>${esc(p.name)}</td><td>${p.pid}</td><td>${p.running?'运行中':'已退出 '+p.exit_code}</td><td class="path">${esc(p.log)}</td></tr>`));
    $('#diagnostics').innerHTML=table(['节点','等级','状态','频率 / 采样'],Object.entries(t.diagnostics||{}).map(([name,d])=>`<tr><td>${esc(name)}</td><td>${d.level}</td><td>${esc(d.message)}</td><td>${esc(JSON.stringify(d.values))}</td></tr>`));
    $('#runtime-details').textContent=JSON.stringify({flight:t.flight,mission:t.mission,position_enu_m:t.position_enu_m,speed_mps:t.speed_mps,telemetry_fresh:fresh},null,2);
    const actorFresh=s.ros_running&&Date.now()/1000-(t.actors_received_wall||0)<3;
    $('#actor-states').innerHTML=table(['名称','状态','航点','详情'],Object.entries(t.actors?.actors||{}).map(([name,a])=>`<tr><td>${esc(name)}</td><td>${esc(a.phase)}${actorFresh?'':'（状态已过期）'}</td><td>${esc(a.waypoint_index||'—')} / ${esc(a.waypoint_count||'—')}</td><td>${esc(JSON.stringify(a))}</td></tr>`));
    for(const op of s.operations){const before=observedOperations.get(op.id);if(before!==op.state){observedOperations.set(op.id,op.state);clientLog(`${op.action} · ${op.state} · ${op.message||''}`,op.state==='failed'?'ERROR':'INFO');}}
    evaluation(t);renderLocalization(t);renderLogs();renderCollectionStatus();
    $$('[data-op]').forEach(b=>b.disabled=b.dataset.op==='force_stop_all'?!!s.force_running:b.dataset.op==='cancel'?!s.busy:s.busy||s.force_running||(dirty&&b.dataset.op.startsWith('start')));
    $$('[data-save]').forEach(b=>b.disabled=s.busy||s.ros_running);
    if(activeTab==='mission')chart();
  }
  const localizationHistory=[];
  let localizationSession=null,localizationOrigin=null,localizationLastStamp=0,localizationClearStamp=0,localizationPaused=false,localizationView=[],localizationChartData={};
  const localizationSeries=[['3d',3,'#87d6ce'],['x',0,'#78b7ff'],['y',1,'#f0ba68'],['z',2,'#cc9bff']];
  function drawLocalizationCharts(){
    const last=localizationView.at(-1),windowS=Number($('#localization-window').value);
    const end=last?(last.stamp-localizationOrigin)*1e-9:windowS||60;
    const start=last?Math.max((localizationView[0].stamp-localizationOrigin)*1e-9,windowS?end-windowS:0):0;
    const finish=Math.max(end,start+.1),span=finish-start;
    const view=localizationView.filter(p=>(p.stamp-localizationOrigin)*1e-9>=start);
    for(const [key,index,color] of localizationSeries){
      const svg=$(`[data-error-chart="${key}"]`),values=view.map(p=>p.values[index]);
      const peak=Math.max(.05,...values.map(Math.abs))*1.15,lo=key==='3d'?0:-peak,hi=peak;
      const px=p=>62+(((p.stamp-localizationOrigin)*1e-9-start)/span)*598;
      const py=v=>194-(v-lo)/(hi-lo)*164;
      localizationChartData[key]={view,px,py,start,span};
      let markup='';
      for(let tick=0;tick<=4;tick++){
        const value=lo+(hi-lo)*tick/4,y=py(value),time=start+span*tick/4,x=62+598*tick/4;
        markup+=`<line x1="62" y1="${y}" x2="660" y2="${y}" stroke="${Math.abs(value)<1e-12?'#768ba3':'#273b50'}"/><text x="55" y="${y+4}" text-anchor="end" fill="#9fb4ca" font-size="12">${value.toFixed(3)}</text><line x1="${x}" y1="30" x2="${x}" y2="194" stroke="#273b50"/><text x="${x}" y="213" text-anchor="middle" fill="#9fb4ca" font-size="12">${time.toFixed(1)}</text>`;
      }
      markup+='<text x="8" y="17" fill="#9fb4ca" font-size="12">误差 / m</text><text x="660" y="233" text-anchor="end" fill="#9fb4ca" font-size="12">会话时间 / s</text>';
      let part=[];const segments=[];
      for(const point of view){
        if(part.length&&point.stamp-part.at(-1).stamp>1e9){segments.push(part);part=[];}
        part.push(point);
      }
      if(part.length)segments.push(part);
      markup+=segments.map(segment=>segment.length===1?`<circle cx="${px(segment[0])}" cy="${py(segment[0].values[index])}" r="2.5" fill="${color}"/>`:`<polyline data-error-line="${key}" points="${segment.map(p=>`${px(p)},${py(p.values[index])}`).join(' ')}" fill="none" stroke="${color}" stroke-width="2"/>`).join('');
      if(!view.length)markup+='<text x="350" y="110" text-anchor="middle" fill="#9fb4ca">等待有效误差数据</text>';
      svg.innerHTML=markup;
    }
  }
  function renderLocalization(t){
    const m=t.localization_metrics,s=t.localization_status||m?.estimator,c=m?.current,summary=m?.summary;
    const fresh=!!state.ros_running&&Date.now()/1000-(t.localization_metrics_received_wall||0)<2&&m?.valid&&c;
    const number=v=>Number.isFinite(v)?v.toFixed(3):'—';
    const cards=[['状态',s?.status||'等待输入'],['模式',s?.mode||'—'],...['X / 东向误差 m','Y / 北向误差 m','Z / 高度误差 m'].map((title,i)=>[title,fresh?number(c?.error_enu_m?.[i]):'过期 / 等待']),['当前三维误差 m',fresh?number(c?.error_3d_m):'过期 / 等待'],['RMSE 3D m',number(summary?.rmse_3d_m)],['P95 3D m',number(summary?.p95_3d_m)],['误差对齐覆盖率',Number.isFinite(summary?.coverage)?(summary.coverage*100).toFixed(1)+'%':'—'],['状态频率 Hz',number(s?.publish_hz)],['状态年龄 s',number(s?.state_age_s)],['GNSS 测试阶段',s?.gnss_fault_mode||'—']];
    $('#localization-metrics').innerHTML=cards.map(([k,v])=>`<div class="metric"><small>${esc(k)}</small><b>${esc(v)}</b></div>`).join('');
    $('#localization-details').textContent=JSON.stringify({summary,outage_end_errors:m?.outage_end_errors,recovery_events:m?.recovery_events,estimator:s},null,2);
    const stamp=c?.stamp_ns;
    if(m?.session_id&&(m.session_id!==localizationSession||(fresh&&Number.isFinite(stamp)&&stamp<localizationLastStamp))){
      localizationSession=m.session_id;localizationOrigin=Number.isFinite(m.origin_stamp_ns)?m.origin_stamp_ns:null;
      localizationHistory.length=0;localizationView=[];localizationLastStamp=0;localizationClearStamp=0;
    }
    if(fresh){
      for(const row of [...(m.recent_errors||[]),c]){
        if(!Number.isFinite(row?.stamp_ns)||!Array.isArray(row.error_enu_m)||row.error_enu_m.length!==3||!row.error_enu_m.every(Number.isFinite)||!Number.isFinite(row.error_3d_m)||row.error_3d_m<0)continue;
        if(row.stamp_ns<=Math.max(localizationLastStamp,localizationClearStamp))continue;
        if(localizationOrigin===null)localizationOrigin=row.stamp_ns;
        if(localizationHistory.length&&row.stamp_ns-localizationHistory.at(-1).stamp<.099e9)continue;
        localizationHistory.push({stamp:row.stamp_ns,values:[...row.error_enu_m,row.error_3d_m],phase:row.phase||''});
        localizationLastStamp=row.stamp_ns;
      }
      while(localizationHistory.length>6000||localizationHistory.length>1&&localizationHistory.at(-1).stamp-localizationHistory[0].stamp>600e9)localizationHistory.shift();
    }
    if(!localizationPaused)localizationView=localizationHistory.slice();
    for(const [key,index] of localizationSeries)$(`[data-localization-value="${key}"]`).textContent=fresh?number(index===3?c.error_3d_m:c.error_enu_m?.[index])+' m':'—';
    $('#localization-curve-status').textContent=`${localizationPaused?'曲线已暂停，后台继续接收':fresh?'实时更新':'数据过期 / 等待，保留历史'} · 已缓存 ${localizationHistory.length} 点 · X=东，Y=北，Z=上`;
    if(activeTab==='localization')drawLocalizationCharts();
  }
  $('#localization-window').onchange=drawLocalizationCharts;
  $('#localization-pause').onclick=()=>{localizationPaused=!localizationPaused;$('#localization-pause').textContent=localizationPaused?'继续曲线':'暂停曲线';renderLocalization(state?.telemetry||{});};
  $('#localization-clear').onclick=()=>{localizationClearStamp=state?.telemetry?.localization_metrics?.current?.stamp_ns||localizationLastStamp;localizationHistory.length=0;localizationView=[];drawLocalizationCharts();renderLocalization(state?.telemetry||{});};
  $('#localization-export').onclick=()=>{
    const rows=['session_id,stamp_ns,session_time_s,error_x_m,error_y_m,error_z_m,error_3d_m,gnss_phase',...localizationHistory.map(p=>[JSON.stringify(localizationSession),(p.stamp).toFixed(0),((p.stamp-localizationOrigin)*1e-9).toFixed(6),...p.values,p.phase].join(','))];
    const blob=new Blob(['\uFEFF'+rows.join('\r\n')],{type:'text/csv;charset=utf-8'}),url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download='evtol-localization-errors.csv';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
  };
  for(const [key,index] of localizationSeries){
    const svg=$(`[data-error-chart="${key}"]`),label=$(`[data-localization-hover="${key}"]`);
    svg.addEventListener('pointermove',event=>{
      const data=localizationChartData[key];if(!data?.view.length)return;
      const box=svg.getBoundingClientRect();if(!box.width)return;
      const x=Math.max(62,Math.min(660,(event.clientX-box.left)/box.width*680));
      const target=(data.start+(x-62)/598*data.span)*1e9+localizationOrigin;
      const point=data.view.reduce((a,b)=>Math.abs(a.stamp-target)<Math.abs(b.stamp-target)?a:b);
      label.textContent=`t=${((point.stamp-localizationOrigin)*1e-9).toFixed(3)} s · 误差=${point.values[index].toFixed(4)} m · GNSS=${point.phase||'—'}`;
    });
  }
  $('#gnss-fault-segments').onchange=()=>{try{const value=JSON.parse($('#gnss-fault-segments').value);if(!Array.isArray(value))throw new Error('应为数组');set('localization.gnss_fault.segments',value);$('#gnss-fault-validation').textContent='时段已编辑；保存时检查范围与重叠。';}catch(e){$('#gnss-fault-validation').textContent='未采用：'+e.message;}};
  $('#gnss-fault-example').onclick=()=>{set('localization.gnss_fault.segments',[{start_s:40,end_s:60,mode:'dropout',bias_enu_m:[0,0,0]}]);$('#gnss-fault-segments').value=JSON.stringify(draft.localization.gnss_fault.segments,null,2);};
  function collectionValue(path){return path.split('.').reduce((v,k)=>v?.[k],collectionDraft);}
  function collectionSet(path,value){const keys=path.split('.'),last=keys.pop();keys.reduce((v,k)=>v[k],collectionDraft)[last]=value;}
  function collectionForm(){
    if(!collectionDraft)return;
    $$('[data-collect]').forEach(el=>{const v=collectionValue(el.dataset.collect);if(el.type==='checkbox')el.checked=v;else el.value=v;});
    $('#collect-detection').checked=collectionDraft.dataset.labels.includes('detection');$('#collect-segmentation').checked=collectionDraft.dataset.labels.includes('segmentation');
    $('#collect-classes').innerHTML=[[1,'飞机'],[2,'建筑'],[3,'行人'],[4,'地面'],[5,'跑道'],[6,'无人机']].map(([id,name])=>`<label class="check"><input data-collect-class="${id}" type="checkbox" ${collectionDraft.dataset.class_ids.includes(id)?'checked':''}>${name}（${id}）</label>`).join('');
    $$('[data-collect-class]').forEach(el=>el.onchange=()=>{collectionDraft.dataset.class_ids=$$('[data-collect-class]').filter(e=>e.checked).map(e=>Number(e.dataset.collectClass));});
    collectionTopicList();
  }
  function visibleCollectionTopics(){
    const filter=$('#collect-topic-filter').value.toLowerCase(),byName=new Map(collectionTopics.map(t=>[t.name,t]));
    for(const name of collectionDraft?.bag.topics||[])if(!byName.has(name))byName.set(name,{name,types:[],publishers:0});
    return [...byName.values()].filter(t=>t.name.toLowerCase().includes(filter)).sort((a,b)=>a.name.localeCompare(b.name));
  }
  function collectionTopicList(){
    if(!collectionDraft)return;
    $('#collect-topics').innerHTML=visibleCollectionTopics().map(t=>`<label class="check"><input type="checkbox" data-collect-topic="${esc(t.name)}" ${collectionDraft.bag.topics.includes(t.name)?'checked':''}><span>${esc(t.name)} <small>${esc(t.types.join(', '))}${t.publishers?'':'（当前未发布）'}</small></span></label>`).join('')||'<p>暂无已发布话题，启动 ROS 后刷新。</p>';
    $$('[data-collect-topic]').forEach(el=>el.onchange=()=>{const chosen=new Set(collectionDraft.bag.topics);el.checked?chosen.add(el.dataset.collectTopic):chosen.delete(el.dataset.collectTopic);collectionDraft.bag.topics=[...chosen];$('#collect-topic-count').textContent='已选择 '+chosen.size+' 个话题';});
    $('#collect-topic-count').textContent='已选择 '+collectionDraft.bag.topics.length+' 个话题';
  }
  function renderCollectionStatus(){
    if(!collectionDraft&&state?.collection_config){collectionDraft=clone(state.collection_config);collectionForm();}
    const c=state?.collection||{state:'idle'},running=c.running||false;
    $('#collect-status').innerHTML=[['状态',c.state||'idle'],['已收尾',c.finalized?'是':'否'],['训练帧',c.dataset_frames||0],['Bag 消息数',Object.values(c.bag_messages||{}).reduce((a,b)=>a+b,0)],['I/O 队列丢弃',c.queue_dropped||0],['未配对图像',c.unmatched_frames||0],['采样跳过',c.sampled_out||0]].map(([k,v])=>`<div class="metric"><small>${k}</small><b>${esc(v)}</b></div>`).join('');
    $('#collect-path').textContent=c.output_dir||'尚未采集';$('#collect-details').textContent=JSON.stringify(c,null,2);
    for(const id of ['collect-bag','collect-dataset'])$('#'+id).disabled=!collectionDraft||running||state?.busy||state?.force_running;
    $('#collect-stop').disabled=!running||state?.busy||state?.force_running;
  }
  $$('[data-collect]').forEach(el=>el.oninput=()=>{if(!collectionDraft)return;collectionSet(el.dataset.collect,el.type==='checkbox'?el.checked:el.type==='number'?(el.value===''?null:Number(el.value)):el.value);});
  for(const task of ['detection','segmentation'])$('#collect-'+task).onchange=()=>{if(collectionDraft)collectionDraft.dataset.labels=['detection','segmentation'].filter(t=>$('#collect-'+t).checked);};
  $('#collect-topic-filter').oninput=collectionTopicList;
  $('#collect-refresh').onclick=async()=>{try{collectionTopics=await call('collection_topics');collectionTopicList();notice('已刷新 ROS 已发布话题');}catch(e){notice(e.message,true);}};
  $('#collect-select-visible').onclick=()=>{if(collectionDraft){collectionDraft.bag.topics=[...new Set([...collectionDraft.bag.topics,...visibleCollectionTopics().map(t=>t.name)])];collectionTopicList();}};
  $('#collect-select-none').onclick=()=>{if(collectionDraft){collectionDraft.bag.topics=[];collectionTopicList();}};
  $('#collect-add-topic').onclick=()=>{if(!collectionDraft)return;const name=$('#collect-custom-topic').value.trim();if(!/^\/(?:[A-Za-z_][A-Za-z0-9_]*)(?:\/[A-Za-z_][A-Za-z0-9_]*)*$/.test(name))return notice('请输入有效的绝对 ROS 话题名称',true);collectionDraft.bag.topics=[...new Set([...collectionDraft.bag.topics,name])];$('#collect-custom-topic').value='';collectionTopicList();};
  $('#collect-save').onclick=async()=>{try{collectionDraft=await call('save_collection',collectionDraft);notice('采集参数已保存，下一次采集使用；当前采集不变');}catch(e){notice(e.message,true);}};
  $('#collect-export').onclick=async()=>{try{download(await call('validate_collection',collectionDraft),'evtol-collection-config.json');}catch(e){notice(e.message,true);}};
  $('#collect-import').onclick=()=>$('#collect-file').click();
  $('#collect-file').onchange=async()=>{try{const file=$('#collect-file').files?.[0];if(!file)return;if(file.size>1048576)throw new Error('配置文件不能超过 1 MB');collectionDraft=await call('validate_collection',JSON.parse(await file.text()));collectionForm();notice('采集配置已载入，尚未保存或启动');}catch(e){notice(e.message,true);}finally{$('#collect-file').value='';}};
  for(const mode of ['bag','dataset'])$('#collect-'+mode).onclick=async()=>{clientLog('按钮按下：开始采集 '+mode);try{const result=await call('start_collection',{mode,config:collectionDraft});clientLog('采集请求已接受，操作 ID '+result.operation_id);notice('已提交采集请求，请查看状态和日志');}catch(e){notice(e.message,true);clientLog('采集启动失败：'+e.message,'ERROR');}};
  $('#collect-stop').onclick=async()=>{clientLog('按钮按下：停止采集并保存');try{const result=await call('stop_collection');notice('正在停止并保存，操作 ID '+result.operation_id);}catch(e){notice(e.message,true);}};
  $$('[data-tab]').forEach(b=>b.onclick=()=>{activeTab=b.dataset.tab;$$('[data-page]').forEach(p=>p.hidden=p.dataset.page!==activeTab);$$('[data-tab]').forEach(t=>t.classList.toggle('active',t===b));if(activeTab==='mission')chart();if(activeTab==='localization')drawLocalizationCharts();});
  $$('[data-tab]')[0].classList.add('active');
  $$('[data-bind]').forEach(el=>el.oninput=()=>{if(!draft)return;set(el.dataset.bind,el.type==='checkbox'?el.checked:el.type==='number'?(el.value===''?null:Number(el.value)):el.value);if(el.dataset.bind==='mission.mode')mode();if(el.dataset.bind.startsWith('mission.'))refreshPreview();});
  $('#log-filter').oninput=()=>{draft.ui.log_filter=$('#log-filter').value;mark();renderLogs();};
  $('#log-errors').onchange=()=>{draft.ui.log_errors=$('#log-errors').checked;mark();renderLogs();};
  $('#log-follow').onchange=()=>{draft.ui.log_follow=$('#log-follow').checked;mark();};
  $('#default-distance').onchange=()=>{draft.ui.profile_default_distance_m=Number($('#default-distance').value);mark();};
  $('#visible').onchange=()=>set('ue.visible',$('#visible').value==='true');
  $('#actor-count').onchange=()=>{const count=Number($('#actor-count').value);if(!Number.isInteger(count)||count<0||count>64)return notice('障碍机数量应为 0–64',true);while(draft.actors.agents.length<count){const i=draft.actors.agents.length;draft.actors.agents.push({name:'Drone'+(i+2),speed_mps:2,spawn_ned_m:[30+i*10,0,0],waypoints_enu_m:[[0,30+i*10,10],[10,30+i*10,10]],repeat:1});}draft.actors.agents.length=count;draft.actors.count=count;mark();actors();};
  $('#add-ego').onclick=()=>{draft.mission.waypoints_enu_m.push(clone(draft.mission.waypoints_enu_m.at(-1)||[0,0,3]));mark();pointEditor($('#ego-points'),draft.mission.waypoints_enu_m);};
  $('#add-profile').onclick=()=>{const last=draft.mission.profile.points.at(-1)||{speed_mps:0,height_m:0};draft.mission.profile.points.push({...last,distance_m:Number($('#default-distance').value)});mark();profileTable();refreshPreview();};
  $('#example-profile').onclick=()=>{draft.mission.profile.points=[{speed_mps:0,height_m:0,distance_m:100},{speed_mps:2,height_m:10,distance_m:100},{speed_mps:2,height_m:10,distance_m:30},{speed_mps:0,height_m:0,distance_m:100}];mark();profileTable();refreshPreview();};
  // Match the coordinate mapping to the rendered SVG even in narrow panels.
  $('#profile-chart').setAttribute('preserveAspectRatio','none');
  const coords=event=>{const r=$('#profile-chart').getBoundingClientRect();return {speed_mps:Math.max(0,Math.round(((event.clientX-r.left)/r.width*680-58)/592*scales.v*10)/10),height_m:Math.max(0,Math.round((330-(event.clientY-r.top)/r.height*380)/300*scales.h*10)/10)};};
  $('#profile-chart').addEventListener('pointerdown',event=>{const circle=event.target.closest?.('[data-point]');if(circle){dragging={index:+circle.dataset.point};$('#profile-chart').setPointerCapture(event.pointerId);}else if($('#click-add').checked){draft.mission.profile.points.push({...coords(event),distance_m:Number($('#default-distance').value)});mark();profileTable();refreshPreview();}});
  $('#profile-chart').addEventListener('pointermove',event=>{if(dragging){Object.assign(draft.mission.profile.points[dragging.index],coords(event));mark();profileTable();chart();}});
  $('#profile-chart').addEventListener('pointerup',()=>{if(dragging){dragging=null;refreshPreview();}});
  $('#profile-chart').addEventListener('pointercancel',()=>{dragging=null;refreshPreview();});
  async function reload(){clientLog('按钮按下：重新读取参数');state=await call('status');draft=clone(state.config);dirty=false;$('#dirty').textContent='';populate();updateStatus();notice('已读取保存的配置');}
  $('#reload').onclick=()=>reload().catch(e=>notice(e.message,true));
  $$('[data-save]').forEach(b=>b.onclick=async()=>{clientLog('按钮按下：保存参数，正在校验并提交');try{state=await call('save',draft);draft=clone(state.config);dirty=false;$('#dirty').textContent='';populate();updateStatus();notice('配置已保存；下次启动使用该配置');clientLog('参数保存成功');}catch(e){clientLog('保存失败：'+e.message,'ERROR');notice(e.message,true);}});
  $$('[data-op]').forEach(b=>b.onclick=async()=>{clientLog('按钮按下：'+b.textContent);try{if(dirty&&b.dataset.op.startsWith('start'))throw new Error('请先保存参数');const result=await call(b.dataset.op);clientLog('请求已接受：'+b.dataset.op+'，操作 ID '+result.operation_id+'，等待实际执行结果');notice('已提交 '+b.textContent+'，请查看最近操作和运行状态');}catch(e){clientLog('请求失败：'+e.message,'ERROR');notice(e.message,true);}});
  async function poll(){if(polling||disposed)return;polling=true;try{state=await call('status');if(disposed)return;if(!draft){draft=clone(state.config);populate();notice('管理程序已连接');}updateStatus();}catch(e){if(!disposed){$('#connection').textContent='连接中断';notice(e.message,true);}}finally{polling=false;}}
  poll();const interval=setInterval(poll,250);
  return ()=>{disposed=true;clearInterval(interval);clearTimeout(previewTimer);host.removeChild(shell);};
}
