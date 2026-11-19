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
  let draft=null,state=null,disposed=false,dirty=false,polling=false,previewTimer=null,dragging=null,activeTab='overview',clientEvents=[],observedOperations=new Map();
  const shell=document.createElement('div');shell.className='evtol';
  shell.innerHTML=`<style>${css}</style><header><h1>EVTOL 仿真控制台</h1><small>Windows · ROS 2 · UE / AirSim</small><span class="grow"></span><button class="danger" data-op="force_stop_all">强制结束 ROS + UE</button><span class="badge" id="connection">正在连接</span></header>
  <nav>${[['overview','运行总览'],['ue','UE 启动'],['actors','障碍机'],['mission','本机飞行'],['monitor','监控 / 日志']].map(([id,t])=>`<button data-tab="${id}">${t}</button>`).join('')}</nav>
  <main>
  <section data-page="overview"><div class="status-grid" id="metrics"></div><div class="grid" style="margin-top:14px"><div class="card"><h2>独立启动与停止</h2><p>① 编辑参数并保存 → ② 启动 UE，等待 AirSim 就绪 → ③ 启动 ROS 实验。</p><div class="buttons"><button class="primary" data-op="start_ue">启动 UE</button><button data-op="stop_ue">关闭 UE</button><button class="primary" data-op="start_ros">启动 ROS 实验</button><button data-op="stop_ros">停止 ROS</button><button class="danger" data-op="stop_all">停止全部</button><button data-op="cancel">取消当前启动</button></div><h3>ROS 启动选项</h3><label class="check"><input type="checkbox" data-bind="run.execute">执行飞行控制（关闭时仅观察）</label><label class="check"><input type="checkbox" data-bind="run.actors">启动障碍机 / 行人控制</label><label class="check"><input type="checkbox" data-bind="run.evaluate">发布目标真值并记录定位评估</label><label class="check"><input type="checkbox" data-bind="run.new_session">启动 ROS 时建立新的 ENU 原点会话</label><p class="hint">停止 ROS 会请求控制器停止和障碍机清理；关闭 UE 必须先停止 ROS。界面停止按钮仅管理本控制台启动的进程。</p></div><div class="card"><h2>本次运行</h2><div id="summary"></div><h3>最近操作</h3><div id="operations"></div></div></div></section>
  <section data-page="ue" hidden><div class="card"><h2>UE 程序与工程</h2><label>UE Editor 程序路径（.exe）<input data-bind="ue.editor" placeholder="D:\\Software\\Epic\\UE_4.27\\Engine\\Binaries\\Win64\\UE4Editor.exe"></label><label>UE 工程路径（.uproject）<input data-bind="ue.project"></label><label>AirSim 基础配置（项目相对路径或完整路径）<input data-bind="ue.settings"></label><div class="row"><label>运行方式<select id="visible"><option value="true">可视化窗口</option><option value="false">后台渲染</option></select></label><label>窗口宽度<input type="number" data-bind="ue.width" min="320" max="7680"></label><label>窗口高度<input type="number" data-bind="ue.height" min="240" max="4320"></label></div><p>相机和雷达仍使用原有共享内存设置。后台模式继续生成传感器数据；窗口尺寸不改变采集图像的分辨率。</p><div class="buttons"><button data-save class="primary">保存全部参数</button><button data-op="start_ue">启动 UE</button><button data-op="stop_ue">关闭 UE</button></div></div></section>
  <section data-page="actors" hidden><div class="card"><h2>障碍机场景</h2><div class="row"><label>障碍机数量<input id="actor-count" type="number" min="0" max="32"></label><label class="check"><input type="checkbox" data-bind="actors.pedestrians_enabled">同时启用原有行人配置</label></div><p>航点使用本机初始点为原点的 ENU 米坐标。数量决定 UE 中实际创建的无人机数量；修改后需保存并重新启动 UE 和 ROS。</p></div><div id="actor-list"></div><button data-save class="primary">保存全部参数</button></section>
  <section data-page="mission" hidden><div class="card"><h2>本机飞行模式</h2><div class="row"><label>模式<select data-bind="mission.mode" id="flight-mode"><option value="waypoints">ENU 航点 + 飞行速度</option><option value="profile">速度—高度剖面</option></select></label><label>航点模式速度（m/s）<input type="number" data-bind="mission.speed_mps" min="0.05" step="0.1"></label><label>总速度上限（m/s）<input type="number" data-bind="mission.max_speed_mps" min="0.1" max="30" step="0.1"></label><label>最大加速度（m/s²）<input type="number" data-bind="mission.max_acceleration_mps2" min="0.05" max="5" step="0.1"></label></div><label class="check"><input type="checkbox" data-bind="mission.land_at_last">末点在地面时自动降落</label></div>
  <div id="waypoint-editor" class="card"><h2>本机 ENU 航点</h2><div id="ego-points"></div><button id="add-ego">＋ 添加航点</button></div>
  <div id="profile-editor"><div class="split"><div class="card"><h2>有序剖面点</h2><label>水平航向（°，0=北，90=东）<input id="heading" data-bind="mission.profile.heading_deg" type="number" step="1"></label><div id="profile-points"></div><div class="buttons"><button id="add-profile">＋ 添加剖面点</button><button id="example-profile">加载示例</button></div><p>每行顺序连接。等高：水平向前飞行该行的距离；高度变化：只垂直升降，该行水平距离忽略。起点和终点速度必须为 0。</p><p class="hint">横轴速度是三维地速大小。水平 / 垂直转向点会按安全要求减速，避障也可能改变实际剖面。</p></div><div class="card"><h2>速度—高度预览</h2><div class="row"><label class="check"><input id="click-add" type="checkbox">点击图中添加点</label><label>新增等高段默认距离（m）<input id="default-distance" type="number" value="100" min="0.05"></label></div><svg id="profile-chart" viewBox="0 0 680 380" aria-label="速度高度图"></svg><p>蓝线：输入剖面；绿线：实际飞行。点按输入顺序连接，可拖动蓝色点，数值表同步更新。</p><div id="profile-description" class="subtle"></div></div></div><div class="card"><h2>转换后的 ENU 航点</h2><div id="compiled-route"></div><div id="validation" class="hint"></div></div></div>
  <div class="buttons"><button class="primary" data-save>校验并保存全部参数</button><button id="reload">重新读取已保存参数</button></div></section>
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
  const mark=()=>{dirty=true;$('#dirty').textContent='未保存';};
  const get=(path)=>path.split('.').reduce((o,k)=>o[k],draft);
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
    renderLogs();
    $$('[data-op]').forEach(b=>b.disabled=b.dataset.op==='force_stop_all'?!!s.force_running:b.dataset.op==='cancel'?!s.busy:s.busy||s.force_running||(dirty&&b.dataset.op.startsWith('start')));
    $$('[data-save]').forEach(b=>b.disabled=s.busy||s.ros_running);
    if(activeTab==='mission')chart();
  }
  $$('[data-tab]').forEach(b=>b.onclick=()=>{activeTab=b.dataset.tab;$$('[data-page]').forEach(p=>p.hidden=p.dataset.page!==activeTab);$$('[data-tab]').forEach(t=>t.classList.toggle('active',t===b));if(activeTab==='mission')chart();});
  $$('[data-tab]')[0].classList.add('active');
  $$('[data-bind]').forEach(el=>el.oninput=()=>{if(!draft)return;set(el.dataset.bind,el.type==='checkbox'?el.checked:el.type==='number'?(el.value===''?null:Number(el.value)):el.value);if(el.dataset.bind==='mission.mode')mode();if(el.dataset.bind.startsWith('mission.'))refreshPreview();});
  $('#visible').onchange=()=>set('ue.visible',$('#visible').value==='true');
  $('#actor-count').onchange=()=>{const count=Number($('#actor-count').value);if(!Number.isInteger(count)||count<0||count>32)return notice('障碍机数量应为 0–32',true);while(draft.actors.agents.length<count){const i=draft.actors.agents.length;draft.actors.agents.push({name:'Drone'+(i+2),speed_mps:2,spawn_ned_m:[30+i*10,0,0],waypoints_enu_m:[[0,30+i*10,10],[10,30+i*10,10]],repeat:1});}draft.actors.agents.length=count;draft.actors.count=count;mark();actors();};
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
  poll();const interval=setInterval(poll,1000);
  return ()=>{disposed=true;clearInterval(interval);clearTimeout(previewTimer);host.removeChild(shell);};
}
