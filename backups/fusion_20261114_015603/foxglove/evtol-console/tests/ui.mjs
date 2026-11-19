import assert from 'node:assert/strict';
import {JSDOM} from 'jsdom';
import {mountConsole,previewProfile} from '../../../src/evtol_console/evtol_console/web/app.js';

const dom=new JSDOM('<div id="panel"></div>',{url:'http://127.0.0.1:9040'});
globalThis.document=dom.window.document;
const wait=ms=>new Promise(r=>setTimeout(r,ms));
const config={version:1,ue:{editor:'editor.exe',project:'scene.uproject',settings:'airsim.json',visible:true,width:1280,height:720},
  actors:{count:0,agents:[],pedestrians_enabled:false},
  mission:{mode:'profile',speed_mps:2,max_speed_mps:4,max_acceleration_mps2:1,land_at_last:true,
    waypoints_enu_m:[[0,0,3]],profile:{heading_deg:0,points:[{speed_mps:0,height_m:0,distance_m:100},{speed_mps:2,height_m:10,distance_m:100},{speed_mps:2,height_m:10,distance_m:30},{speed_mps:0,height_m:0,distance_m:100}]}},
  run:{execute:false,actors:false,evaluate:true,new_session:true}};
let state={config,processes:{},busy:false,airsim_ready:false,ue_owned:false,ros_running:false,operations:[],logs:[],telemetry:{}};
const requests=[];
const dispose=mountConsole(document.querySelector('#panel'),async request=>{
  requests.push(structuredClone(request));
  if(request.action==='preview')return {ok:true,data:{waypoints_enu_m:previewProfile(request.payload.mission.profile)}};
  if(request.action==='save'){state.config=structuredClone(request.payload);return {ok:true,data:state};}
  if(request.action==='status')return {ok:true,data:state};
  return {ok:true,data:{operation_id:'fixture'}};
});
const $=s=>document.querySelector(s);
const input=(element,value)=>{element.value=String(value);element.dispatchEvent(new dom.window.Event('input',{bubbles:true}));};
try {
  await wait(25);
  assert.equal(document.querySelectorAll('[data-tab]').length,5);
  $('[data-tab="mission"]').click();
  assert.equal($('#profile-chart').querySelectorAll('circle').length,4);
  input($('[data-profile-index="2"][data-profile-key="distance_m"]'),50);
  await wait(350);
  assert.match($('#compiled-route').textContent,/50.00/);
  assert.deepEqual(previewProfile(config.mission.profile),[[0,0,0],[0,0,10],[0,30,10],[0,30,0]]);
  $('#actor-count').value='2';$('#actor-count').dispatchEvent(new dom.window.Event('change'));
  assert.equal(document.querySelectorAll('[data-agent]').length,2);
  input($('[data-agent="0"] [data-agent-key="speed_mps"]'),3.5);
  $('[data-save]').click();await wait(25);
  const save=requests.findLast(r=>r.action==='save');
  assert.equal(save.payload.actors.count,2);assert.equal(save.payload.actors.agents[0].speed_mps,3.5);
  assert.equal(save.payload.mission.profile.points[2].distance_m,50);
  $('[data-tab="ue"]').click();$('#visible').value='false';$('#visible').dispatchEvent(new dom.window.Event('change'));
  $('[data-save]').click();await wait(25);
  assert.equal(requests.findLast(r=>r.action==='save').payload.ue.visible,false);
  $('[data-op="start_ue"]').click();await wait(25);
  assert.equal(requests.findLast(r=>r.action==='start_ue').action,'start_ue');
  assert.match($('#logs').textContent,/按钮按下/);
  assert.match($('#logs').textContent,/请求已接受/);
  state.busy=true;state.operations=[{id:'blocked',action:'stop_all',state:'running',message:'waiting for actors',started_wall:Date.now()/1000}];
  await wait(1100);
  assert.equal($('[data-op="force_stop_all"]').disabled,false,'Force button must remain available during blocked cleanup');
  $('[data-op="force_stop_all"]').click();await wait(25);
  assert.equal(requests.findLast(r=>r.action==='force_stop_all').action,'force_stop_all');
  assert.match($('#logs').textContent,/waiting for actors/);
  $('#log-filter').value='not-a-source';$('#log-filter').dispatchEvent(new dom.window.Event('input'));
  assert.equal($('#logs').textContent,'');$('#log-filter').value='';$('#log-filter').dispatchEvent(new dom.window.Event('input'));
  state.busy=false;
  $('[data-tab="mission"]').click();
  const chart=$('#profile-chart');chart.getBoundingClientRect=()=>({left:0,top:0,width:680,height:380});
  $('#click-add').checked=true;
  chart.dispatchEvent(new dom.window.MouseEvent('pointerdown',{bubbles:true,clientX:200,clientY:120}));
  assert.equal(chart.querySelectorAll('circle').length,5);
  chart.setPointerCapture=()=>{};
  chart.querySelector('[data-point="1"]').dispatchEvent(new dom.window.MouseEvent('pointerdown',{bubbles:true,clientX:200,clientY:120}));
  chart.dispatchEvent(new dom.window.MouseEvent('pointermove',{bubbles:true,clientX:250,clientY:140}));
  chart.dispatchEvent(new dom.window.MouseEvent('pointerup',{bubbles:true,clientX:250,clientY:140}));
  assert.notEqual(Number($('[data-profile-index="1"][data-profile-key="speed_mps"]').value),2);
  assert.equal(document.querySelectorAll('main').length,1);
  console.log('PASS: tabs, actor editing, profile table/plot, click/drag, saved payload, and launch service dispatch');
} finally {dispose();dom.window.close();}
