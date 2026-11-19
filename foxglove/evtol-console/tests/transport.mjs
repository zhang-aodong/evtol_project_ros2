/* Tests the actual Foxglove service wire protocol, not a mocked panel context. */
import assert from 'node:assert/strict';
const [httpPort,wsPort,live]=process.argv.slice(2);
const response=await fetch(`http://127.0.0.1:${httpPort}/api/status`);
const initial=await response.json();assert.equal(initial.ok,true);
const pending=new Map();let nextId=1,serviceId=null,serverInfo=null;
const socket=new WebSocket(`ws://127.0.0.1:${wsPort}`,['foxglove.sdk.v1','foxglove.websocket.v1']);socket.binaryType='arraybuffer';
const ready=new Promise((resolve,reject)=>{
  const timer=setTimeout(()=>reject(new Error('No console service advertisement')),10000);
  socket.addEventListener('error',e=>reject(e.error||new Error(e.message||'WebSocket handshake failed')));
  socket.addEventListener('message',event=>{
    if(typeof event.data==='string'){
      const msg=JSON.parse(event.data);
      if(msg.op==='serverInfo')serverInfo=msg;
      if(msg.op==='advertiseServices'){
        const svc=msg.services.find(s=>s.name==='/simulation/console');
        if(svc){serviceId=svc.id;clearTimeout(timer);resolve();}
      }
      if(msg.op==='serviceCallFailure'){pending.get(msg.callId)?.reject(new Error(msg.message));pending.delete(msg.callId);}
    }else{
      const bytes=new Uint8Array(event.data);if(bytes[0]!==3)return;
      const view=new DataView(event.data);const id=view.getUint32(5,true),length=view.getUint32(9,true);
      const result=JSON.parse(new TextDecoder().decode(bytes.slice(13+length)));
      pending.get(id)?.resolve(result);pending.delete(id);
    }
  });
});
async function call(action,payload={}){
  const id=nextId++,data=new TextEncoder().encode(JSON.stringify({action,payload}));
  const bytes=new Uint8Array(17+data.length);const view=new DataView(bytes.buffer);
  bytes[0]=2;view.setUint32(1,serviceId,true);view.setUint32(5,id,true);view.setUint32(9,4,true);
  bytes.set(new TextEncoder().encode('json'),13);bytes.set(data,17);
  const result=await new Promise((resolve,reject)=>{
    const timer=setTimeout(()=>{pending.delete(id);reject(new Error('Service request timed out'));},15000);
    pending.set(id,{resolve:v=>{clearTimeout(timer);resolve(v);},reject:e=>{clearTimeout(timer);reject(e);}});
    socket.send(bytes);
  });
  assert.equal(result.ok,true,result.error);return result.data;
}
async function operation(action){
  const result=await call(action);const deadline=Date.now()+150000;
  while(Date.now()<deadline){
    const status=await call('status'),op=status.operations.find(o=>o.id===result.operation_id);
    if(op?.state==='failed')throw new Error(action+': '+op.message);
    if(op?.state==='done'){console.log(action+': PASS');return status;}
    await new Promise(r=>setTimeout(r,500));
  }
  throw new Error(action+' operation timed out');
}
try{
  await ready;assert.ok(serverInfo.capabilities.includes('services'));
  const state=await call('status');assert.equal(state.config.version,2);
  const exported=await call('export_config');assert.deepEqual(exported,state.config);
  const imported=structuredClone(exported);imported.experiment.radius_m=15;
  const draft=await call('import_config',imported);assert.equal(draft.experiment.radius_m,15);
  assert.equal((await call('status')).config.experiment.radius_m,exported.experiment.radius_m);
  const preview=await call('preview',state.config);assert.ok(preview.waypoints_enu_m.length);
  console.log('PASS: HTTP status + real Foxglove JSON service status/preview');
  if(live==='force'){
    await call('stop_all');
    await new Promise(r=>setTimeout(r,300));
    assert.equal((await call('status')).busy,true,'Normal cleanup must be pending');
    const started=Date.now();await operation('force_stop_all');
    assert.ok(Date.now()-started<10000,'Force waited for the normal cleanup timeout');
    let stopped=await call('status');
    const deadline=Date.now()+3000;
    while(stopped.busy&&Date.now()<deadline){await new Promise(r=>setTimeout(r,50));stopped=await call('status');}
    assert.equal(stopped.busy,false);
    assert.ok(Object.values(stopped.processes).every(p=>!p.running));
    assert.equal(stopped.telemetry.actors.actors.ProbeDrone.phase,'landing');
    console.log('PASS: real ROS actor status + force preempts cleanup + owned processes stopped');
  }
  if(live==='live'){
    await operation('start_ue');
    const running=await operation('start_ros');assert.equal(running.config.run.execute,false);assert.equal(running.config.run.actors,false);
    const deadline=Date.now()+45000;let observed=null;
    while(Date.now()<deadline){
      observed=await call('status');
      if(observed.telemetry.position_enu_m&&observed.telemetry.flight?.phase==='shadow')break;
      await new Promise(r=>setTimeout(r,1000));
    }
    assert.ok(observed?.telemetry.position_enu_m,'No real state telemetry from ROS');
    assert.equal(observed.telemetry.flight?.phase,'shadow');
    assert.ok(Object.values(observed.processes).filter(p=>p.running).length>=12);
    console.log('PASS: live UE + full ROS shadow stack + real ENU state telemetry');
    await operation('stop_ros');await operation('stop_ue');
  }
}finally{socket.close();}
