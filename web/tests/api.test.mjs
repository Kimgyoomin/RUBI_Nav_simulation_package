import test from 'node:test';
import assert from 'node:assert/strict';
import {mkdtempSync,rmSync} from 'node:fs';
import {tmpdir} from 'node:os';
import {join} from 'node:path';
import {once} from 'node:events';
import {createService} from '../api/server.mjs';

test('responses are validated, idempotent, persistent and exported only to researchers',async()=>{
  const dir=mkdtempSync(join(tmpdir(),'rubi-api-'));let service;
  const config={dbPath:join(dir,'test.sqlite'),allowedOrigins:['http://localhost:5173'],adminToken:'test-token'};
  const start=async()=>{service=createService(config);service.server.listen(0,'127.0.0.1');await once(service.server,'listening');return `http://127.0.0.1:${service.server.address().port}/api`;};
  let base=await start();
  const run=route=>({completed:true,route,scenarioId:'s1',duration:20,actualLength:6,plannedLength:6});
  const p={schemaVersion:1,submissionId:'response-1',participantId:'participant-1',experimentId:'rubi-hpp-pilot-v1',studyStatus:'draft',trialId:'trial-1',consent:true,choice:'direct',scenario:{id:'s1',height:.05,detour:.8,speed:.3},presentation:{a:'direct',b:'detour'},runs:{direct:run('direct'),detour:run('detour')},previewCoverage:{direct:1,detour:1},viewTime:{direct:20,detour:20},decisionMs:150,elapsedTrialMs:42000,hashes:{'rubi.xml':'a'.repeat(64),'encoder.onnx':'b'.repeat(64),'policy.onnx':'c'.repeat(64)},createdAt:new Date().toISOString()};
  const post=(payload,origin='http://localhost:5173')=>fetch(base+'/responses',{method:'POST',headers:{'Content-Type':'application/json',Origin:origin},body:JSON.stringify(payload)});
  try {
    assert.equal((await fetch(base+'/health')).status,200);
    assert.equal((await post({...p,consent:false})).status,400);
    assert.equal((await post({...p,previewCoverage:{direct:.5,detour:1}})).status,400);
    assert.equal((await post(p,'http://unconfigured.example')).status,403);
    assert.equal((await post({...p,studyStatus:'released'})).status,409);
    assert.equal((await post(p)).status,201);
    assert.equal((await post(p)).status,200);
    assert.equal((await post({...p,choice:'detour'})).status,409);
    assert.equal((await post({...p,submissionId:'response-2'})).status,409);
    assert.equal((await fetch(base+'/export')).status,401);
    await service.close();base=await start();
    const rows=await (await fetch(base+'/export',{headers:{Authorization:'Bearer test-token'}})).json();
    assert.equal(rows.length,1);assert.equal(rows[0].choice,'direct');assert(rows[0].receivedAt);
    const csv=await (await fetch(base+'/export?format=csv',{headers:{Authorization:'Bearer test-token'}})).text();
    assert(csv.startsWith('submissionId,'));assert(csv.includes('"response-1"'));
  } finally {await service.close();rmSync(dir,{recursive:true,force:true});}
});
