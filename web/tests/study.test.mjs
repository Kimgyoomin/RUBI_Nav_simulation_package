import test from 'node:test';
import assert from 'node:assert/strict';
import {Coverage,shuffledIndices} from '../src/core/study.ts';
import {makeScenario,length,Follower} from '../src/core/scenario.ts';
import {TerrainController} from '../src/core/terrain-controller.ts';

test('displayed detour equals the geometric extra distance across all pilot settings',()=>{
  for(const h of [0,.05,.09,.12])for(const extra of [.4,.8,1.2,1.6,2.4]){
    const s=makeScenario(h,extra);
    assert(Math.abs(length(s.routes.detour)-length(s.routes.direct)-extra)<1e-12);
    assert(s.routes.detour[1][1]-s.width/2>=.4);
  }
  assert.throws(()=>makeScenario(.2));assert.throws(()=>makeScenario(.05,NaN));
});
test('duplicate viewing and seek gaps do not count as complete viewing',()=>{
  const c=new Coverage();c.add(0,5);c.add(0,5);c.add(9,10);assert.equal(c.fraction(10),.6);
  c.add(4,9);assert.equal(c.fraction(10),1);c.add(5,6);assert.equal(c.fraction(10),1);
  c.add(20,10);assert.equal(c.fraction(10),1);
});
test('every randomized order is a complete permutation',()=>{
  for(let i=0;i<50;i++)assert.deepEqual(shuffledIndices(20).sort((a,b)=>a-b),Array.from({length:20},(_,j)=>j));
});
test('route follower distinguishes arrival and lateral departure',()=>{
  const f=new Follower([[0,0],[6,0]],.3);
  assert.equal(f.command([0,0],0).done,false);assert.deepEqual(f.command([6,0],0).velocity,[0,0,0]);
  assert.equal(f.command([3,1],0).error,1);
});
test('malformed controller input is stopped before ready or policy execution',async()=>{
  const c=new TerrainController();c.setMode('ready');
  await assert.rejects(c.update({q:Array(6).fill(0),qd:Array(6).fill(0),omega:[0,0,0],quat:[]},[0,0,0],{}));
  assert.equal(c.mode,'off');
});
