import '@fontsource-variable/noto-sans-kr';
import './style.css';
import { Coverage,shuffledIndices } from './core/study.ts';
import { makeScenario,length,turnAngles } from './core/scenario.ts';
import type { Scenario,RouteKey } from './core/scenario.ts';
import { loadFiles,references } from './core/model.ts';
import type { Bundle } from './core/model.ts';
import { PROFILE } from './core/terrain-controller.ts';
import { OnnxNetworks } from './runtime/onnx.ts';
import { Physics } from './runtime/physics.ts';
import type { Rollout } from './runtime/physics.ts';
import { Viewer } from './runtime/viewer.ts';

document.querySelector('#app')!.innerHTML=`
<header class="topbar"><div class="brand"><div class="brand-symbol" aria-hidden="true">R</div><div><div class="brand-name">RUBI</div><small>HUMAN PREFERENCE PATH</small></div></div><div class="top-meta"><span class="tag">RC LAB · RESEARCH</span><nav class="modebar" aria-label="화면 모드"><button id="studio-mode" class="active">연구자</button><button id="survey-mode">설문 미리보기</button></nav></div></header>
<main class="workspace"><div class="heading"><div><div class="eyebrow">ROUTE COMPARISON / <span id="trial-counter">01</span></div><h1>같은 목적지, 두 가지 경로</h1><p>RUBI의 보행을 확인하고 이동 경로를 비교하세요.</p></div><div class="status" id="status" role="status" aria-live="polite">모델 파일을 불러오면 보행을 실행할 수 있어요.</div></div>
<div class="error" id="error" role="alert" hidden></div>
<div class="layout"><section class="main-column" aria-label="경로 미리보기"><div class="surface"><div class="viewport"><canvas id="world" aria-label="단차 통과 경로와 평지 우회 경로의 3D 환경"></canvas><div class="view-tools"><button data-camera="overview">전체 경로</button><button data-camera="step">단차 확대</button><button data-camera="top">위에서</button></div><div class="scene-badge" id="scene-badge">WORLD PREVIEW · 보행 미생성</div><div class="scale"><strong>1 m</strong>격자 한 칸 · 실제 비율</div><div class="viewport-overlay" id="progress"><span id="progress-label">보행을 계산하고 있어요.</span><progress></progress></div></div><div class="playback"><button id="play" disabled>재생</button><button id="restart" disabled>처음부터</button><input id="timeline" type="range" min="0" max="1000" value="0" aria-label="보행 재생 위치" disabled><span class="time" id="time">0.0 / 0.0 s</span></div></div>
<div class="route-grid"><article class="route-card" id="card-a"><div class="route-title">경로 A <span id="viewed-a">미리보기 전</span></div><div class="route-metric"><span id="length-a">6.0</span><small>m</small></div><p id="desc-a">5 cm 플랫폼 통과</p><button id="preview-a" disabled>경로 A 보행 보기</button></article><article class="route-card b" id="card-b"><div class="route-title">경로 B <span id="viewed-b">미리보기 전</span></div><div class="route-metric"><span id="length-b">6.8</span><small>m</small></div><p id="desc-b">평지로 우회</p><button id="preview-b" disabled>경로 B 보행 보기</button></article></div>
<section class="surface question"><h2>RUBI에게 어느 경로를 지정하시겠습니까?</h2><p id="choice-hint">두 경로의 보행을 끝까지 확인한 뒤 선택할 수 있어요.</p><label class="consent"><input type="checkbox" id="consent"> 시뮬레이션 경로 선호 조사임을 이해했고, 익명 응답 저장에 동의합니다.</label><div class="choices"><button id="choose-a" disabled>경로 A 선택</button><button id="choose-b" disabled>경로 B 선택</button><button id="choose-unsure" disabled>판단 어려움</button></div><div class="saved" id="save-status" aria-live="polite">현재 연구자 준비 단계입니다. 응답은 아직 수집되지 않습니다.</div><button id="retry-save" class="secondary" hidden>확정한 응답 다시 저장</button><button id="next-trial" class="secondary" hidden>다음 장면</button></section><section class="completion" id="completion" hidden><h2>모든 경로를 확인했어요.</h2><p id="complete-text"></p></section></section>
<aside class="sidebar"><section class="surface panel research-only"><h2><span class="number">01</span> RUBI 모델 연결</h2><div class="upload"><strong>XML · ONNX · meshes</strong><button id="select-folder">폴더 선택</button> <button id="select-files">파일 선택</button><input type="file" id="folder-input" webkitdirectory multiple hidden><input type="file" id="file-input" multiple accept=".xml,.onnx,.stl,.STL,.obj,.png" hidden></div><div class="file-status" id="file-status">이 기기에서만 불러옵니다. 파일은 서버로 전송하지 않아요.</div><details><summary>필요한 파일과 정책 설정</summary><ul><li>rubi.xml</li><li>encoder.onnx · policy.onnx</li><li>XML이 참조하는 STL 9개</li></ul><p>Gazebo terrain: 330 → 32 / 65 → 6<br>물리 500 Hz · 정책 100 Hz</p><p>world include는 설문 장면으로 교체하고 로봇의 접촉 형상은 유지합니다.</p></details></section>
<section class="surface panel research-only"><h2><span class="number">02</span> 장면 설정</h2><fieldset id="settings"><div class="field"><label for="height">단차 높이 <output id="height-out">5 cm</output></label><input id="height" type="range" min="0" max="12" step="1" value="5"><small>조사 높이는 실제 보행 검증 후 확정하세요.</small></div><div class="field"><label for="detour">추가 우회거리 <output id="detour-out">0.8 m</output></label><input id="detour" type="range" min="0.4" max="2.4" step="0.2" value="0.8"></div><div class="field"><label for="speed">전진 속도 명령 <output id="speed-out">0.30 m/s</output></label><input id="speed" type="range" min="0.1" max="0.5" step="0.05" value="0.3"></div></fieldset><div class="divider"></div><div class="meta-line"><span>플랫폼 길이 × 폭</span><span>0.8 × 0.7 m</span></div><div class="meta-line"><span>출발 · 목적점</span><span>동일</span></div><p class="hint">지형과 정책을 고정한 상태에서 두 경로를 실행합니다. 회전 특성도 기록됩니다.</p></section>
<section class="surface panel"><h2><span class="number">03</span> 보행 미리보기</h2><button class="primary" id="generate" disabled>두 경로 보행 생성</button><button class="secondary danger" id="cancel" hidden>계산 중지</button><p class="hint" id="run-hint">실제 정책의 물리 시뮬레이션을 계산한 뒤, 두 경로를 같은 시간 배율로 재생합니다.</p><button class="secondary research-only" id="export-runs" disabled>실행 기록 내려받기</button></section>
<section class="surface panel research-only"><details><summary>응답 저장 연결</summary><div class="field"><label for="api-url">응답 API 주소</label><input id="api-url" type="url" placeholder="https://your-server.example/api" autocomplete="off"></div><button class="secondary" id="connect-api">저장 연결 확인</button><p class="hint" id="api-status">미연결 · 설문 미리보기 응답은 이 기기에만 저장됩니다.</p><button class="secondary" id="export-responses">기기 응답 내려받기</button></details></section></aside></div><footer class="footer"><span>RUBI · Human Preference Path<br>시뮬레이션에서 관찰한 보행에 대한 경로 선호를 기록합니다.</span><span>MuJoCo 3.13.0 · ONNX Runtime Web 1.30.0<br>지형 · 로봇 · 제어기 조건에 따른 연구 자료</span></footer></main>`;

const el=<T extends HTMLElement=HTMLElement>(id:string)=>document.getElementById(id) as T;
const button=(id:string)=>el<HTMLButtonElement>(id);const input=(id:string)=>el<HTMLInputElement>(id);
const status=(text:string)=>{el('status').textContent=text;};
const error=(text:string)=>{el('error').textContent=text;el('error').hidden=!text;};
const stored=<T>(key:string,fallback:T):T=>{try{return JSON.parse(localStorage.getItem(key)||'null')??fallback;}catch{return fallback;}};
const persist=(key:string,value:unknown)=>{try{localStorage.setItem(key,JSON.stringify(value));return true;}catch{error('이 브라우저는 임시 저장을 허용하지 않습니다. 서버 저장 연결을 사용하세요.');return false;}};
const session=stored<string>('rubi-hpp-participant',crypto.randomUUID());persist('rubi-hpp-participant',session);
let scenario=makeScenario(),bundle:Bundle|undefined,networks:OnnxNetworks|undefined,engine:Physics|undefined;
let viewer:Viewer;
try {viewer=new Viewer(el<HTMLCanvasElement>('world'));viewer.setScenario(scenario);} catch(e){error('3D 화면을 시작할 수 없습니다. WebGL을 지원하는 브라우저에서 다시 열어 주세요. '+String(e));throw e;}
let rollouts:Partial<Record<RouteKey,Rollout>>={},busy=false,abort:AbortController|undefined,participant=false;
let aKey:RouteKey='direct',bKey:RouteKey='detour',playing=false,current:RouteKey|undefined,playTime=0,lastWall=0;
let viewed=new Set<RouteKey>(),viewTime:Record<RouteKey,number>={direct:0,detour:0},replays:Record<RouteKey,number>={direct:0,detour:0};
let coverage={direct:new Coverage(),detour:new Coverage()},bothViewedAt=0;
let previewEvents:{route:RouteKey;atMs:number}[]=[];
let answered=false,trialStart=performance.now(),api=stored<string>('rubi-hpp-api',''),apiHealthy=false;
let order:number[]=[],presentations:RouteKey[]=[];
const progressKey=()=>`rubi-hpp-progress-${study.id}`;
const saveProgress=()=>persist(progressKey(),{order,presentations,trial,answered});
let pendingPayload:Record<string,unknown>|undefined;
const localResponses=stored<Record<string,unknown>[]>('rubi-hpp-responses',[]);
type Study={id:string;status:string;responseApi:string;bundleBaseUrl:string;scenarios:{height:number;detour:number;speed:number}[]};
let study:Study={id:'rubi-hpp-pilot-v1',status:'draft',responseApi:'',bundleBaseUrl:'',scenarios:[]},trial=0;
input('api-url').value=api;

function refresh() {
  const ready=Boolean(bundle&&networks&&engine);button('generate').disabled=!ready||busy||(participant&&answered);
  button('select-folder').disabled=busy;button('select-files').disabled=busy;el<HTMLFieldSetElement>('settings').disabled=busy;
  button('studio-mode').disabled=busy;button('survey-mode').disabled=busy;
  for(const [label,key] of [['a',aKey],['b',bKey]] as const) {
    el(`length-${label}`).textContent=length(scenario.routes[key]).toFixed(1);
    el(`desc-${label}`).textContent=key==='direct'?`${Math.round(scenario.height*100)} cm 플랫폼 통과`:'평지로 우회';
    el(`viewed-${label}`).textContent=viewed.has(key)?'확인 완료':rollouts[key]?`${rollouts[key]!.duration.toFixed(1)} s · ${rollouts[key]!.reason}`:'미리보기 전';
    button(`preview-${label}`).disabled=!rollouts[key]||busy;
    el(`card-${label}`).classList.toggle('selected',current===key);
  }
  const valid=Boolean(rollouts.direct?.completed&&rollouts.detour?.completed);
  for(const id of ['choose-a','choose-b','choose-unsure']) button(id).disabled=!valid||viewed.size!==2||busy||answered||!input('consent').checked||Boolean(pendingPayload);
  button('export-runs').disabled=!rollouts.direct||!rollouts.detour||busy;
  const clip=current?rollouts[current]:undefined;
  button('play').disabled=!clip||busy;button('restart').disabled=!clip||busy;input('timeline').disabled=!clip||busy||!current||!viewed.has(current);
  button('play').textContent=playing?'일시정지':'재생';
  if(!valid && rollouts.direct && rollouts.detour) el('choice-hint').textContent='완주하지 못한 경로가 있어 이 장면의 선호 응답을 받지 않습니다. 설정과 실행 결과를 확인하세요.';
  else el('choice-hint').textContent=viewed.size===2?'같은 조건에서 RUBI에게 맡길 경로를 선택하세요.':'두 경로의 보행을 끝까지 확인한 뒤 선택할 수 있어요.';
  el('scene-badge').textContent=busy?'POLICY ROLLOUT · 계산 중':clip?'실제 정책 실행 기록 · 1× 재생':ready?'RUBI 연결됨 · 보행 생성 전':'WORLD PREVIEW · 보행 미생성';
}
function resetTrial() {coverage={direct:new Coverage(),detour:new Coverage()};bothViewedAt=0;previewEvents=[];button('retry-save').hidden=true;playing=false;current=undefined;playTime=0;rollouts={};viewed.clear();viewTime={direct:0,detour:0};replays={direct:0,detour:0};answered=false;pendingPayload=undefined;trialStart=performance.now();button('next-trial').hidden=true;el('completion').hidden=true;input('timeline').value='0';el('time').textContent='0.0 / 0.0 s';refresh();}
async function rebuild() {if(!bundle)return;const next=await Physics.create(bundle,scenario);viewer.attach(next);engine?.dispose();engine=next;refresh();}
async function importFiles(files:File[]) {
  if(busy)return;error('');busy=true;refresh();status('모델과 정책 파일을 검사하고 있어요.');let nextNetworks:OnnxNetworks|undefined,nextEngine:Physics|undefined;
  try {
    const candidate=await loadFiles(files);nextNetworks=await OnnxNetworks.create(candidate.files);nextEngine=await Physics.create(candidate,scenario);
    viewer.attach(nextEngine);engine?.dispose();await networks?.dispose();bundle=candidate;networks=nextNetworks;engine=nextEngine;nextNetworks=undefined;nextEngine=undefined;
    resetTrial();el('file-status').textContent=`파일 검사 통과 · ${candidate.files.size+1}개 연결. 보행 성능은 실행 후 확인하세요.`;
    status('RUBI 연결 완료. 두 경로의 보행을 생성하세요.');
  } catch(e) {error(String(e));status('필요한 파일과 정책 구성을 확인해 주세요.');await nextNetworks?.dispose();nextEngine?.dispose();}
  finally {busy=false;refresh();}
}
button('select-folder').onclick=()=>input('folder-input').click();button('select-files').onclick=()=>input('file-input').click();
for(const id of ['folder-input','file-input']) input(id).onchange=()=>{void importFiles(Array.from(input(id).files||[]));};

for(const id of ['height','detour','speed']) input(id).oninput=()=>{
  const h=Number(input('height').value)/100,d=Number(input('detour').value),v=Number(input('speed').value);
  el('height-out').textContent=`${Math.round(h*100)} cm`;el('detour-out').textContent=`${d.toFixed(1)} m`;el('speed-out').textContent=`${v.toFixed(2)} m/s`;
  scenario=makeScenario(h,d,v);resetTrial();viewer.setScenario(scenario);viewer.robot.visible=false;status('설정을 바꿨습니다. 새 조건으로 보행을 생성하세요.');
};
button('generate').onclick=async()=>{
  if(!bundle||!networks||busy)return;error('');resetTrial();busy=true;abort=new AbortController();refresh();el('progress').classList.add('visible');button('cancel').hidden=false;viewer.robot.visible=false;
  try {
    await rebuild();viewer.robot.visible=false;
    for(const key of [aKey,bKey]) {
      const label=key===aKey?'A':'B';status(`경로 ${label}의 실제 정책 보행을 계산하고 있어요.`);
      const run=await engine!.rollout(key,networks,abort.signal,(time)=>{el('progress-label').textContent=`경로 ${label} · 시뮬레이션 ${time.toFixed(1)}초 계산 중`;});
      rollouts[key]=run;refresh();
    }
    status(rollouts.direct!.completed&&rollouts.detour!.completed?'두 경로가 도착했습니다. 보행을 확인하세요.':'완주하지 못한 경로가 있습니다. 실행 기록을 확인하세요.');
    engine!.applyFrame(rollouts[aKey]!.frames[0]);viewer.updateRobot();viewer.robot.visible=true;current=aKey;
  } catch(e){if((e as Error).name==='AbortError')status('보행 계산을 중지했습니다.');else{error(String(e));status('실행 중 오류가 발생했습니다.');}}
  finally{busy=false;el('progress').classList.remove('visible');button('cancel').hidden=true;refresh();}
};
button('cancel').onclick=()=>abort?.abort();
function preview(key:RouteKey) {if(!rollouts[key]||!engine)return;previewEvents.push({route:key,atMs:performance.now()-trialStart});current=key;playTime=0;playing=true;lastWall=0;replays[key]++;viewer.robot.visible=true;viewer.setScenario(scenario,key);refresh();}
button('preview-a').onclick=()=>preview(aKey);button('preview-b').onclick=()=>preview(bKey);
button('restart').onclick=()=>{if(current)preview(current);};button('play').onclick=()=>{if(current){if(playTime>=rollouts[current]!.duration){preview(current);return;}playing=!playing;lastWall=0;refresh();}};
input('timeline').oninput=()=>{if(current){playing=false;playTime=Number(input('timeline').value)/1000*rollouts[current]!.duration;renderFrame();refresh();}};
function renderFrame() {
  if(!current||!engine||busy)return;const clip=rollouts[current];if(!clip)return;
  let lo=0,hi=clip.frames.length-1;while(lo<hi){const mid=Math.ceil((lo+hi)/2);if(clip.frames[mid].time<=playTime)lo=mid;else hi=mid-1;}
  engine.applyFrame(clip.frames[lo]);viewer.updateRobot();input('timeline').value=String(Math.round(playTime/Math.max(clip.duration,0.001)*1000));el('time').textContent=`${playTime.toFixed(1)} / ${clip.duration.toFixed(1)} s`;
}
viewer.onFrame=(now)=>{
  if(!playing||!current||busy||document.hidden){lastWall=0;return;}const dt=lastWall?(now-lastWall)/1000:0;lastWall=now;
  const clip=rollouts[current]!,before=playTime;playTime=Math.min(clip.duration,playTime+dt);viewTime[current]+=playTime-before;if(dt<.25)coverage[current].add(before,playTime);renderFrame();
  if(playTime>=clip.duration){playing=false;if(coverage[current].fraction(clip.duration)>=.98)viewed.add(current);else status('재생 중 화면이 지연되었습니다. 놓친 구간을 다시 확인해 주세요.');if(viewed.size===2&&!bothViewedAt)bothViewedAt=performance.now();refresh();}
};
document.querySelectorAll<HTMLButtonElement>('[data-camera]').forEach(b=>b.onclick=()=>viewer.setCamera(b.dataset.camera as 'overview'|'step'|'top'));
function applyTrial() {
  const s=study.scenarios[order[trial]];
  if(s)scenario=makeScenario(s.height,s.detour,s.speed);
  input('height').value=String(scenario.height*100);input('detour').value=String(scenario.detour);input('speed').value=String(scenario.speed);
  el('height-out').textContent=`${Math.round(scenario.height*100)} cm`;el('detour-out').textContent=`${scenario.detour.toFixed(1)} m`;el('speed-out').textContent=`${scenario.speed.toFixed(2)} m/s`;
  aKey=presentations[trial]??'direct';bKey=aKey==='direct'?'detour':'direct';viewer.aKey=aKey;
  el('trial-counter').textContent=String(trial+1).padStart(2,'0');
}
function setMode(value:boolean) {
  if(study.status==='released'&&!value)return;
  if(participant===value)return;
  participant=value;button('studio-mode').classList.toggle('active',!value);button('survey-mode').classList.toggle('active',value);
  document.querySelectorAll<HTMLElement>('.research-only').forEach(e=>e.hidden=value);
  resetTrial();
  if(value){applyTrial();const saved=stored<{answered:boolean}>(progressKey(),{answered:false});answered=saved.answered;if(answered)button('next-trial').hidden=false;}
  else{aKey='direct';bKey='detour';viewer.aKey=aKey;}
  el('save-status').textContent=study.status==='released'&&apiHealthy?'이 조건의 응답은 연구용 데이터로 저장됩니다.':'설문 미리보기입니다. 중앙 수집이 연결되지 않으면 응답은 이 기기에만 저장돼요.';
  viewer.setScenario(scenario);viewer.robot.visible=false;refresh();
}
button('studio-mode').onclick=()=>setMode(false);button('survey-mode').onclick=()=>setMode(true);
input('consent').onchange=()=>refresh();
function apiBase(value:string):string {if(!value.trim())throw new Error('응답 API 주소를 입력하세요.');const url=new URL(value,location.href);if(!['http:','https:'].includes(url.protocol))throw new Error('HTTP 또는 HTTPS API 주소를 입력하세요.');return url.href.replace(/\/$/,'');}
async function checkApi() {apiHealthy=false;error('');try{api=apiBase(input('api-url').value);const response=await fetch(api+'/health',{signal:AbortSignal.timeout(8000)});if(!response.ok)throw new Error('응답 서비스에 연결할 수 없습니다.');const data=await response.json();if(data.service!=='rubi-hpp')throw new Error('RUBI 응답 서비스가 아닙니다.');if(data.experimentId!==study.id||data.studyStatus!==study.status)throw new Error('웹사이트와 응답 서버의 연구 ID 또는 draft/released 설정이 다릅니다.');apiHealthy=true;persist('rubi-hpp-api',api);el('api-status').textContent='응답 서비스 연결 확인. 연구자 승인 전에는 미리보기 자료로 기록됩니다.';}catch(e){error(String(e));el('api-status').textContent='미연결 · 중앙 저장 완료로 표시하지 않습니다.';}}
button('connect-api').onclick=()=>void checkApi();
async function choose(choice:RouteKey|'unsure') {
  if(answered||busy||!input('consent').checked||viewed.size!==2||!rollouts.direct?.completed||!rollouts.detour?.completed)return;
  const payload=pendingPayload??{schemaVersion:1,consent:true,submissionId:crypto.randomUUID(),participantId:session,experimentId:study.id,studyStatus:study.status,trialId:participant?`${trial}-${scenario.id}`:`studio-${crypto.randomUUID()}`,scenario:{...scenario,lengthMetric:'planned_xy_polyline',turnAngles:{direct:turnAngles(scenario.routes.direct),detour:turnAngles(scenario.routes.detour)},initialTurns:{direct:0,detour:Math.atan2(scenario.routes.detour[1][1],scenario.routes.detour[1][0])},crossingType:'platform_ascent_and_descent'},choice,presentation:{a:aKey,b:bKey},runs:{direct:{...rollouts.direct,frames:undefined},detour:{...rollouts.detour,frames:undefined}},hashes:bundle!.hashes,viewTime:{...viewTime},replays:{...replays},previewEvents:[...previewEvents],previewCoverage:{direct:coverage.direct.fraction(rollouts.direct.duration),detour:coverage.detour.fraction(rollouts.detour.duration)},decisionMs:performance.now()-bothViewedAt,elapsedTrialMs:performance.now()-trialStart,trialOrder:order,browser:{userAgent:navigator.userAgent,viewport:[innerWidth,innerHeight]},createdAt:new Date().toISOString()};
  pendingPayload=payload;busy=true;refresh();
  try {
    if(study.status==='released'&&!apiHealthy)throw new Error('연구 응답을 저장할 서버 연결이 필요합니다. 연결 확인 후 다시 제출하세요.');
    if(apiHealthy) {const result=await fetch(api+'/responses',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload),signal:AbortSignal.timeout(12000)});if(!result.ok)throw new Error(`서버 저장 실패 (${result.status}). 같은 응답을 다시 제출할 수 있어요.`);const saved=await result.json();if(saved.submissionId!==payload.submissionId)throw new Error('서버의 저장 확인이 일치하지 않습니다.');el('save-status').textContent='응답이 서버에 저장되었습니다.';}
    else el('save-status').textContent='미리보기 응답을 이 기기에 저장했습니다. 연구자에게 전송되지는 않았어요.';
    if(!localResponses.some(p=>p.submissionId===payload.submissionId))localResponses.push(payload);const storedOk=persist('rubi-hpp-responses',localResponses);if(!apiHealthy&&!storedOk)throw new Error('기기 저장에 실패했습니다. 저장 연결을 확인한 후 다시 시도하세요.');answered=true;pendingPayload=undefined;button('retry-save').hidden=true;button('next-trial').hidden=!participant;saveProgress();
  } catch(e){error(String(e));button('retry-save').hidden=false;button('retry-save').textContent=`확정한 ${payload.choice==='unsure'?'판단 어려움':payload.choice===aKey?'경로 A':'경로 B'} 응답 다시 저장`;} finally{busy=false;refresh();}
}
button('retry-save').onclick=()=>void choose(pendingPayload!.choice as RouteKey|'unsure');
button('choose-a').onclick=()=>void choose(aKey);button('choose-b').onclick=()=>void choose(bKey);button('choose-unsure').onclick=()=>void choose('unsure');
button('next-trial').onclick=()=>{
  if(trial+1>=order.length){button('next-trial').hidden=true;el('completion').hidden=false;el('complete-text').textContent=apiHealthy?'응답 저장을 확인했습니다. 참여해 주셔서 감사합니다.':'미리보기를 완료했습니다. 응답은 이 기기에만 저장되어 있습니다.';return;}
  trial++;applyTrial();resetTrial();saveProgress();viewer.setScenario(scenario);viewer.robot.visible=false;status('다음 장면의 보행을 생성해 주세요.');
};
function download(data:unknown,name:string) {const url=URL.createObjectURL(new Blob([JSON.stringify(data,null,2)],{type:'application/json'}));const a=document.createElement('a');a.href=url;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);}
button('export-runs').onclick=()=>download({schemaVersion:1,scenario,profile:PROFILE,hashes:bundle?.hashes,rollouts},`rubi-runs-${scenario.id}.json`);
button('export-responses').onclick=()=>download(localResponses,'rubi-preference-responses.json');

async function bootstrap() {
  try {const response=await fetch(`${import.meta.env.BASE_URL}study.json`);if(response.ok)study=await response.json();
    if(!['draft','released'].includes(study.status)||!Array.isArray(study.scenarios)||!study.scenarios.length)throw new Error('study.json 연구 설정이 올바르지 않습니다.');
    study.scenarios.forEach(s=>makeScenario(s.height,s.detour,s.speed));
    const saved=stored<{order:number[];presentations:RouteKey[];trial:number;answered:boolean}>(progressKey(),{order:[],presentations:[],trial:0,answered:false});
    const valid=saved.order.length===study.scenarios.length&&new Set(saved.order).size===study.scenarios.length&&saved.order.every(n=>Number.isInteger(n)&&n>=0&&n<study.scenarios.length)&&saved.presentations.length===saved.order.length&&saved.presentations.every(k=>k==='direct'||k==='detour')&&Number.isInteger(saved.trial)&&saved.trial>=0&&saved.trial<saved.order.length;
    order=valid?saved.order:shuffledIndices(study.scenarios.length);presentations=valid?saved.presentations:order.map(()=>crypto.getRandomValues(new Uint8Array(1))[0]%2?'direct':'detour');trial=valid?saved.trial:0;if(!valid)saveProgress();
    if(import.meta.env.VITE_RESPONSE_API)study.responseApi=import.meta.env.VITE_RESPONSE_API;
    if(study.responseApi){input('api-url').value=study.responseApi;await checkApi();}
    if(study.bundleBaseUrl){
      const base=new URL(study.bundleBaseUrl,new URL(import.meta.env.BASE_URL,location.href));
      const response=await fetch(new URL('rubi.xml',base));if(!response.ok)throw new Error('배포된 rubi.xml을 불러올 수 없습니다.');const xml=await response.text();
      const files=[new File([xml],'rubi.xml',{type:'application/xml'})];
      for(const name of [...references(xml).files,'encoder.onnx','policy.onnx']){const r=await fetch(new URL(name,base));if(!r.ok)throw new Error(`배포 자산이 없습니다: ${name}`);files.push(new File([await r.arrayBuffer()],name.split('/').at(-1)!));}
      await importFiles(files);
    }
    if(study.status==='released'){if(!study.bundleBaseUrl||!study.responseApi)throw new Error('공개 설문에는 모델 자산과 응답 API를 모두 설정해야 합니다.');button('studio-mode').hidden=true;button('survey-mode').textContent='설문 참여';setMode(true);}
  }catch(e){error(String(e));}
}
refresh();void bootstrap();
