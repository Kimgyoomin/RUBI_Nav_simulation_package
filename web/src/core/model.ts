import type { Scenario } from './scenario.ts';
export type Bundle = { xml:string; files:Map<string,Uint8Array>; hashes:Record<string,string>; name:string };
const normal = (s:string) => s.replaceAll('\\','/').replace(/^\.\//,'');
export function xmlDocument(xml:string): XMLDocument {
  if(/<!DOCTYPE|<!ENTITY/i.test(xml)) throw new Error('외부 XML entity는 지원하지 않습니다.');
  const doc=new DOMParser().parseFromString(xml,'application/xml');
  if(doc.querySelector('parsererror') || doc.documentElement.tagName!=='mujoco') throw new Error('유효한 MuJoCo XML이 아닙니다.');
  return doc;
}
export function references(xml:string): {files:string[]; worlds:string[]} {
  const doc=xmlDocument(xml),compiler=doc.querySelector('compiler');
  const meshdir=compiler?.getAttribute('meshdir')||'';
  const texturedir=compiler?.getAttribute('texturedir')||'';
  const files=Array.from(doc.querySelectorAll('asset [file]')).map(e=>normal([e.tagName==='mesh'?meshdir:e.tagName==='texture'?texturedir:'',e.getAttribute('file')].filter(Boolean).join('/')));
  const worlds=Array.from(doc.documentElement.children).filter(e=>e.tagName==='include').map(e=>e.getAttribute('file')||'');
  return {files:[...new Set(files)],worlds};
}
export async function loadFiles(input:File[]): Promise<Bundle> {
  const xmlFile=input.find(f=>f.name.toLowerCase()==='rubi.xml')||input.find(f=>f.name.toLowerCase().endsWith('.xml'));
  if(!xmlFile) throw new Error('rubi.xml을 선택하세요.');
  const xml=await xmlFile.text(); const refs=references(xml);
  const files=new Map<string,Uint8Array>(),hashes:Record<string,string>={};
  const requirements=[...refs.files,'encoder.onnx','policy.onnx'];
  const missing:string[]=[];
  for(const wanted of requirements) {
    const matches=input.filter(f=>normal(f.webkitRelativePath||f.name).endsWith('/'+wanted)||normal(f.webkitRelativePath||f.name)===wanted||f.name===wanted.split('/').at(-1));
    if(matches.length>1) throw new Error(`${wanted}: 같은 이름의 파일이 여러 개입니다.`);
    if(!matches.length) { missing.push(wanted); continue; }
    const bytes=new Uint8Array(await matches[0].arrayBuffer()); files.set(wanted,bytes);
    hashes[wanted]=await sha256(bytes);
  }
  if(missing.length) throw new Error('필요한 파일: '+missing.join(', '));
  hashes['rubi.xml']=await sha256(new TextEncoder().encode(xml));
  return {xml,files,hashes,name:xmlFile.name};
}
export async function sha256(bytes:Uint8Array): Promise<string> {
  const digest=await crypto.subtle.digest('SHA-256',new Uint8Array(bytes));
  return Array.from(new Uint8Array(digest),v=>v.toString(16).padStart(2,'0')).join('');
}
export function sceneXml(source:string,s:Scenario): {xml:string; removed:string[]} {
  const doc=xmlDocument(source),root=doc.documentElement,removed:string[]=[];
  for(const e of Array.from(root.children)) if(e.tagName==='include') {
    const name=e.getAttribute('file')||'';
    if(!/world|terrain|stair|lrc_/i.test(name)) throw new Error(`추가 include가 필요합니다: ${name}`);
    removed.push(name); e.remove();
  }
  if(doc.querySelector('include')) throw new Error('로봇 내부 include는 하나의 XML로 합쳐 주세요.');
  const world=doc.querySelector('worldbody'); if(!world) throw new Error('worldbody가 없습니다.');
  const option=doc.querySelector('option');
  if(option?.getAttribute('timestep') && Math.abs(Number(option.getAttribute('timestep'))-0.002)>1e-10) throw new Error('물리 timestep은 terrain 정책과 같은 0.002초여야 합니다.');
  if(!option) { const e=doc.createElement('option'); e.setAttribute('timestep','0.002'); root.insertBefore(e,root.firstChild); }
  else option.setAttribute('timestep','0.002');
  if(!Array.from(world.children).some(e=>e.tagName==='geom'&&e.getAttribute('type')==='plane')) {
    const floor=doc.createElement('geom'); Object.entries({name:'hpp_floor',type:'plane',size:'10 10 .1',contype:'1',conaffinity:'15',friction:'0.9 0.9 0.0001'}).forEach(([k,v])=>floor.setAttribute(k,v));world.appendChild(floor);
  }
  if(s.height>0) {
    const step=doc.createElement('geom');
    Object.entries({name:'hpp_platform',type:'box',pos:`3 0 ${s.height/2}`,size:`${s.depth/2} ${s.width/2} ${s.height/2}`,rgba:'0.35 0.48 0.56 1',contype:'1',conaffinity:'15',friction:'0.9 0.9 0.0001'}).forEach(([k,v])=>step.setAttribute(k,v));
    world.appendChild(step);
  }
  return {xml:new XMLSerializer().serializeToString(doc),removed};
}
