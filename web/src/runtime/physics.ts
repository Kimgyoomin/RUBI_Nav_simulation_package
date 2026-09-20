import loadMujoco from '@mujoco/mujoco';
import { PROFILE, TerrainController, navCommand, finite } from '../core/terrain-controller.ts';
import type { State, Networks } from '../core/terrain-controller.ts';
import { Follower, length } from '../core/scenario.ts';
import type { Scenario,RouteKey } from '../core/scenario.ts';
import { sceneXml } from '../core/model.ts';
import type { Bundle } from '../core/model.ts';

let modulePromise:Promise<any>|undefined;
export const loadEngine=()=>modulePromise??=(loadMujoco({locateFile:(p:string)=>p.endsWith('.wasm')?new URL(`${import.meta.env.BASE_URL}vendor/mujoco/mujoco.wasm`,window.location.href).href:p}));
export type Frame={time:number;qpos:number[]};
export type Rollout={id:string;route:RouteKey;scenarioId:string;frames:Frame[];duration:number;completed:boolean;reason:string;actualLength:number;plannedLength:number;maxError:number;inferences:number;simulator:string;profile:string};

export class Physics {
  mj:any; model:any; data:any; vfs:any; scenario:Scenario;
  jointQ:number[]=[]; jointV:number[]=[]; motors:number[]=[]; gyro=0; quat=0; root=0; support=-1;
  removedWorlds:string[]=[];
  private constructor(mj:any,model:any,data:any,vfs:any,scenario:Scenario) {this.mj=mj;this.model=model;this.data=data;this.vfs=vfs;this.scenario=scenario;}
  static async create(bundle:Bundle,scenario:Scenario):Promise<Physics> {
    const mj=await loadEngine(),vfs=new mj.MjVFS(); let model:any,data:any;
    try {
      const normalized=sceneXml(bundle.xml,scenario);
      const doc=new DOMParser().parseFromString(normalized.xml,'application/xml');
      for(const e of Array.from(doc.querySelectorAll('equality weld'))) {
        const names=[e.getAttribute('site1'),e.getAttribute('site2')];
        if(names.includes('world_body_Connect')&&names.includes('body_world_Connect')) e.setAttribute('name','hpp_start_support');
      }
      const xml=new XMLSerializer().serializeToString(doc);
      for(const [p,bytes] of bundle.files) if(!p.endsWith('.onnx')) vfs.addBuffer(p,bytes);
      vfs.addBuffer('rubi.xml',new TextEncoder().encode(xml));
      model=mj.MjModel.from_xml_path('rubi.xml',vfs); data=new mj.MjData(model);
      const engine=new Physics(mj,model,data,vfs,scenario); engine.removedWorlds=normalized.removed;
      engine.validate(); engine.reset(); return engine;
    } catch(error) {data?.delete();model?.delete();vfs.delete();throw new Error(`MuJoCo 모델 연결 실패: ${String(error)}`);}
  }
  private id(type:string,name:string) {const id=this.mj.mj_name2id(this.model,this.mj.mjtObj[type].value,name);if(id<0) throw new Error(`모델에 ${name}이 없습니다.`);return id;}
  private validate() {
    if(this.model.nu!==6) throw new Error('6개 torque actuator 모델이 필요합니다.');
    if(Math.abs(this.model.opt.timestep-PROFILE.physicsDt)>1e-10)throw new Error('물리 timestep은 0.002초여야 합니다.');
    for(let i=0;i<6;i++) {
      const j=this.id('mjOBJ_JOINT',PROFILE.joints[i]),a=this.id('mjOBJ_ACTUATOR',PROFILE.motors[i]);
      if(this.model.jnt_type[j]!==3) throw new Error('정책 관절은 hinge여야 합니다.');
      if(this.model.actuator_trnid[2*a]!==j || Math.abs(this.model.actuator_gear[6*a]-1)>1e-8) throw new Error('관절과 torque motor의 연결/gear가 다릅니다.');
      if(this.model.actuator_dyntype[a]!==0 || this.model.actuator_gaintype[a]!==0 || this.model.actuator_biastype[a]!==0 || Math.abs(this.model.actuator_gainprm[10*a]-1)>1e-8)throw new Error('단위 gain의 직접 torque motor가 필요합니다.');
      if(this.model.actuator_ctrlrange[2*a]>-90 || this.model.actuator_ctrlrange[2*a+1]<90) throw new Error('motor ctrlrange가 ±90 Nm보다 작습니다.');
      this.jointQ.push(this.model.jnt_qposadr[j]);this.jointV.push(this.model.jnt_dofadr[j]);this.motors.push(a);
    }
    const rootId=this.id('mjOBJ_JOINT','root'); if(this.model.jnt_type[rootId]!==0) throw new Error('root freejoint가 필요합니다.');
    this.root=this.model.jnt_qposadr[rootId];
    const gyroId=this.id('mjOBJ_SENSOR','angular_velocity'),quatId=this.id('mjOBJ_SENSOR','imu_quat');
    if(this.model.sensor_dim[gyroId]!==3||this.model.sensor_dim[quatId]!==4) throw new Error('IMU sensor 차원이 다릅니다.');
    this.gyro=this.model.sensor_adr[gyroId];this.quat=this.model.sensor_adr[quatId];
    this.support=this.mj.mj_name2id(this.model,this.mj.mjtObj.mjOBJ_EQUALITY.value,'hpp_start_support');
  }
  reset() { this.mj.mj_resetData(this.model,this.data);this.data.ctrl.fill(0);this.data.qvel.fill(0);this.mj.mj_forward(this.model,this.data); }
  state():State {
    const wxyz=Array.from(this.data.sensordata.slice(this.quat,this.quat+4)) as number[];
    return {q:this.jointQ.map(i=>this.data.qpos[i]),qd:this.jointV.map(i=>this.data.qvel[i]),quat:[wxyz[1],wxyz[2],wxyz[3],wxyz[0]],omega:Array.from(this.data.sensordata.slice(this.gyro,this.gyro+3))};
  }
  position():number[] { return Array.from(this.data.qpos.slice(this.root,this.root+3)); }
  private step(torque:number[]) { for(let i=0;i<6;i++) this.data.ctrl[this.motors[i]]=torque[i];this.mj.mj_step(this.model,this.data); }
  applyFrame(frame:Frame) {if(frame.qpos.length!==this.model.nq || !finite(frame.qpos)) throw new Error('재생 데이터의 관절 구성이 다릅니다.');this.data.qpos.set(frame.qpos);this.mj.mj_forward(this.model,this.data);}
  async rollout(route:RouteKey,networks:Networks,signal:AbortSignal,onProgress:(time:number,pos:number[])=>void):Promise<Rollout> {
    this.reset(); const control=new TerrainController();control.setMode('ready');
    const yieldUi=()=>new Promise<void>(resolve=>setTimeout(resolve,0));
    for(let i=0;!control.ready;i++) {
      if(signal.aborted) throw new DOMException('실행을 취소했습니다.','AbortError');
      this.step(await control.update(this.state(),[0,0,0],networks));
      if(i%40===0) await yieldUi();
    }
    if(this.support>=0) this.data.eq_active[this.support]=0;
    control.setMode('policy');
    const follower=new Follower(this.scenario.routes[route],this.scenario.speed);
    const frames:Frame[]=[],start=this.data.time;
    let previous=this.position(),actualLength=0,maxError=0,completed=false,reason='시간 초과',settledAt=-1;
    const maxTime=Math.min(100,length(this.scenario.routes[route])/this.scenario.speed*3+8);
    let iterations=0;
    while(this.data.time-start<maxTime) {
      if(signal.aborted) throw new DOMException('실행을 취소했습니다.','AbortError');
      const pos=this.position(),q=this.data.qpos,r=this.root,w=q[r+3],x=q[r+4],y=q[r+5],z=q[r+6];
      if(!finite(pos)||!finite(q)) {reason='유효하지 않은 물리 상태';break;}
      const time=this.data.time-start;
      if(time>0.5&&(pos[2]<0.22 || 1-2*(x*x+y*y)<0.35)) {reason='넘어짐 감지';break;}
      const yaw=Math.atan2(2*(w*z+x*y),1-2*(y*y+z*z));
      const command=follower.command([pos[0],pos[1]],yaw);maxError=Math.max(maxError,command.error);
      if(command.error>0.65 && time>1) {reason='경로 이탈';break;}
      if(command.done && settledAt<0) settledAt=time;
      if(!command.done)settledAt=-1;
      if(settledAt>=0 && time-settledAt>=0.6) {completed=true;reason='도착';break;}
      if(iterations%10===0) frames.push({time,qpos:Array.from(q)});
      this.step(await control.update(this.state(),navCommand(command.velocity),networks));
      const next=this.position(); actualLength+=Math.hypot(next[0]-previous[0],next[1]-previous[1]);previous=next;
      if(iterations++%50===0) {onProgress(time,next);await yieldUi();}
    }
    const duration=this.data.time-start;frames.push({time:duration,qpos:Array.from(this.data.qpos)});
    this.data.ctrl.fill(0);
    return {id:crypto.randomUUID(),route,scenarioId:this.scenario.id,frames,duration,completed,reason,actualLength,plannedLength:length(this.scenario.routes[route]),maxError,inferences:control.inferenceCount,simulator:'MuJoCo WASM 3.13.0',profile:PROFILE.id};
  }
  dispose() {this.data.delete();this.model.delete();this.vfs.delete();}
}
