// Port of GazeboTerrainPolicyAdapter at 8991543654f2d608b9eb722906dd7f2740611ed6.
// Simulator sampling, IMU noise and reset semantics remain backend-specific.
export const PROFILE = {
  id: 'gazebo-terrain-330-32-65-6-v1', physicsDt: 0.002, decimation: 5,
  inferenceDt: 0.01, historyFrames: 10, actorDim: 33, latentDim: 32,
  joints: ['L_HR_JOINT', 'L_HP_JOINT', 'L_KN_JOINT', 'R_HR_JOINT', 'R_HP_JOINT', 'R_KN_JOINT'],
  motors: ['L_HR_motor', 'L_HP_motor', 'L_KN_motor', 'R_HR_motor', 'R_HP_motor', 'R_KN_motor'],
  pose: [0, 0.65, -1.3, 0, 0.65, -1.3], kp: 40, kd: 2, limit: 90,
} as const;

export type State = { q: number[]; qd: number[]; quat: number[]; omega: number[] };
export type Networks = { encode(x: Float32Array): Promise<Float32Array>; act(x: Float32Array): Promise<Float32Array> };
export const clamp = (v: number, lo: number, hi: number) => Math.min(hi, Math.max(lo, v));
export const finite = (a: ArrayLike<number>) => Array.from(a).every(Number.isFinite);
export function gravity(q: number[]): number[] {
  const norm = Math.hypot(...q);
  if (q.length !== 4 || !Number.isFinite(norm) || norm < 1e-12) throw new Error('IMU quaternion이 유효하지 않습니다.');
  const [x, y, z, w] = q.map(v => v / norm);
  return [2*(y*w-x*z), -2*(y*z+x*w), -(1-2*(x*x+y*y))];
}
export function navCommand(v: number[]): number[] {
  if (v.length !== 3 || !finite(v)) throw new Error('속도 명령이 유효하지 않습니다.');
  return [clamp(v[0]/0.5,-1,1), clamp(v[1]/0.5,-1,1)*0.75, clamp(v[2],-1,1)*1.5];
}

export class TerrainController {
  phase = 0; tick = 0; modeTick = 0; ready = false; mode: 'off' | 'ready' | 'policy' = 'off';
  actor = new Float32Array(33); history = new Float32Array(330);
  input = new Float32Array(65); action = new Float32Array(6); rawAction = new Float32Array(6);
  inferenceCount = 0;
  reset() {
    this.phase=0; this.tick=0; this.modeTick=0; this.ready=false; this.mode='off'; this.inferenceCount=0;
    for (const a of [this.actor,this.history,this.input,this.action,this.rawAction]) a.fill(0);
  }
  setMode(mode: 'off' | 'ready' | 'policy') {
    if (mode==='policy' && !this.ready) throw new Error('먼저 자세 준비를 완료해야 합니다.');
    if (this.mode!==mode) { this.mode=mode; this.modeTick=0; }
  }
  observe(s: State, command: number[]): Float32Array {
    this.phase += 0.01*2; this.phase -= Math.floor(this.phase);
    this.actor.set(s.omega.map(v=>v*0.25),0); this.actor.set(gravity(s.quat),3);
    for(let i=0;i<6;i++) {
      this.actor[6+i]=s.q[i]-PROFILE.pose[i]; this.actor[12+i]=s.qd[i]*0.1; this.actor[18+i]=this.action[i];
    }
    this.actor[24]=Math.sin(this.phase*2*Math.PI); this.actor[25]=Math.cos(this.phase*2*Math.PI);
    this.actor.set([2,0.5,0.5,0],26);
    this.actor.set([command[0]*2,command[1]*2,command[2]*0.25],30);
    if(!finite(this.actor)) throw new Error('정책 관측에 유효하지 않은 값이 있습니다.');
    this.history.copyWithin(0,33); this.history.set(this.actor,297);
    return this.actor;
  }
  acceptAction(s: State, raw: Float32Array) {
    if(raw.length!==6 || !finite(raw)) throw new Error('정책 출력은 유효한 float32 6개여야 합니다.');
    this.rawAction.set(raw);
    for(let i=0;i<6;i++) {
      const lo=s.q[i]-PROFILE.pose[i]+(2*s.qd[i]-90)/40;
      const hi=s.q[i]-PROFILE.pose[i]+(2*s.qd[i]+90)/40;
      this.action[i]=clamp(raw[i],lo,hi);
    }
  }
  pd(s: State, target: number[]): number[] {
    const torque=target.map((v,i)=>clamp(40*(v-s.q[i])-2*s.qd[i],-90,90));
    if(!finite(torque)) throw new Error('PD 출력에 유효하지 않은 값이 있습니다.');
    return torque;
  }
  async update(s: State, command: number[], networks: Networks): Promise<number[]> {
    if(s.q.length!==6 || s.qd.length!==6 || s.omega.length!==3 || s.quat.length!==4 || command.length!==3 || !finite([...s.q,...s.qd,...s.quat,...s.omega,...command])) {
      this.mode='off'; throw new Error('로봇 상태가 유효하지 않아 실행을 중지했습니다.');
    }
    if(this.mode==='off') { this.tick++; return [0,0,0,0,0,0]; }
    let target: number[];
    if(this.mode==='ready') {
      const t=this.modeTick*0.002;
      if(t<=0.5) { const b=0.5*(1-Math.cos(Math.PI*t/0.5)); target=PROFILE.pose.map(q=>b*q); this.modeTick++; }
      else { target=[...PROFILE.pose]; this.ready=true; }
    } else {
      if(this.tick%5===0) {
        this.observe(s,command);
        const latent=await networks.encode(this.history);
        if(latent.length!==32 || !finite(latent)) throw new Error('Encoder 출력은 유효한 float32 32개여야 합니다.');
        this.input.set(latent,0); this.input.set(this.actor,32);
        this.acceptAction(s,await networks.act(this.input)); this.inferenceCount++;
      }
      target=PROFILE.pose.map((v,i)=>v+this.action[i]);
    }
    this.tick++; return this.pd(s,target);
  }
}
