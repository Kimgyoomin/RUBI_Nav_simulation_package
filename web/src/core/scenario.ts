import { clamp } from './terrain-controller.ts';
export type Point = [number,number];
export type RouteKey = 'direct' | 'detour';
export type Scenario = { id: string; height: number; detour: number; width: number; depth: number; speed: number; routes: Record<RouteKey,Point[]> };
export function length(points: Point[]): number { return points.slice(1).reduce((s,p,i)=>s+Math.hypot(p[0]-points[i][0],p[1]-points[i][1]),0); }
export function makeScenario(height=0.05, detour=0.8, speed=0.3): Scenario {
  if(![height,detour,speed].every(Number.isFinite) || height<0 || height>0.12 || detour<0.4 || detour>2.4 || speed<0.1 || speed>0.5) throw new Error('장면 설정 범위를 확인하세요.');
  // 2*sqrt(2^2+y^2)+2 = 6+detour. Clearance is verified below.
  const y=Math.sqrt((2+detour/2)**2-4);
  const width=0.7, depth=0.8;
  if(y-width/2<0.4) throw new Error('우회 경로의 로봇 여유 공간이 부족합니다.');
  return { id:`platform-h${Math.round(height*1000)}-d${Math.round(detour*1000)}-v${Math.round(speed*1000)}`,height,detour,width,depth,speed,
    routes:{direct:[[0,0],[6,0]],detour:[[0,0],[2,y],[4,y],[6,0]]} };
}
export function turnAngles(points: Point[]) { return points.slice(1,-1).map((p,i)=>{
  const a=Math.atan2(p[1]-points[i][1],p[0]-points[i][0]); const b=Math.atan2(points[i+2][1]-p[1],points[i+2][0]-p[0]);
  return Math.abs(Math.atan2(Math.sin(b-a),Math.cos(b-a)));
}); }
export class Follower {
  progress=0;
  route: Point[]; speed: number;
  constructor(route: Point[],speed: number) { this.route=route; this.speed=speed; }
  command(pos: Point,yaw: number): { velocity:number[]; done:boolean; error:number } {
    let distance=Infinity, nearest=this.progress, cumulative=0;
    for(let i=0;i<this.route.length-1;i++) {
      const a=this.route[i],b=this.route[i+1],dx=b[0]-a[0],dy=b[1]-a[1],l=Math.hypot(dx,dy);
      const t=clamp(((pos[0]-a[0])*dx+(pos[1]-a[1])*dy)/(l*l),0,1);
      const d=Math.hypot(pos[0]-a[0]-t*dx,pos[1]-a[1]-t*dy);
      if(d<distance) { distance=d; nearest=cumulative+t*l; } cumulative+=l;
    }
    this.progress=Math.max(this.progress,nearest);
    const goal=this.route.at(-1)!; const remaining=Math.hypot(goal[0]-pos[0],goal[1]-pos[1]);
    if(remaining<0.14) return {velocity:[0,0,0],done:true,error:distance};
    let targetDistance=Math.min(cumulative,this.progress+0.45),target=goal;
    for(let i=0;i<this.route.length-1;i++) {
      const a=this.route[i],b=this.route[i+1],l=Math.hypot(b[0]-a[0],b[1]-a[1]);
      if(targetDistance<=l) { target=[a[0]+(b[0]-a[0])*targetDistance/l,a[1]+(b[1]-a[1])*targetDistance/l]; break; }
      targetDistance-=l;
    }
    const angle=Math.atan2(target[1]-pos[1],target[0]-pos[0])-yaw;
    const err=Math.atan2(Math.sin(angle),Math.cos(angle));
    return {velocity:[this.speed*Math.max(0.1,Math.cos(err))*Math.min(1,remaining/0.5),0,clamp(2*err,-0.8,0.8)],done:false,error:distance};
  }
}
