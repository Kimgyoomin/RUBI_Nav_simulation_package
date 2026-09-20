import * as T from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import type { Scenario, RouteKey } from '../core/scenario.ts';
import type { Physics } from './physics.ts';

export class Viewer {
  renderer:T.WebGLRenderer; scene=new T.Scene(); camera=new T.PerspectiveCamera(42,1,0.01,150);
  controls:OrbitControls; world=new T.Group();robot=new T.Group(); robotMeshes:{id:number;mesh:T.Mesh}[]=[];
  engine:Physics|undefined; animation=0; observer:ResizeObserver; canvas:HTMLCanvasElement; selected:RouteKey|undefined;
  aKey:RouteKey='direct';
  onFrame:(time:number)=>void=()=>{};
  constructor(canvas:HTMLCanvasElement) {
    this.canvas=canvas;this.renderer=new T.WebGLRenderer({canvas,antialias:true,alpha:false});
    this.renderer.setPixelRatio(Math.min(devicePixelRatio,2));this.renderer.setClearColor('#e7edf0');
    this.renderer.shadowMap.enabled=true;this.renderer.shadowMap.type=T.PCFShadowMap;
    this.camera.up.set(0,0,1);this.camera.position.set(7,-7,5.5);
    this.controls=new OrbitControls(this.camera,canvas);this.controls.target.set(3,0,0.15);
    this.controls.enableDamping=true;this.controls.minDistance=0.6;this.controls.maxDistance=24;
    this.controls.maxPolarAngle=Math.PI/2-0.02;
    this.scene.add(new T.HemisphereLight(0xffffff,0x627784,2));
    const sun=new T.DirectionalLight(0xffffff,2.6);sun.position.set(1,-3,8);sun.castShadow=true;sun.shadow.mapSize.set(2048,2048);
    Object.assign(sun.shadow.camera,{left:-8,right:8,top:8,bottom:-8,near:0.1,far:30});sun.shadow.bias=-0.0002;this.scene.add(sun);
    this.scene.add(this.world,this.robot);
    this.observer=new ResizeObserver(()=>this.resize());this.observer.observe(canvas.parentElement!);this.resize();
    const frame=(time:number)=>{this.animation=requestAnimationFrame(frame);this.onFrame(time);this.controls.update();this.renderer.render(this.scene,this.camera);};
    this.animation=requestAnimationFrame(frame);
  }
  resize() {const r=this.canvas.parentElement!.getBoundingClientRect();this.renderer.setSize(r.width,r.height,false);this.camera.aspect=r.width/r.height;this.camera.updateProjectionMatrix();}
  clear(group:T.Group) {group.traverse(o=>{if(o instanceof T.Mesh||o instanceof T.Line){o.geometry.dispose();const m=Array.isArray(o.material)?o.material:[o.material];m.forEach(v=>v.dispose());}if(o instanceof T.Sprite){o.material.map?.dispose();o.material.dispose();}});group.clear();}
  label(text:string,x:number,y:number,z:number,color='#254554') {
    const c=document.createElement('canvas');c.width=512;c.height=128;const ctx=c.getContext('2d')!;
    ctx.fillStyle='rgba(255,255,255,0.94)';ctx.beginPath();ctx.roundRect(6,6,500,116,28);ctx.fill();
    ctx.fillStyle=color;ctx.font='bold 44px Arial, sans-serif';ctx.textAlign='center';ctx.textBaseline='middle';ctx.fillText(text,256,66);
    const texture=new T.CanvasTexture(c),sprite=new T.Sprite(new T.SpriteMaterial({map:texture,depthTest:false}));
    sprite.position.set(x,y,z);sprite.scale.set(1.25,0.31,1);this.world.add(sprite);
  }
  setScenario(s:Scenario,selected?:RouteKey) {
    this.selected=selected;this.clear(this.world);
    const floor=new T.Mesh(new T.PlaneGeometry(28,22),new T.MeshStandardMaterial({color:'#e7edf0',roughness:1}));floor.receiveShadow=true;floor.position.set(3,0,-0.004);this.world.add(floor);
    const grid=new T.GridHelper(24,24,0xa4b7c0,0xcad6dc);grid.rotation.x=Math.PI/2;grid.position.set(3,0,0);this.world.add(grid);
    if(s.height>0) {
      const box=new T.Mesh(new T.BoxGeometry(s.depth,s.width,s.height),new T.MeshStandardMaterial({color:'#738d9b',roughness:0.82}));box.position.set(3,0,s.height/2);box.castShadow=true;box.receiveShadow=true;this.world.add(box);
      const edges=new T.LineSegments(new T.EdgesGeometry(box.geometry),new T.LineBasicMaterial({color:'#334e5e'}));edges.position.copy(box.position);this.world.add(edges);
    }
    for(const key of ['direct','detour'] as const) {
      const points=s.routes[key].map(p=>new T.Vector3(p[0],p[1],0.02));
      const line=new T.Line(new T.BufferGeometry().setFromPoints(points),new T.LineBasicMaterial({color:key===this.aKey?0x087d79:0xcc6640,transparent:true,opacity:!selected||selected===key?1:0.25}));this.world.add(line);
      // Repeated markers keep routes readable when WebGL ignores lineWidth.
      for(let i=0;i<points.length-1;i++) {const a=points[i],b=points[i+1],n=Math.ceil(a.distanceTo(b)/0.12);for(let j=0;j<n;j++){
        const marker=new T.Mesh(new T.SphereGeometry(0.026,8,6),new T.MeshBasicMaterial({color:key===this.aKey?0x087d79:0xcc6640,transparent:true,opacity:!selected||selected===key?0.85:0.2}));marker.position.lerpVectors(a,b,j/n);this.world.add(marker);
      }}
    }
    for(const [x,color] of [[0,0x087d79],[6,0x254554]] as const) {const marker=new T.Mesh(new T.CylinderGeometry(0.13,0.13,0.009,32),new T.MeshStandardMaterial({color}));marker.rotation.x=Math.PI/2;marker.position.set(x,0,0.009);this.world.add(marker);}
    this.label('START',0,-0.5,0.12);this.label('GOAL',6,-0.5,0.12);this.label(`${Math.round(s.height*100)} cm`,3,-0.55,0.28);
  }
  setCamera(preset:'overview'|'step'|'top') {if(preset==='overview'){this.camera.position.set(7,-7,5.5);this.controls.target.set(3,0,0.15);}if(preset==='step'){this.camera.position.set(4.3,-2.4,1.6);this.controls.target.set(3,0,0.2);}if(preset==='top'){this.camera.position.set(3,-0.01,9);this.controls.target.set(3,0,0);}this.controls.update();}
  attach(engine:Physics) {
    this.clear(this.robot);this.robotMeshes=[];this.engine=engine;
    const m=engine.model;
    for(let id=0;id<m.ngeom;id++) {
      if(m.geom_bodyid[id]===0 || m.geom_rgba[id*4+3]<0.01) continue;
      const size=Array.from(m.geom_size.slice(id*3,id*3+3)) as number[],type=m.geom_type[id];let geometry:T.BufferGeometry;
      if(type===7) {
        const meshId=m.geom_dataid[id],va=m.mesh_vertadr[meshId],vn=m.mesh_vertnum[meshId],fa=m.mesh_faceadr[meshId],fn=m.mesh_facenum[meshId];
        geometry=new T.BufferGeometry();geometry.setAttribute('position',new T.BufferAttribute(new Float32Array(m.mesh_vert.slice(3*va,3*(va+vn))),3));
        geometry.setIndex(new T.BufferAttribute(new Uint32Array(m.mesh_face.slice(3*fa,3*(fa+fn))),1));geometry.computeVertexNormals();
      } else if(type===6) geometry=new T.BoxGeometry(size[0]*2,size[1]*2,size[2]*2);
      else if(type===5) {geometry=new T.CylinderGeometry(size[0],size[0],size[1]*2,24);geometry.rotateX(Math.PI/2);}
      else if(type===3) {geometry=new T.CapsuleGeometry(size[0],size[1]*2,6,16);geometry.rotateX(Math.PI/2);}
      else {geometry=new T.SphereGeometry(type===4?1:size[0],24,16);if(type===4)geometry.scale(size[0],size[1],size[2]);}
      const rgba=Array.from(m.geom_rgba.slice(id*4,id*4+4)) as number[];
      const mesh=new T.Mesh(geometry,new T.MeshStandardMaterial({color:new T.Color(rgba[0],rgba[1],rgba[2]),roughness:0.65,metalness:0.12,side:T.DoubleSide}));
      mesh.castShadow=true;mesh.receiveShadow=true;mesh.matrixAutoUpdate=false;this.robot.add(mesh);this.robotMeshes.push({id,mesh});
    }
    this.updateRobot();
  }
  updateRobot() {if(!this.engine)return;const d=this.engine.data;for(const {id,mesh} of this.robotMeshes) {const i=id*9,j=id*3,r=d.geom_xmat,p=d.geom_xpos;mesh.matrix.set(r[i],r[i+1],r[i+2],p[j],r[i+3],r[i+4],r[i+5],p[j+1],r[i+6],r[i+7],r[i+8],p[j+2],0,0,0,1);mesh.matrixWorldNeedsUpdate=true;}}
  dispose() {cancelAnimationFrame(this.animation);this.observer.disconnect();this.controls.dispose();this.clear(this.robot);this.clear(this.world);this.renderer.dispose();}
}
