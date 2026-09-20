/** Union of watched intervals; seeking or repeating a section cannot fill unseen time. */
export class Coverage {
  intervals:[number,number][]=[];
  add(start:number,end:number) {
    if(!Number.isFinite(start)||!Number.isFinite(end)||end<=start)return;
    const merged:[number,number][]=[];
    for(const [a,b] of [...this.intervals,[start,end] as [number,number]].sort((x,y)=>x[0]-y[0])) {
      const last=merged.at(-1);
      if(last && a<=last[1]+1e-6)last[1]=Math.max(last[1],b);else merged.push([a,b]);
    }
    this.intervals=merged;
  }
  fraction(duration:number) {return duration>0?Math.min(1,this.intervals.reduce((sum,[a,b])=>sum+b-a,0)/duration):0;}
}
export function shuffledIndices(n:number) {
  const order=Array.from({length:n},(_,i)=>i);
  for(let i=n-1;i>0;i--){const j=crypto.getRandomValues(new Uint32Array(1))[0]%(i+1);[order[i],order[j]]=[order[j],order[i]];}
  return order;
}
