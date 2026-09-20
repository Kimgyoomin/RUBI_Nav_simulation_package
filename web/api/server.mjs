import http from 'node:http';
import { DatabaseSync } from 'node:sqlite';
import { mkdirSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { createHash, timingSafeEqual } from 'node:crypto';
import { pathToFileURL } from 'node:url';

const id = value => typeof value === 'string' && /^[a-zA-Z0-9_.:-]{1,180}$/.test(value);
const positive = value => Number.isFinite(value) && value >= 0;
export function validateResponse(p) {
  if (!p || typeof p !== 'object' || !['submissionId','participantId','experimentId','trialId'].every(k => id(p[k]))) return 'Invalid identifiers';
  if (p.schemaVersion !== 1 || !['draft','released'].includes(p.studyStatus) || !['direct','detour','unsure'].includes(p.choice)) return 'Invalid schema or choice';
  if (!p.presentation || new Set([p.presentation.a,p.presentation.b]).size !== 2 || ![p.presentation.a,p.presentation.b].every(x => ['direct','detour'].includes(x))) return 'Invalid presentation';
  const s=p.scenario;
  if (!s || !positive(s.height) || s.height>.12 || !Number.isFinite(s.detour) || s.detour<.4 || s.detour>2.4 || !Number.isFinite(s.speed) || s.speed<.1 || s.speed>.5) return 'Invalid scenario';
  for (const key of ['direct','detour']) {
    const run=p.runs?.[key];
    if (!run || run.completed!==true || run.route!==key || run.scenarioId!==s.id || !positive(run.duration) || run.duration===0 || !positive(run.actualLength) || !positive(run.plannedLength) || run.frames) return 'Both completed rollout metadata are required';
    if (!positive(p.previewCoverage?.[key]) || p.previewCoverage[key]<.98 || p.previewCoverage[key]>1 || !positive(p.viewTime?.[key])) return 'Both previews must be watched';
  }
  if (!positive(p.decisionMs) || !positive(p.elapsedTrialMs) || p.consent!==true) return 'Missing timing or participation agreement';
  if (!p.hashes || !['rubi.xml','encoder.onnx','policy.onnx'].every(k => /^[a-f0-9]{64}$/.test(p.hashes[k]??''))) return 'Missing model fingerprints';
  if (Number.isNaN(Date.parse(p.createdAt))) return 'Invalid timestamp';
  return null;
}

export function createService({dbPath='data/responses.sqlite',allowedOrigins=['http://localhost:5173','http://127.0.0.1:5173'],adminToken='',experimentId='rubi-hpp-pilot-v1',studyStatus='draft'}={}) {
  mkdirSync(dirname(resolve(dbPath)),{recursive:true});
  const db=new DatabaseSync(dbPath);
  db.exec('PRAGMA journal_mode=WAL; PRAGMA busy_timeout=5000; CREATE TABLE IF NOT EXISTS responses (submission_id TEXT PRIMARY KEY, participant_id TEXT NOT NULL, experiment_id TEXT NOT NULL, trial_id TEXT NOT NULL, received_at TEXT NOT NULL, digest TEXT NOT NULL, payload TEXT NOT NULL, UNIQUE(participant_id,experiment_id,trial_id));');
  const insert=db.prepare('INSERT INTO responses VALUES(?,?,?,?,?,?,?)');
  const rates=new Map();
  const send=(res,status,data)=>{res.writeHead(status,{'Content-Type':'application/json; charset=utf-8','Cache-Control':'no-store'});res.end(JSON.stringify(data));};
  const server=http.createServer(async(req,res)=>{
    const origin=req.headers.origin;
    if(origin && !allowedOrigins.includes(origin)) return send(res,403,{error:'Origin is not allowed'});
    if(origin){res.setHeader('Access-Control-Allow-Origin',origin);res.setHeader('Vary','Origin');}
    res.setHeader('Access-Control-Allow-Methods','GET, POST, OPTIONS');res.setHeader('Access-Control-Allow-Headers','Content-Type, Authorization');
    if(req.method==='OPTIONS'){res.writeHead(204);res.end();return;}
    const url=new URL(req.url,'http://localhost');
    if(req.method==='GET' && url.pathname==='/api/health') return send(res,200,{service:'rubi-hpp',schemaVersion:1,experimentId,studyStatus});
    if(req.method==='GET' && url.pathname==='/api/export') {
      const supplied=Buffer.from(req.headers.authorization?.replace(/^Bearer /,'')??''),expected=Buffer.from(adminToken);
      if(!expected.length || supplied.length!==expected.length || !timingSafeEqual(supplied,expected)) return send(res,401,{error:'Researcher token required'});
      const rows=db.prepare('SELECT received_at, payload FROM responses ORDER BY received_at').all().map(row=>({...JSON.parse(row.payload),receivedAt:row.received_at}));
      if(url.searchParams.get('format')==='csv') {
        const fields=['submissionId','participantId','experimentId','trialId','studyStatus','choice','height','detour','speed','directLength','detourLength','directDuration','detourDuration','decisionMs','receivedAt'];
        const quote=v=>'"'+String(v??'').replaceAll('"','""')+'"';
        const data=rows.map(p=>({...p,height:p.scenario.height,detour:p.scenario.detour,speed:p.scenario.speed,directLength:p.runs.direct.actualLength,detourLength:p.runs.detour.actualLength,directDuration:p.runs.direct.duration,detourDuration:p.runs.detour.duration}));
        res.writeHead(200,{'Content-Type':'text/csv; charset=utf-8','Cache-Control':'no-store'});res.end([fields.join(','),...data.map(row=>fields.map(k=>quote(row[k])).join(','))].join('\n'));return;
      }
      return send(res,200,rows);
    }
    if(req.method!=='POST' || url.pathname!=='/api/responses') return send(res,404,{error:'Not found'});
    if(!req.headers['content-type']?.startsWith('application/json')) return send(res,415,{error:'application/json required'});
    const address=req.socket.remoteAddress??'unknown',minute=Math.floor(Date.now()/60000);
    const rate=rates.get(address);const count=rate?.minute===minute?rate.count+1:1;rates.set(address,{minute,count});
    if(rates.size>10000) for(const [key,value] of rates) if(value.minute<minute) rates.delete(key);
    if(count>120) return send(res,429,{error:'Please retry later'});
    try {
      const chunks=[];let size=0;
      for await (const chunk of req) {size+=chunk.length;if(size>128*1024){send(res,413,{error:'Response too large'});return;}chunks.push(chunk);}
      const raw=Buffer.concat(chunks).toString('utf8'),payload=JSON.parse(raw),problem=validateResponse(payload);
      if(problem) return send(res,400,{error:problem});
      if(payload.experimentId!==experimentId || payload.studyStatus!==studyStatus) return send(res,409,{error:'Study configuration does not match this service'});
      const digest=createHash('sha256').update(raw).digest('hex');
      const existing=db.prepare('SELECT digest FROM responses WHERE submission_id=?').get(payload.submissionId);
      if(existing) return existing.digest===digest?send(res,200,{submissionId:payload.submissionId,duplicate:true}):send(res,409,{error:'Submission ID already has a different response'});
      try {insert.run(payload.submissionId,payload.participantId,payload.experimentId,payload.trialId,new Date().toISOString(),digest,raw);}
      catch(e){if(String(e).includes('UNIQUE constraint'))return send(res,409,{error:'This participant already answered this trial'});throw e;}
      return send(res,201,{submissionId:payload.submissionId});
    } catch(e){if(e instanceof SyntaxError)return send(res,400,{error:'Invalid JSON'});console.error('Response service:',e.message);return send(res,500,{error:'Response was not confirmed; retry with the same submission ID'});}
  });
  server.requestTimeout=15000;server.headersTimeout=10000;
  return {server,close:()=>new Promise(resolve=>server.close(()=>{db.close();resolve();}))};
}

if(import.meta.url===pathToFileURL(process.argv[1]??'').href) {
  const service=createService({dbPath:process.env.RUBI_DB_PATH??'data/responses.sqlite',allowedOrigins:(process.env.RUBI_ALLOWED_ORIGIN??'http://localhost:5173,http://127.0.0.1:5173').split(',').map(x=>x.trim()),adminToken:process.env.RUBI_ADMIN_TOKEN??'',experimentId:process.env.RUBI_EXPERIMENT_ID??'rubi-hpp-pilot-v1',studyStatus:process.env.RUBI_STUDY_STATUS??'draft'});
  service.server.listen(Number(process.env.RUBI_API_PORT??8787),'0.0.0.0',()=>console.log(`RUBI response service listening on port ${service.server.address().port}`));
  for(const signal of ['SIGTERM','SIGINT']) process.on(signal,()=>void service.close().then(()=>process.exit(0)));
}
