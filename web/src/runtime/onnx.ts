import * as ort from 'onnxruntime-web/wasm';
import type { Networks } from '../core/terrain-controller.ts';
import { finite } from '../core/terrain-controller.ts';

ort.env.wasm.numThreads=1;
ort.env.wasm.proxy=false;
ort.env.wasm.wasmPaths=new URL(`${import.meta.env.BASE_URL}vendor/ort/`,window.location.href).href;

export class OnnxNetworks implements Networks {
  encoder: ort.InferenceSession; policy: ort.InferenceSession;
  private constructor(encoder:ort.InferenceSession,policy:ort.InferenceSession) { this.encoder=encoder; this.policy=policy; }
  static async create(files:Map<string,Uint8Array>): Promise<OnnxNetworks> {
    const options:ort.InferenceSession.SessionOptions={executionProviders:['wasm'],graphOptimizationLevel:'all'};
    let encoder:ort.InferenceSession|undefined,policy:ort.InferenceSession|undefined;
    try {
      encoder=await ort.InferenceSession.create(files.get('encoder.onnx')!,options);
      policy=await ort.InferenceSession.create(files.get('policy.onnx')!,options);
      const networks=new OnnxNetworks(encoder,policy);
      networks.check(encoder,330,32,'encoder'); networks.check(policy,65,6,'policy');
      // Runtime probes check real tensor compatibility, not just filenames.
      await networks.encode(new Float32Array(330)); await networks.act(new Float32Array(65));
      return networks;
    } catch(error) { await encoder?.release(); await policy?.release(); throw new Error(`ONNX 연결 실패: ${String(error)}`); }
  }
  private check(session:ort.InferenceSession,input:number,output:number,label:string) {
    if(session.inputNames.length!==1 || session.outputNames.length!==1 || session.inputNames[0]!=='mlp_input' || session.outputNames[0]!=='mlp_output') throw new Error(`${label}: mlp_input / mlp_output 한 쌍이 필요합니다.`);
    const a=session.inputMetadata[0], b=session.outputMetadata[0];
    if(!a.isTensor || !b.isTensor || a.type!=='float32' || b.type!=='float32') throw new Error(`${label}: float32 tensor가 필요합니다.`);
    if(a.shape.length!==1 || a.shape[0]!==input || b.shape.length!==1 || b.shape[0]!==output) throw new Error(`${label}: 이 terrain 실행기는 [${input}] → [${output}]를 사용합니다. 선택한 모델의 shape가 다릅니다.`);
  }
  private async run(session:ort.InferenceSession,x:Float32Array,n:number):Promise<Float32Array> {
    const input=new ort.Tensor('float32',new Float32Array(x),[x.length]);
    let outputs:ort.InferenceSession.ReturnType|undefined;
    try {
      outputs=await session.run({mlp_input:input});
      const tensor=outputs.mlp_output;
      if(tensor.type!=='float32' || tensor.data.length!==n) throw new Error('예상한 출력 tensor와 다릅니다.');
      const data=new Float32Array(tensor.data as Float32Array);
      if(!finite(data)) throw new Error('정책에서 NaN 또는 무한대가 나왔습니다.');
      return data;
    } finally { input.dispose(); if(outputs) for(const value of Object.values(outputs)) value.dispose(); }
  }
  encode(x:Float32Array) { return this.run(this.encoder,x,32); }
  act(x:Float32Array) { return this.run(this.policy,x,6); }
  async dispose() { await this.encoder.release(); await this.policy.release(); }
}
