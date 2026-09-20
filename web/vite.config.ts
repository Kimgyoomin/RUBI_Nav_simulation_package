import { defineConfig } from 'vite';
import fs from 'node:fs';
import path from 'node:path';

const roots = [
  ['ort', path.resolve('node_modules/onnxruntime-web/dist')],
  ['mujoco', path.resolve('node_modules/@mujoco/mujoco')],
] as const;

function binaries() {
  const files = new Map<string, string>();
  for (const [vendor, root] of roots) {
    const scan = (dir: string) => {
      for (const name of fs.readdirSync(dir)) {
        const p = path.join(dir, name);
        if (fs.statSync(p).isDirectory()) continue;
        if ((vendor === 'mujoco' && name === 'mujoco.wasm') || (vendor === 'ort' && /^ort-wasm-simd-threaded\.(wasm|mjs)$/.test(name))) files.set(`/vendor/${vendor}/${name}`, p);
      }
    };
    scan(root);
  }
  return {
    name: 'local-runtime-assets',
    configureServer(server: any) {
      server.middlewares.use((req: any, res: any, next: any) => {
        const file = files.get(req.url?.split('?')[0]);
        if (!file) return next();
        res.setHeader('Content-Type', file.endsWith('.wasm') ? 'application/wasm' : 'text/javascript');
        fs.createReadStream(file).pipe(res);
      });
    },
    closeBundle() {
      for (const [url, file] of files) {
        const target = path.join('dist', url);
        fs.mkdirSync(path.dirname(target), { recursive: true });
        fs.copyFileSync(file, target);
      }
    },
  };
}

export default defineConfig({
  base: process.env.GITHUB_PAGES === 'true' ? '/RUBI_Nav_simulation_package/' : './',
  plugins: [binaries()],
  optimizeDeps: { exclude: ['@mujoco/mujoco', 'onnxruntime-web'] },
  build: { target: 'es2022', chunkSizeWarningLimit: 2000 },
  server: { proxy: { '/api': 'http://127.0.0.1:8787' } },
});
