// Minimal static file server for local development (ES modules don't load from file://).
//   node tools/serve.mjs [port]
import { createServer } from 'node:http';
import { readFile, stat } from 'node:fs/promises';
import { extname, join, normalize, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const root = resolve(fileURLToPath(new URL('..', import.meta.url)));
const port = Number(process.argv[2] || process.env.PORT || 8080);
const types = {
  '.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8', '.mjs': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8', '.json': 'application/json; charset=utf-8', '.urdf': 'application/xml',
  '.xml': 'application/xml', '.dae': 'model/vnd.collada+xml', '.stl': 'model/stl', '.svg': 'image/svg+xml',
  '.png': 'image/png', '.jpg': 'image/jpeg', '.glb': 'model/gltf-binary', '.gltf': 'model/gltf+json',
};

createServer(async (req, res) => {
  try {
    const urlPath = decodeURIComponent(new URL(req.url, 'http://x').pathname);
    let file = normalize(join(root, urlPath));
    if (!file.startsWith(root)) { res.writeHead(403).end(); return; }
    if ((await stat(file)).isDirectory()) file = join(file, 'index.html');
    const body = await readFile(file);
    res.writeHead(200, { 'Content-Type': types[extname(file).toLowerCase()] || 'application/octet-stream', 'Cache-Control': 'no-cache' });
    res.end(body);
  } catch {
    res.writeHead(404, { 'Content-Type': 'text/plain' }).end('Not found');
  }
}).listen(port, () => console.log(`Serving ${root} at http://localhost:${port}/`));
