// Multi-robot browser test: loads several URDF packages as if they were
// folder uploads and checks each one end to end.
//
//   node tests/e2e/run-multi.mjs <manifest.json> [--headful]
//
// manifest.json: { "robots": [ { "name": "panda", "dir": "/abs/path/to/panda_description",
//                                "urdf": "urdf/panda.urdf", "zip": false } ] }
// `dir` is the package folder (uploaded with its folder name, like "Open folder").
// `urdf` is relative to `dir`; other .urdf files in the folder are left out.
// Screenshots: tests/e2e/artifacts/multi/<name>.png

import { createServer } from 'node:http';
import { readFile, stat } from 'node:fs/promises';
import { existsSync, mkdirSync, readdirSync, readFileSync, statSync } from 'node:fs';
import { basename, extname, join, normalize, relative, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import puppeteer from 'puppeteer-core';

const root = resolve(fileURLToPath(new URL('../..', import.meta.url)));
const out = join(root, 'tests', 'e2e', 'artifacts', 'multi');
mkdirSync(out, { recursive: true });
const args = process.argv.slice(2);
const manifest = JSON.parse(readFileSync(args[0], 'utf8'));

// Static server: project at /, each robot package at /__pkg/<i>/.
const mounts = manifest.robots.map((r) => resolve(r.dir));
const server = createServer(async (req, res) => {
  try {
    const p = decodeURIComponent(new URL(req.url, 'http://x').pathname);
    let base = root;
    let rest = p;
    const m = /^\/__pkg\/(\d+)\/(.*)$/.exec(p);
    if (m) { base = mounts[Number(m[1])]; rest = '/' + m[2]; }
    let file = normalize(join(base, rest));
    if (!file.startsWith(base)) throw new Error('outside');
    if ((await stat(file)).isDirectory()) file = join(file, 'index.html');
    const types = { '.html': 'text/html', '.js': 'text/javascript', '.css': 'text/css', '.json': 'application/json' };
    res.writeHead(200, { 'Content-Type': types[extname(file)] || 'application/octet-stream' });
    res.end(await readFile(file));
  } catch {
    res.writeHead(404).end();
  }
});
await new Promise((r) => server.listen(0, r));
const port = server.address().port;

const executablePath = [process.env.CHROME_PATH, 'C:/Program Files/Google/Chrome/Application/chrome.exe',
  'C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe', '/usr/bin/google-chrome', '/usr/bin/chromium']
  .filter(Boolean).find((p) => existsSync(p));
const browser = await puppeteer.launch({
  executablePath, headless: !args.includes('--headful'),
  args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'],
  defaultViewport: { width: 1280, height: 800 },
});

function listFiles(dir) {
  const outFiles = [];
  const walk = (d) => {
    for (const f of readdirSync(d)) {
      const p = join(d, f);
      if (statSync(p).isDirectory()) walk(p);
      else outFiles.push(relative(dir, p).replace(/\\/g, '/'));
    }
  };
  walk(dir);
  return outFiles;
}

/** Independent count of user-controllable joints straight from the XML text. */
function expectedDof(urdfText) {
  const body = urdfText.replace(/<!--[\s\S]*?-->/g, '');
  // Only top-level <joint ... type="..."> elements (transmission joints have no type attribute).
  const joints = [...body.matchAll(/<joint\b[^>]*\btype\s*=\s*"(revolute|continuous|prismatic)"[^>]*>([\s\S]*?)<\/joint>/g)];
  return joints.filter((m) => !/<mimic\b/.test(m[2])).length;
}

const summary = [];
const page = await browser.newPage();
const errors = [];
page.on('console', (m) => { if (m.type() === 'error') errors.push(m.text()); });
page.on('pageerror', (e) => errors.push(String(e)));
await page.goto(`http://localhost:${port}/`, { waitUntil: 'networkidle0', timeout: 90000 });
await page.waitForFunction(() => window.app && window.app.robot, { timeout: 90000 });
await page.evaluate(() => window.app.robot.ready);

for (const [i, r] of manifest.robots.entries()) {
  errors.length = 0;
  const row = { name: r.name, checks: {}, info: {} };
  const ok = (k, v, detail) => { row.checks[k] = v; if (detail !== undefined) row.info[k] = detail; };
  try {
    const dir = mounts[i];
    const folder = basename(dir);
    const files = listFiles(dir).filter((f) => !/\.(urdf|xacro)$/i.test(f) || f === r.urdf);
    const exp = expectedDof(readFileSync(join(dir, r.urdf), 'utf8'));

    const t0 = Date.now();
    const loaded = await page.evaluate(async ({ i, folder, files }) => {
      const entries = await Promise.all(files.map(async (f) => {
        const res = await fetch(`/__pkg/${i}/${f.split('/').map(encodeURIComponent).join('/')}`);
        return { path: `${folder}/${f}`, file: new Blob([await res.arrayBuffer()]) };
      }));
      const okLoad = await window.app.loadEntries(entries);
      return okLoad;
    }, { i, folder, files });
    ok('loads', loaded === true);
    await page.evaluate(() => window.app.robot.ready);
    row.info.loadMs = Date.now() - t0;

    const info = await page.evaluate(async () => {
      const { Box3, Vector3 } = await import('three');
      const app = window.app;
      const rb = app.robot;
      let meshes = 0;
      for (const l of rb.links.values()) l.visual.traverse((c) => { if (c.isMesh) meshes++; });
      // Mesh orientation sanity: collision geometry is often simplified, so a raw
      // bounding-box gap proves little. Flag a link only when turning its visual by
      // 90° about a link axis would match the collision box much better — the
      // signature of a mis-handled up-axis or a swapped mesh frame.
      const { Euler } = await import('three');
      let worstRel = 0; let worstLink = ''; let compared = 0;
      const gapOf = (a, b, size) => Math.max(a.min.distanceTo(b.min), a.max.distanceTo(b.max)) / size;
      for (const [name, l] of rb.links) {
        if (!l.visual.children.length || !l.collision.children.length) continue;
        const was = l.collision.visible;
        l.collision.visible = true;
        const b = new Box3().setFromObject(l.collision, true);
        const a = new Box3().setFromObject(l.visual, true);
        if (a.isEmpty() || b.isEmpty()) { l.collision.visible = was; continue; }
        const size = Math.max(a.getSize(new Vector3()).length(), b.getSize(new Vector3()).length(), 1e-6);
        const g0 = gapOf(a, b, size);
        let gRot = Infinity;
        const q0 = l.visual.quaternion.clone();
        for (const e of [[Math.PI / 2, 0, 0], [-Math.PI / 2, 0, 0], [0, Math.PI / 2, 0], [0, -Math.PI / 2, 0]]) {
          l.visual.quaternion.copy(q0).multiply(new (q0.constructor)().setFromEuler(new Euler(...e)));
          l.visual.updateMatrixWorld(true);
          gRot = Math.min(gRot, gapOf(new Box3().setFromObject(l.visual, true), b, size));
        }
        l.visual.quaternion.copy(q0);
        l.visual.updateMatrixWorld(true);
        l.collision.visible = was;
        compared++;
        const suspicious = g0 > 0.15 && gRot < g0 * 0.35 ? g0 : 0;
        if (suspicious > worstRel) { worstRel = suspicious; worstLink = `${name} (rotated fits ${(gRot * 100).toFixed(0)}% vs ${(g0 * 100).toFixed(0)}%)`; }
      }
      return {
        robot: rb.name, dof: rb.controllable.length, meshes, missing: [...rb.meshLoader.missing], failed: [...rb.meshLoader.failed.entries()],
        tip: app.cart.chain.tipLink, chainDof: app.cart.chain.dof, ikMode: app.cart.ikMode, warnings: rb.model.warnings.length,
        worstRel, worstLink, compared,
      };
    });
    Object.assign(row.info, info, { expectedDof: exp });
    ok('dof matches URDF', info.dof === exp, `${info.dof} vs ${exp}`);
    if (r.expectTip) {
      const want = Array.isArray(r.expectTip) ? r.expectTip : [r.expectTip];
      ok('default end-effector', want.includes(info.tip), `${info.tip}, expected ${want.join(' or ')}`);
    }
    ok('no missing meshes', info.missing.length === 0 && info.failed.length === 0, [...info.missing, ...info.failed.map((f) => f.join(': '))].slice(0, 4).join(' | '));
    ok('meshes oriented (no 90° misfit)', info.worstRel === 0, info.worstRel ? `suspicious: ${info.worstLink}` : `${info.compared} links compared`);

    // IK round trip: random reachable pose (seeded), solve from the zero/home pose.
    const ik = await page.evaluate((seed) => {
      const app = window.app;
      const chain = app.cart.chain;
      if (!chain || chain.dof === 0) return { skipped: true };
      let s = seed * 9301 + 49297;
      const rnd = () => ((s = (s * 9301 + 49297) % 233280) / 233280);
      let tries = 0; let conv = 0; let worst = 0;
      for (let k = 0; k < 6; k++) {
        const q = chain.variables.map((j) => {
          const lo = Math.max(j.limit.lower, j.type === 'prismatic' ? j.limit.lower : -1.6);
          const hi = Math.min(j.limit.upper, j.type === 'prismatic' ? j.limit.upper : 1.6);
          return lo + (hi - lo) * rnd();
        });
        const goal = {};
        chain.toValues(q, goal);
        app.setJoints(goal, 'api');
        const pose = app.cart.tcpInReference(app.getValues());
        const zero = {};
        for (const j of app.robot.controllable) zero[j.name] = Math.min(j.limit.upper, Math.max(j.limit.lower, 0));
        app.setJoints(zero, 'api');
        const res = app.cart.goToReferencePose(pose.position.toArray(), pose.rpy);
        tries++;
        if (res.converged) conv++;
        const got = app.cart.tcpInReference(app.getValues());
        worst = Math.max(worst, got.position.distanceTo(pose.position));
      }
      return { tries, conv, worstPosErrMm: worst * 1000, mode: app.cart.ikMode };
    }, i + 1);
    row.info.ik = ik;
    ok('IK reaches random reachable poses', ik.skipped || ik.conv >= ik.tries - 1, `${ik.conv}/${ik.tries}`);

    // Program: P1 zero, P2 random (MoveJ), P3 = P2 + 30 mm along reference Z (MoveL).
    const prog = await page.evaluate(async () => {
      const app = window.app;
      const pp = app.programPanel;
      const chain = app.cart.chain;
      pp.points = [];
      const zero = {};
      for (const j of app.robot.controllable) zero[j.name] = Math.min(j.limit.upper, Math.max(j.limit.lower, 0));
      app.setJoints(zero, 'api');
      pp.addPoint();
      const mid = {};
      for (const j of app.robot.controllable) {
        const lo = Math.max(j.limit.lower, j.type === 'prismatic' ? j.limit.lower : -0.8);
        const hi = Math.min(j.limit.upper, j.type === 'prismatic' ? j.limit.upper : 0.8);
        mid[j.name] = lo + (hi - lo) * 0.62;
      }
      app.setJoints(mid, 'api');
      pp.addPoint();
      let jogged = false;
      if (chain && chain.dof > 0) {
        for (const sgn of [1, -1]) { if (app.cart.jogStep(2, sgn, 'ref', 0.03, 0)) { jogged = true; break; } }
      }
      if (jogged) { pp.addPoint(); pp.points[2].move = 'L'; pp.points[2].speed = 500; }
      for (const p of pp.points) p.speed = p.move === 'L' ? 500 : 100;
      pp._render();
      pp._changed();
      app.setJoints(zero, 'api');
      app.runner.speedOverride = 1;
      pp.run();
      const t0 = performance.now();
      await new Promise((res) => {
        const tick = () => (['done', 'error', 'idle'].includes(app.runner.state) || performance.now() - t0 > 45000 ? res() : setTimeout(tick, 50));
        tick();
      });
      const last = pp.points[pp.points.length - 1].joints;
      const vals = app.getValues();
      const err = Math.max(...Object.keys(last).map((k) => Math.abs(vals[k] - last[k])));
      return { state: app.runner.state, message: pp._lastMessage || '', points: pp.points.length, moveL: jogged, err, ms: Math.round(performance.now() - t0) };
    });
    row.info.program = prog;
    ok('program runs (MoveJ' + (prog.moveL ? ' + MoveL' : '') + ')', prog.state === 'done' && prog.err < 5e-3, `${prog.state} ${prog.message} err=${prog.err.toExponential(1)}`);

    await page.evaluate(() => window.app.viewer.fitTo(window.app.robot.object));
    await new Promise((res) => setTimeout(res, 400));
    await page.screenshot({ path: join(out, `${r.name}.png`) });
    const errs = errors.filter((e) => !/favicon/i.test(e));
    ok('no console errors', errs.length === 0, errs.slice(0, 3).join(' | '));
  } catch (e) {
    ok('harness', false, String(e.stack || e).slice(0, 400));
  }
  summary.push(row);
  const failedChecks = Object.entries(row.checks).filter(([, v]) => !v).map(([k]) => k);
  console.log(`${failedChecks.length ? 'FAIL' : 'PASS'}  ${r.name.padEnd(18)} dof=${row.info.dof ?? '?'} tip=${row.info.tip ?? '?'} meshes=${row.info.meshes ?? '?'} ik=${row.info.ik ? `${row.info.ik.conv}/${row.info.ik.tries}` : '?'} prog=${row.info.program ? row.info.program.state : '?'} load=${row.info.loadMs ?? '?'}ms${failedChecks.length ? `\n      failed: ${failedChecks.map((k) => `${k} (${row.info[k] ?? ''})`).join('; ')}` : ''}`);
}

await browser.close();
server.close();
const bad = summary.filter((r) => Object.values(r.checks).some((v) => !v));
console.log(`\n${summary.length - bad.length}/${summary.length} robots passed every check. Screenshots: ${relative(root, out)}`);
console.log(JSON.stringify(summary.map((r) => ({ name: r.name, ...r.info })), null, 1).slice(0, 6000));
process.exit(bad.length ? 1 : 0);
