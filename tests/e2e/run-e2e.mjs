// End-to-end smoke test in headless Chrome/Edge (puppeteer-core, no bundled browser).
//   node tests/e2e/run-e2e.mjs [--fanuc <path-to-fanuc_m710ic_description>] [--headful]
// Screenshots go to tests/e2e/artifacts/.

import { spawn } from 'node:child_process';
import { existsSync, mkdirSync, readdirSync, statSync } from 'node:fs';
import { join, relative, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import puppeteer from 'puppeteer-core';

const root = resolve(fileURLToPath(new URL('../..', import.meta.url)));
const out = join(root, 'tests', 'e2e', 'artifacts');
mkdirSync(out, { recursive: true });
const args = process.argv.slice(2);
const fanucDir = args.includes('--fanuc') ? args[args.indexOf('--fanuc') + 1] : null;
const port = 8000 + Math.floor(Math.random() * 900);

const browsers = [
  process.env.CHROME_PATH,
  'C:/Program Files/Google/Chrome/Application/chrome.exe',
  'C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe',
  '/usr/bin/google-chrome', '/usr/bin/chromium', '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome',
].filter(Boolean);
const executablePath = browsers.find((p) => existsSync(p));
if (!executablePath) throw new Error('No Chrome/Edge found; set CHROME_PATH');

const server = spawn(process.execPath, [join(root, 'tools', 'serve.mjs'), String(port)], { stdio: 'pipe' });
await new Promise((r) => server.stdout.once('data', r));

const results = [];
const check = (name, ok, detail = '') => {
  results.push({ name, ok, detail });
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? `  — ${detail}` : ''}`);
};

const browser = await puppeteer.launch({
  executablePath,
  headless: !args.includes('--headful'),
  args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist', '--window-size=1440,900'],
  defaultViewport: { width: 1440, height: 900 },
});
const errors = [];
try {
  const page = await browser.newPage();
  page.on('console', (m) => { if (m.type() === 'error') errors.push(m.text()); });
  page.on('pageerror', (e) => errors.push(String(e)));
  await page.goto(`http://localhost:${port}/`, { waitUntil: 'networkidle0', timeout: 90000 });
  await page.waitForFunction(() => window.app && window.app.robot && window.app.robot.pending.length === 0 && !document.getElementById('loading').hidden === false, { timeout: 90000 });
  await page.evaluate(() => window.app.robot.ready);
  await new Promise((r) => setTimeout(r, 800));

  const info = await page.evaluate(() => {
    const r = window.app.robot;
    let meshes = 0;
    for (const l of r.links.values()) l.visual.traverse((c) => { if (c.isMesh) meshes++; });
    return {
      name: r.name, dof: r.controllable.length, meshes, missing: [...r.meshLoader.missing], failed: [...r.meshLoader.failed.keys()],
      tip: window.app.cart.chain.tipLink, sliders: document.querySelectorAll('#joint-panel .slider').length,
    };
  });
  check('default robot is UR10', info.name === 'ur10', JSON.stringify(info));
  check('6 joint sliders', info.sliders === 6);
  check('all UR10 meshes loaded', info.missing.length === 0 && info.failed.length === 0 && info.meshes > 0, `${info.meshes} meshes`);
  check('default end-effector is tool0', info.tip === 'tool0');
  await page.screenshot({ path: join(out, '01-ur10-joint.png') });

  // Visual DAE and collision STL must line up (validates the COLLADA up-axis handling).
  const align = await page.evaluate(async () => {
    const { Box3 } = await import('three');
    const r = window.app.robot;
    const rows = [];
    for (const [name, l] of r.links) {
      if (!l.visual.children.length || !l.collision.children.length) continue;
      l.collision.visible = true;
      const a = new Box3().setFromObject(l.visual, true);
      const b = new Box3().setFromObject(l.collision, true);
      l.collision.visible = false;
      if (a.isEmpty() || b.isEmpty()) continue;
      const d = Math.max(a.min.distanceTo(b.min), a.max.distanceTo(b.max));
      rows.push([name, d]);
    }
    return rows;
  });
  const worst = align.reduce((m, [, d]) => Math.max(m, d), 0);
  check('visual and collision meshes coincide (DAE up-axis)', align.length >= 6 && worst < 0.03, `worst bbox corner gap ${(worst * 1000).toFixed(1)} mm over ${align.length} links`);

  // Joint slider drives the joint.
  const moved = await page.evaluate(() => {
    const s = document.querySelector('#joint-panel .joint-row[data-joint="shoulder_pan_joint"] .slider');
    s.value = '45';
    s.dispatchEvent(new Event('input', { bubbles: true }));
    return window.app.robot.getJointValue('shoulder_pan_joint');
  });
  check('slider sets joint value', Math.abs(moved - Math.PI / 4) < 1e-3, `${moved}`);

  // Cartesian mode + jog.
  await page.click('.mode-switch [data-mode="cartesian"]');
  const jog = await page.evaluate(() => {
    const app = window.app;
    const before = app.cart.tcpInReference(app.getValues()).position.clone();
    const ok = app.cart.jogStep(2, 1, 'ref', 0.05, 0.1);
    const after = app.cart.tcpInReference(app.getValues()).position;
    return { ok, dz: after.z - before.z, dx: after.x - before.x, dy: after.y - before.y };
  });
  check('jog +Z 50 mm moves TCP straight up', jog.ok && Math.abs(jog.dz - 0.05) < 2e-4 && Math.abs(jog.dx) < 2e-4 && Math.abs(jog.dy) < 2e-4, JSON.stringify(jog));
  const goto = await page.evaluate(() => {
    const app = window.app;
    const res = app.cart.goToReferencePose([0.6, 0.3, 0.5], [Math.PI, 0, 0]);
    const p = app.cart.tcpInReference(app.getValues());
    return { converged: res.converged, p: p.position.toArray(), rpy: p.rpy };
  });
  check('go-to pose reaches [600, 300, 500] mm', goto.converged && Math.abs(goto.p[0] - 0.6) < 1e-3 && Math.abs(goto.p[1] - 0.3) < 1e-3 && Math.abs(goto.p[2] - 0.5) < 1e-3, JSON.stringify(goto));
  // Cartesian axis slider: drag Z up 80 mm in small steps (as a real drag would), X/Y/orientation must hold.
  const slid = await page.evaluate(() => {
    const app = window.app;
    const before = app.cart.tcpInReference(app.getValues());
    const s = document.querySelectorAll('#cartesian-panel .cart-axis-row .slider')[2];
    const z0 = Number(s.value);
    for (let k = 1; k <= 16; k++) {
      s.value = String(z0 + 5 * k);
      s.dispatchEvent(new Event('input', { bubbles: true }));
    }
    s.dispatchEvent(new Event('change', { bubbles: true }));
    const after = app.cart.tcpInReference(app.getValues());
    const dq = before.quaternion.angleTo(after.quaternion);
    return {
      sliders: document.querySelectorAll('#cartesian-panel .cart-axis-row .slider').length,
      dz: after.position.z - before.position.z, dx: after.position.x - before.position.x, dy: after.position.y - before.position.y, dq,
      field: Number(document.querySelectorAll('#cartesian-panel .cart-axis-row input.num')[2].value),
    };
  });
  check('Cartesian Z slider moves TCP 80 mm straight up', slid.sliders === 6 && Math.abs(slid.dz - 0.08) < 1e-3 && Math.abs(slid.dx) < 5e-4 && Math.abs(slid.dy) < 5e-4 && slid.dq < 2e-3, JSON.stringify(slid));
  await page.screenshot({ path: join(out, '02-ur10-cartesian.png') });

  // Program: two points, MoveL between them, run to completion.
  const prog = await page.evaluate(async () => {
    const app = window.app;
    app.programPanel.addPoint();
    app.cart.jogStep(0, -1, 'ref', 0.2, 0);
    app.cart.jogStep(2, 1, 'ref', 0.1, 0);
    app.programPanel.addPoint();
    app.programPanel.points[1].move = 'L';
    app.programPanel.points[1].speed = 1000;
    app.programPanel._render();
    app.programPanel._changed();
    const target = { ...app.programPanel.points[1].joints };
    app.setJoints(app.programPanel.points[0].joints, 'api');
    app.programPanel.run();
    const t0 = performance.now();
    await new Promise((resolve) => {
      const tick = () => (app.runner.state === 'done' || app.runner.state === 'error' || performance.now() - t0 > 20000 ? resolve() : setTimeout(tick, 50));
      tick();
    });
    const vals = app.getValues();
    const err = Math.max(...Object.keys(target).map((k) => Math.abs(vals[k] - target[k])));
    return { state: app.runner.state, err, points: app.programPanel.points.length, ms: performance.now() - t0, labels: document.querySelectorAll('.wp-label').length };
  });
  check('program runs MoveL to completion', prog.state === 'done' && prog.err < 2e-3, JSON.stringify(prog));
  await page.screenshot({ path: join(out, '03-ur10-program.png') });

  // Upload a second URDF package (Fanuc) through the file input.
  if (fanucDir && existsSync(fanucDir)) {
    const files = [];
    const walk = (d) => { for (const f of readdirSync(d)) { const p = join(d, f); if (statSync(p).isDirectory()) walk(p); else if (/\.(urdf|stl|dae)$/i.test(f)) files.push(p); } };
    walk(fanucDir);
    const input = await page.$('#file-input');
    await input.uploadFile(...files);
    await page.waitForFunction(() => window.app.robot && window.app.robot.name !== 'ur10', { timeout: 60000 });
    await page.evaluate(() => window.app.robot.ready);
    await new Promise((r) => setTimeout(r, 800));
    const f = await page.evaluate(() => {
      const r = window.app.robot;
      let meshes = 0;
      for (const l of r.links.values()) l.visual.traverse((c) => { if (c.isMesh) meshes++; });
      return { name: r.name, dof: r.controllable.length, meshes, missing: [...r.meshLoader.missing], tip: window.app.cart.chain.tipLink };
    });
    check('uploaded Fanuc URDF loads with 6 joints and meshes', f.dof === 6 && f.meshes > 0, JSON.stringify(f));
    await page.screenshot({ path: join(out, '04-fanuc.png') });
  }

  // Mobile layout renders without horizontal scroll.
  await page.setViewport({ width: 390, height: 844, isMobile: true, hasTouch: true });
  await new Promise((r) => setTimeout(r, 500));
  const overflow = await page.evaluate(() => document.documentElement.scrollWidth - document.documentElement.clientWidth);
  check('no horizontal scroll at 390 px', overflow <= 0, `overflow ${overflow}px`);
  await page.screenshot({ path: join(out, '05-mobile.png'), fullPage: true });

  const realErrors = errors.filter((e) => !/favicon/i.test(e));
  check('no console errors', realErrors.length === 0, realErrors.slice(0, 5).join(' | '));
} catch (e) {
  check('e2e run', false, e.stack || String(e));
} finally {
  await browser.close();
  server.kill();
}

const failed = results.filter((r) => !r.ok);
console.log(`\n${results.length - failed.length}/${results.length} checks passed. Screenshots: ${relative(root, out)}`);
process.exit(failed.length ? 1 : 0);
