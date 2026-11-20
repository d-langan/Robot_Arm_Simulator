// Application wiring: loading robots, mode switching, panels, HUD, shortcuts.

import { Matrix4, Vector3, Quaternion } from 'three';
import { Viewer } from './viewer.js';
import { parseURDF, URDFParseError } from './urdf/parser.js';
import { Robot } from './urdf/robot.js';
import { UrlResolver, FileResolver } from './urdf/assets.js';
import { CartesianController } from './cartesian.js';
import { ProgramRunner } from './program/program.js';
import { WaypointMarkers } from './program/markers.js';
import { JointPanel } from './ui/joint-panel.js';
import { CartesianPanel } from './ui/cartesian-panel.js';
import { ProgramPanel } from './ui/program-panel.js';
import { h, setChildren, toast, store, fmtJoint, jointUnit, RAD2DEG } from './ui/dom.js';
import { entriesFromFileList, entriesFromDataTransfer, expandZips, findRobotFiles } from './intake.js';

const FALLBACK_ROBOTS = [
  {
    id: 'ur10',
    name: 'Universal Robots UR10',
    urdf: 'robots/ur10/ur10.urdf',
    home: { shoulder_pan_joint: 0, shoulder_lift_joint: -1.5708, elbow_joint: 1.5708, wrist_1_joint: -1.5708, wrist_2_joint: -1.5708, wrist_3_joint: 0 },
  },
];
const VIEW_DEFAULTS = { visual: true, collision: false, frames: false, axes: false, skeleton: false, tcp: true, points: true, grid: true };
const MANUAL_SOURCES = new Set(['joint-panel', 'joint-panel-btn', 'gizmo', 'jog', 'cart-input', 'cart-slider']);

class App {
  constructor() {
    this.viewer = new Viewer(document.getElementById('viewport'));
    this.robot = null;
    this.mode = 'joint';
    this.builtins = FALLBACK_ROBOTS;
    this.view = { ...VIEW_DEFAULTS, ...(store.get('urdf-arm-sim:view', {}) || {}) };
    this.loadToken = 0;

    this.cart = new CartesianController(this.viewer, {
      getValues: () => this.getValues(),
      apply: (values, source) => this.setJoints(values, source),
    });
    this.runner = new ProgramRunner({
      robot: () => this.robot,
      chain: () => this.cart.chain,
      ikMode: () => this.cart.ikMode,
      getValues: () => this.getValues(),
      apply: (values) => this.setJoints(values, 'program'),
    });
    this.viewer.onFrame((dt) => this.runner.update(dt));
    this.markers = new WaypointMarkers(this.viewer.scene);

    this.jointPanel = new JointPanel(document.getElementById('joint-panel'), this);
    this.cartPanel = new CartesianPanel(document.getElementById('cartesian-panel'), this.cart, this);
    this.cartPanel.onChainChanged = () => this._refreshMarkers();
    this.programPanel = new ProgramPanel(document.getElementById('program-panel'), this.runner, {
      robot: () => this.robot,
      chain: () => this.cart.chain,
      getValues: () => this.getValues(),
      setJoints: (v, s) => this.setJoints(v, s),
      capturePose: () => this.capturePose(),
      onProgramChanged: (points, active, loop) => {
        this._program = { points, active, loop };
        this._refreshMarkers();
      },
    });
    this.cart.addEventListener('status', () => this._updateHud());

    this._initToolbar();
    this._initModeSwitch();
    this._initLoading();
    this._initShortcuts();
    this._applyView();
  }

  // ---------------------------------------------------------------- state

  getValues() {
    return this.robot ? this.robot.getJointValues() : {};
  }

  setJoints(values, source = 'api') {
    if (!this.robot) return;
    if (MANUAL_SOURCES.has(source) && this.runner.active) {
      this.runner.stop('Stopped: manual input');
      toast('Program stopped (manual input)', 'warn', 2000);
    }
    this.robot.setJointValues(values);
    this._onJointsChanged(source);
  }

  _onJointsChanged(source) {
    const values = this.getValues();
    this.jointPanel.refresh(values, source);
    if (this.mode === 'cartesian') this.cartPanel.refresh(values);
    this.cart.syncTarget();
    this._updateJointReadout(values);
    this._updateHud();
    this.viewer.requestRender();
  }

  capturePose() {
    const chain = this.cart.chain;
    if (!chain) return null;
    const T = chain.fk(chain.getQ(this.getValues()), new Matrix4());
    const p = new Vector3();
    const q = new Quaternion();
    T.decompose(p, q, new Vector3());
    return { frame: this.robot.model.root, tip: chain.tipLink, position: p.toArray(), quaternion: q.toArray() };
  }

  _refreshMarkers() {
    const prog = this._program || { points: [], active: -1, loop: false };
    this.markers.update(prog.points, this.cart.chain, this.getValues(), prog.active, prog.loop);
    this.markers.setVisible(this.view.points);
    this.viewer.requestRender();
  }

  // ---------------------------------------------------------------- loading

  async _initLoading() {
    const select = document.getElementById('robot-select');
    try {
      const res = await fetch('robots/index.json', { cache: 'no-cache' });
      if (res.ok) {
        const json = await res.json();
        if (Array.isArray(json.robots) && json.robots.length) this.builtins = json.robots;
      }
    } catch { /* keep fallback */ }
    select.replaceChildren(...this.builtins.map((r) => h('option', { value: `builtin:${r.id}` }, r.name)));
    select.addEventListener('change', () => {
      const v = select.value;
      if (v.startsWith('builtin:')) this.loadBuiltin(v.slice(8));
    });

    const fileInput = document.getElementById('file-input');
    const folderInput = document.getElementById('folder-input');
    document.getElementById('btn-open-files').addEventListener('click', () => fileInput.click());
    document.getElementById('btn-open-folder').addEventListener('click', () => folderInput.click());
    document.getElementById('btn-open-url').addEventListener('click', () => this._urlDialog());
    fileInput.addEventListener('change', async () => {
      if (fileInput.files.length) await this.loadEntries(entriesFromFileList(fileInput.files));
      fileInput.value = '';
    });
    folderInput.addEventListener('change', async () => {
      if (folderInput.files.length) await this.loadEntries(entriesFromFileList(folderInput.files));
      folderInput.value = '';
    });
    this._initDragDrop();

    const params = new URLSearchParams(location.search);
    if (params.get('urdf')) {
      const ok = await this.loadUrl(params.get('urdf'));
      if (ok) return;
    }
    const id = params.get('robot');
    await this.loadBuiltin(id && this.builtins.some((r) => r.id === id) ? id : this.builtins[0].id);
  }

  async loadBuiltin(id) {
    const def = this.builtins.find((r) => r.id === id) || this.builtins[0];
    const select = document.getElementById('robot-select');
    select.value = `builtin:${def.id}`;
    return this.loadUrl(def.urdf, { label: def.name, home: def.home, packages: def.packages, builtin: true });
  }

  async loadUrl(url, { label, home, packages, builtin = false } = {}) {
    this._setLoading(`Loading ${label || url}…`);
    let text;
    try {
      const res = await fetch(url);
      if (!res.ok) throw new Error(`HTTP ${res.status}`);
      text = await res.text();
    } catch (e) {
      this._setLoading(null);
      this._showError(`Could not fetch ${url}: ${e.message}`);
      return false;
    }
    const ok = await this._loadText(text, new UrlResolver(url, packages || {}), { label: label || url, home });
    if (ok && !builtin) this._markCustom(this.robot.name);
    return ok;
  }

  async loadEntries(entries) {
    this._setLoading('Reading files…');
    try {
      entries = await expandZips(entries);
    } catch (e) {
      this._setLoading(null);
      this._showError(`Could not read zip: ${e.message}`);
      return false;
    }
    const { urdfs, xacros, sdfs } = await findRobotFiles(entries);
    if (!urdfs.length) {
      this._setLoading(null);
      if (xacros.length) {
        this._showError(`Found ${xacros[0].path}, which is xacro. Expand it to plain URDF first: xacro ${xacros[0].path.split('/').pop()} > robot.urdf (ROS 2: ros2 run xacro xacro …, or pip install xacro).`);
      } else if (sdfs.length) {
        this._showError('Found an SDF file. This simulator reads URDF; convert SDF to URDF first.');
      } else {
        this._showError('No .urdf file found. Select the .urdf together with its mesh files, or a folder / .zip containing both.');
      }
      return false;
    }
    let chosen = urdfs[0];
    if (urdfs.length > 1) {
      this._setLoading(null);
      chosen = await this._chooseDialog('Several URDF files found', urdfs.map((u) => u.path));
      if (chosen === null) return false;
      chosen = urdfs[chosen];
    }
    const text = await chosen.file.text();
    const meshCount = entries.length - urdfs.length;
    const ok = await this._loadText(text, new FileResolver(entries, chosen.path), { label: chosen.path, fileCount: meshCount });
    if (ok) this._markCustom(this.robot.name);
    return ok;
  }

  async _loadText(text, resolver, { label, home, fileCount } = {}) {
    const token = ++this.loadToken;
    let model;
    try {
      model = parseURDF(text);
    } catch (e) {
      this._setLoading(null);
      this._showError(e instanceof URDFParseError ? e.message : `Failed to parse URDF: ${e.message}`);
      resolver.dispose();
      return false;
    }
    this._setLoading(`Building ${model.name}…`);
    this.runner.stop();
    if (this.robot) this.robot.dispose();
    const robot = new Robot(model, resolver);
    this.robot = robot;
    this.viewer.world.add(robot.object);
    if (home) robot.setJointValues(home);

    this.cart.setRobot(robot);
    this.jointPanel.setRobot(robot);
    this.cartPanel.setRobot(robot);
    this.programPanel.setRobot(robot);
    this._applyView();
    this._renderRobotCard({ label, loading: true });
    this._onJointsChanged('load');
    this.viewer.fitTo(robot.object, { keepDirection: false });

    await robot.ready;
    if (token !== this.loadToken) return false; // superseded by a newer load
    this._setLoading(null);
    const missing = [...robot.meshLoader.missing];
    const failed = [...robot.meshLoader.failed.entries()];
    let visualMeshes = 0;
    for (const l of robot.links.values()) l.visual.traverse((c) => { if (c.isMesh) visualMeshes++; });
    if (visualMeshes === 0 && !this.view.skeleton) {
      robot.setDisplay({ skeleton: true });
      this._setToggle('skeleton', true, false);
    }
    this._renderRobotCard({ label, missing, failed, fileCount });
    this.viewer.fitTo(robot.object, { keepDirection: false });
    this.cart.syncTarget();
    this._refreshMarkers();
    if (missing.length) {
      toast(`${missing.length} mesh file${missing.length === 1 ? '' : 's'} not found — load the URDF together with its mesh folder (or a .zip).`, 'warn', 7000);
    } else {
      toast(`Loaded ${model.name}: ${robot.controllable.length} joints`, 'ok', 2200);
    }
    return true;
  }

  _markCustom(name) {
    const select = document.getElementById('robot-select');
    let opt = select.querySelector('option[value="custom"]');
    if (!opt) {
      opt = h('option', { value: 'custom' });
      select.append(opt);
    }
    opt.textContent = `Uploaded: ${name}`;
    select.value = 'custom';
  }

  _setLoading(message) {
    const el = document.getElementById('loading');
    el.hidden = !message;
    if (message) el.querySelector('.loading-text').textContent = message;
  }

  _showError(message) {
    toast(message, 'error', 9000);
    const card = document.getElementById('robot-card');
    const err = h('div.error-box', { role: 'alert' }, message);
    card.querySelector('.error-box')?.remove();
    card.append(err);
  }

  _renderRobotCard({ label, missing = [], failed = [], loading = false, fileCount } = {}) {
    const r = this.robot;
    const card = document.getElementById('robot-card');
    if (!r) {
      setChildren(card, h('p.muted', 'No robot loaded.'));
      return;
    }
    const m = r.model;
    const fixed = [...m.joints.values()].filter((j) => j.type === 'fixed').length;
    const mimic = [...m.joints.values()].filter((j) => j.mimicResolved).length;
    const warnings = m.warnings;
    const issues = [
      ...missing.map((x) => `Missing mesh: ${x}`),
      ...failed.map(([x, e]) => `Could not load ${x}: ${e}`),
      ...warnings,
    ];
    setChildren(card, 
      h('div.robot-title', h('h1', m.name), h('span.muted.robot-src', { title: label }, label || '')),
      h('div.chips',
        h('span.chip', `${r.controllable.length} DOF`),
        h('span.chip', `${m.links.size} links`),
        h('span.chip', `${fixed} fixed`),
        mimic ? h('span.chip', `${mimic} mimic`) : null,
        h('span.chip', `${r.stats.meshes} meshes`),
        loading ? h('span.chip.chip-busy', 'loading meshes…') : null,
        !loading && missing.length ? h('span.chip.chip-warn', `${missing.length} missing`) : null,
      ),
      issues.length
        ? h('details.issues', h('summary', `${issues.length} note${issues.length === 1 ? '' : 's'} from the URDF`),
          h('ul', issues.slice(0, 200).map((x) => h('li', x))),
          missing.length ? h('p.hint', fileCount === undefined
            ? 'Mesh paths are resolved relative to the URDF URL (package:// assumes the URDF sits in <package>/urdf/).'
            : 'Tip: select the whole package folder (Open folder) or drop a .zip so mesh paths can be matched.') : null)
        : null,
    );
  }

  _initDragDrop() {
    const overlay = document.getElementById('drop-overlay');
    let depth = 0;
    const hasFiles = (e) => e.dataTransfer && [...e.dataTransfer.types].includes('Files');
    window.addEventListener('dragenter', (e) => {
      if (!hasFiles(e)) return;
      e.preventDefault();
      depth++;
      overlay.hidden = false;
    });
    window.addEventListener('dragover', (e) => {
      if (!hasFiles(e)) return;
      e.preventDefault();
      e.dataTransfer.dropEffect = 'copy';
    });
    window.addEventListener('dragleave', (e) => {
      if (!hasFiles(e)) return;
      depth = Math.max(0, depth - 1);
      if (depth === 0) overlay.hidden = true;
    });
    window.addEventListener('drop', async (e) => {
      if (!hasFiles(e)) return;
      e.preventDefault();
      depth = 0;
      overlay.hidden = true;
      const entries = await entriesFromDataTransfer(e.dataTransfer);
      if (entries.length) await this.loadEntries(entries);
    });
  }

  _dialog(build) {
    const dlg = document.getElementById('dialog');
    return new Promise((resolve) => {
      const close = (v) => {
        dlg.close();
        resolve(v);
      };
      setChildren(dlg, build(close));
      dlg.onclose = () => resolve(null);
      dlg.showModal();
    });
  }

  async _urlDialog() {
    const url = await this._dialog((close) => {
      const input = h('input.text-input', { type: 'url', placeholder: 'https://raw.githubusercontent.com/<user>/<repo>/main/<pkg>/urdf/robot.urdf', required: true, autofocus: true });
      const form = h('form', { method: 'dialog' },
        h('h2', 'Load URDF from URL'),
        h('p.muted', 'The server must allow cross-origin requests (raw.githubusercontent.com and GitHub Pages do). Relative and package:// mesh paths are resolved from the URDF location.'),
        input,
        h('div.btn-row.right',
          h('button.btn', { type: 'button', on: { click: () => close(null) } }, 'Cancel'),
          h('button.btn.btn-primary', { type: 'submit' }, 'Load')));
      form.addEventListener('submit', (e) => {
        e.preventDefault();
        if (input.value.trim()) close(input.value.trim());
      });
      return form;
    });
    if (url) await this.loadUrl(url);
  }

  _chooseDialog(title, options) {
    return this._dialog((close) => h('div',
      h('h2', title),
      h('div.choice-list', options.map((o, i) => h('button.btn.choice', { type: 'button', on: { click: () => close(i) } }, o))),
      h('div.btn-row.right', h('button.btn', { type: 'button', on: { click: () => close(null) } }, 'Cancel'))));
  }

  // ---------------------------------------------------------------- UI chrome

  _initModeSwitch() {
    for (const b of document.querySelectorAll('.mode-switch [data-mode]')) {
      b.addEventListener('click', () => this.setMode(b.dataset.mode));
    }
  }

  setMode(mode) {
    this.mode = mode === 'cartesian' ? 'cartesian' : 'joint';
    for (const b of document.querySelectorAll('.mode-switch [data-mode]')) {
      b.setAttribute('aria-selected', String(b.dataset.mode === this.mode));
    }
    document.getElementById('joint-panel').hidden = this.mode !== 'joint';
    document.getElementById('cartesian-panel').hidden = this.mode !== 'cartesian';
    document.getElementById('joint-readout').hidden = this.mode !== 'cartesian';
    this.cart.setEnabled(this.mode === 'cartesian');
    this.cart.setTcpVisible(this.mode === 'cartesian' || this.view.tcp);
    if (this.mode === 'cartesian') this.cartPanel.refresh(this.getValues(), true);
    this._updateHud();
  }

  _initToolbar() {
    for (const b of document.querySelectorAll('.view-toolbar [data-view]')) {
      b.addEventListener('click', () => this._setToggle(b.dataset.view, !this.view[b.dataset.view]));
    }
    document.getElementById('btn-fit').addEventListener('click', () => this.robot && this.viewer.fitTo(this.robot.object));
    document.getElementById('btn-help').addEventListener('click', () => this._help());
    document.getElementById('btn-theme').addEventListener('click', () => {
      const next = document.documentElement.dataset.theme === 'dark' ? 'light' : 'dark';
      if (next === 'dark') document.documentElement.dataset.theme = 'dark';
      else delete document.documentElement.dataset.theme;
      try { localStorage.setItem('urdf-arm-sim:theme', next); } catch { /* private mode */ }
      this.viewer.setTheme(next);
    });
  }

  _setToggle(key, value, persist = true) {
    this.view[key] = value;
    if (persist) store.set('urdf-arm-sim:view', this.view);
    this._applyView();
  }

  _applyView() {
    const v = this.view;
    for (const b of document.querySelectorAll('.view-toolbar [data-view]')) {
      b.setAttribute('aria-pressed', String(!!v[b.dataset.view]));
    }
    if (this.robot) {
      this.robot.setDisplay({ visual: v.visual, collision: v.collision, frames: v.frames, axes: v.axes, skeleton: v.skeleton });
    }
    this.viewer.grid.visible = v.grid;
    this.viewer.worldAxes.visible = v.grid;
    this.cart.setTcpVisible(this.mode === 'cartesian' || v.tcp);
    this.markers.setVisible(v.points);
    this.viewer.requestRender();
  }

  _initShortcuts() {
    window.addEventListener('keydown', (e) => {
      const t = e.target;
      if (t && (t.tagName === 'INPUT' || t.tagName === 'SELECT' || t.tagName === 'TEXTAREA' || t.isContentEditable)) return;
      if (e.ctrlKey || e.metaKey || e.altKey) return;
      const k = e.key.toLowerCase();
      if (k === 'j') this.setMode('joint');
      else if (k === 'c') this.setMode('cartesian');
      else if (k === 'w' && this.mode === 'cartesian') { this.cart.setGizmoMode('translate'); this.cartPanel.gizmoModeSeg?.setValue('translate'); }
      else if (k === 'e' && this.mode === 'cartesian') { this.cart.setGizmoMode('rotate'); this.cartPanel.gizmoModeSeg?.setValue('rotate'); }
      else if (k === 'q' && this.mode === 'cartesian') {
        const s = this.cart.gizmo.space === 'local' ? 'world' : 'local';
        this.cart.setGizmoSpace(s);
        this.cartPanel.gizmoSpaceSeg?.setValue(s);
      } else if (k === 'p') this.programPanel.addPoint();
      else if (k === ' ') { e.preventDefault(); this.programPanel.togglePause(); }
      else if (k === 'f') this.robot && this.viewer.fitTo(this.robot.object);
      else if (k === 'escape' && this.runner.active) this.runner.stop();
      else return;
    });
  }

  _updateJointReadout(values) {
    const el = document.getElementById('joint-readout');
    if (!this.robot || el.hidden) return;
    if (el.dataset.robot !== this.robot.name + this.robot.controllable.length) {
      el.dataset.robot = this.robot.name + this.robot.controllable.length;
      el.replaceChildren(h('h3', 'Joint values'), h('div.readout-grid', this.robot.controllable.map((j) =>
        h('div.readout-item', { dataset: { joint: j.name } }, h('span.readout-name', { title: j.name }, j.name), h('span.readout-val')))));
    }
    for (const item of el.querySelectorAll('.readout-item')) {
      const j = this.robot.model.joints.get(item.dataset.joint);
      const v = values[j.name] ?? 0;
      item.querySelector('.readout-val').textContent = `${fmtJoint(j, v)}${jointUnit(j)}`;
      const span = j.limit.upper - j.limit.lower;
      item.classList.toggle('at-limit', span > 0 && (v - j.limit.lower < span * 0.02 || j.limit.upper - v < span * 0.02));
    }
  }

  _updateHud() {
    const hud = document.getElementById('hud');
    if (!this.robot || !this.cart.chain) {
      hud.hidden = true;
      return;
    }
    hud.hidden = false;
    const pose = this.cart.tcpInReference(this.getValues());
    const p = pose.position;
    const r = pose.rpy.map((x) => x * RAD2DEG);
    const res = this.cart.lastResult;
    const ikBad = this.mode === 'cartesian' && res && !res.converged;
    setChildren(hud, 
      h('div.hud-title', `${this.cart.chain.tipLink}`, h('span.muted', ` in ${this.cart.reference}`)),
      h('div.hud-grid',
        h('span.axis-x', 'X'), h('span', (p.x * 1000).toFixed(1)),
        h('span.axis-y', 'Y'), h('span', (p.y * 1000).toFixed(1)),
        h('span.axis-z', 'Z'), h('span', (p.z * 1000).toFixed(1)),
        h('span.muted', 'mm'),
        h('span.axis-x', 'Rx'), h('span', r[0].toFixed(1)),
        h('span.axis-y', 'Ry'), h('span', r[1].toFixed(1)),
        h('span.axis-z', 'Rz'), h('span', r[2].toFixed(1)),
        h('span.muted', '°')),
      ikBad ? h('div.hud-warn', 'IK: target not reached') : null,
    );
  }

  _help() {
    this._dialog((close) => h('div.help',
      h('h2', 'Arm Simulator'),
      h('h3', 'Load a robot'),
      h('ul',
        h('li', h('b', 'Open files'), ' — pick the .urdf and its mesh files together (or a .zip).'),
        h('li', h('b', 'Open folder'), ' — pick the robot package folder; mesh paths (package://, relative, file://) are matched by path suffix.'),
        h('li', h('b', 'Drag & drop'), ' a .urdf, folder, or .zip anywhere on the page.'),
        h('li', h('b', 'URL'), ' — load a URDF hosted online; or link ', h('code', '?urdf=<url>'), ' directly.'),
        h('li', 'xacro files must be expanded first: ', h('code', 'xacro robot.urdf.xacro > robot.urdf'), '.')),
      h('h3', 'Control'),
      h('ul',
        h('li', h('b', 'Joint mode'), ' — sliders/fields per joint, limits from the URDF.'),
        h('li', h('b', 'Cartesian mode'), ' — drag the gizmo, type a pose, or jog ±X/Y/Z/Rx/Ry/Rz in the reference or tool frame. Step mode moves one increment per click; Hold mode moves while pressed.'),
        h('li', h('b', 'Program'), ' — save points, choose MoveJ (joint interpolation) or MoveL (straight line), set speed and dwell, then Run. Programs are saved in this browser per robot and can be exported as JSON.')),
      h('h3', 'Shortcuts'),
      h('div.kbd-grid',
        ...[['J / C', 'Joint / Cartesian mode'], ['W / E', 'Gizmo move / rotate'], ['Q', 'Gizmo tool / world axes'], ['P', 'Save point'], ['Space', 'Run / pause program'], ['Esc', 'Stop program'], ['F', 'Fit view']]
          .flatMap(([k, d]) => [h('kbd', k), h('span', d)])),
      h('div.btn-row.right', h('button.btn.btn-primary', { type: 'button', on: { click: () => close(null) } }, 'Close'))));
  }
}

window.app = new App();
