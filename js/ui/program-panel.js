// Program panel: teach points, edit move type / speed / dwell, reorder, run.

import { h, toast, downloadJSON, store } from './dom.js';
import { newPoint, serializeProgram, parseProgram, MOVE_DEFAULTS } from '../program/program.js';

export class ProgramPanel {
  /**
   * @param {HTMLElement} root
   * @param {ProgramRunner} runner
   * @param {{ getValues, setJoints, capturePose, robot, onProgramChanged, chain }} app
   */
  constructor(root, runner, app) {
    this.root = root;
    this.runner = runner;
    this.app = app;
    this.points = [];
    this.loop = false;
    this.activeIndex = -1;
    runner.addEventListener('state', (e) => this._onRunnerState(e.detail));
  }

  _key() {
    const r = this.app.robot();
    return r ? `urdf-arm-sim:program:${r.name}:${r.controllable.map((j) => j.name).join(',')}` : null;
  }

  _persist() {
    const key = this._key();
    if (!key) return;
    store.set(key, { points: this.points, loop: this.loop, override: this.runner.speedOverride });
  }

  setRobot(robot) {
    this.runner.stop();
    this.points = [];
    this.activeIndex = -1;
    // Per-robot state: nothing from the previous robot carries over.
    this.loop = false;
    this.defaultMove = undefined;
    this.runner.speedOverride = 1;
    this._lastMessage = '';
    if (robot) {
      const saved = store.get(this._key());
      if (saved && Array.isArray(saved.points)) {
        try {
          this.points = parseProgram({ format: 'urdf-arm-sim-program', robot: robot.name, points: saved.points }, robot).points;
          this.loop = !!saved.loop;
          if (Number.isFinite(saved.override)) this.runner.speedOverride = saved.override;
        } catch {
          this.points = [];
        }
      }
    }
    this._render();
    this._changed();
  }

  _changed() {
    this._persist();
    this.app.onProgramChanged(this.points, this.activeIndex, this.loop);
  }

  _nextName() {
    const used = new Set(this.points.map((p) => p.name));
    let i = this.points.length + 1;
    while (used.has(`P${i}`)) i++;
    return `P${i}`;
  }

  addPoint() {
    if (!this.app.robot()) return;
    const move = this.points.length ? (this.defaultMove || 'J') : 'J';
    this.points.push(newPoint({ name: this._nextName(), joints: this.app.getValues(), pose: this.app.capturePose(), move }));
    this._render();
    this._changed();
    const list = this.root.querySelector('.wp-list');
    if (list) list.scrollTop = list.scrollHeight;
  }

  _render() {
    const robot = this.app.robot();
    this.root.replaceChildren();
    const r = this.runner;
    const running = r.active;

    const saveBtn = h('button.btn.btn-primary', { type: 'button', disabled: !robot, title: 'Save the current pose as a new point (P)', on: { click: () => this.addPoint() } }, '+ Save point');
    const runBtn = h('button.btn', {
      type: 'button',
      disabled: !this.points.length,
      title: 'Run the program from the first point (Space pauses / resumes)',
      on: { click: () => (r.state === 'paused' ? r.resume() : r.state === 'running' || r.state === 'dwell' ? r.pause() : this.run()) },
    }, r.state === 'running' || r.state === 'dwell' ? '❚❚ Pause' : r.state === 'paused' ? '▶ Resume' : '▶ Run');
    const stopBtn = h('button.btn', { type: 'button', disabled: !running, on: { click: () => r.stop() } }, '■ Stop');
    this.runBtn = runBtn;
    this.stopBtn = stopBtn;
    const loopBox = h('input', { type: 'checkbox', checked: this.loop, 'aria-label': 'Loop program' });
    loopBox.addEventListener('change', () => { this.loop = loopBox.checked; r.loop = this.loop && !r.single; this._changed(); });

    const ovr = h('input', { type: 'range', min: 5, max: 100, step: 5, value: Math.round(r.speedOverride * 100), 'aria-label': 'Speed override' });
    const ovrOut = h('output', `${Math.round(r.speedOverride * 100)}%`);
    ovr.addEventListener('input', () => {
      r.speedOverride = Number(ovr.value) / 100;
      ovrOut.textContent = `${ovr.value}%`;
      this._persist();
    });

    this.statusEl = h('div.prog-status', { role: 'status', 'aria-live': 'polite' }, this._statusText());

    const list = h('ol.wp-list');
    if (!this.points.length) {
      list.append(h('li.wp-empty', 'No points yet. Move the robot (joint or Cartesian) and press ', h('b', '+ Save point'), '.'));
    }
    this.points.forEach((p, i) => list.append(this._row(p, i, running)));

    const importInput = h('input', { type: 'file', accept: '.json,application/json', hidden: true });
    importInput.addEventListener('change', () => {
      const f = importInput.files[0];
      importInput.value = ''; // so picking the same file again (after a failure or cancel) fires 'change'
      this._import(f);
    });

    this.root.append(
      h('div.panel-head', h('h2', 'Program'), h('span.muted', `${this.points.length} point${this.points.length === 1 ? '' : 's'}`)),
      h('div.btn-row', saveBtn, runBtn, stopBtn),
      h('div.prog-opts',
        h('label.check', loopBox, 'Loop'),
        h('label.ovr', 'Speed', ovr, ovrOut)),
      this.statusEl,
      list,
      h('div.btn-row',
        h('button.btn.btn-sm', { type: 'button', on: { click: () => importInput.click() } }, 'Import…'),
        h('button.btn.btn-sm', { type: 'button', disabled: !this.points.length, on: { click: () => this._export() } }, 'Export'),
        h('span.spacer'),
        (this.clearBtn = h('button.btn.btn-sm.btn-danger', { type: 'button', disabled: !this.points.length || running, on: { click: () => this._clear() } }, 'Clear'))),
      importInput,
    );
  }

  _row(p, i, running) {
    const name = h('input.wp-name', { type: 'text', value: p.name, 'aria-label': `Point ${i + 1} name`, maxlength: 32 });
    name.addEventListener('change', () => { p.name = name.value.trim() || `P${i + 1}`; this._changed(); });
    const move = h('select.wp-move', { 'aria-label': `Point ${i + 1} move type`, title: 'MoveJ: joint interpolation (fast, curved TCP path).\nMoveL: straight-line TCP path.' },
      h('option', { value: 'J', selected: p.move === 'J' }, 'MoveJ'),
      h('option', { value: 'L', selected: p.move === 'L' }, 'MoveL'));
    const speed = h('input.wp-speed.num', { type: 'number', min: 1, max: p.move === 'J' ? 100 : 5000, step: p.move === 'J' ? 5 : 10, value: p.speed, 'aria-label': `Point ${i + 1} speed` });
    const unit = h('span.unit', MOVE_DEFAULTS[p.move].unit);
    move.addEventListener('change', () => {
      p.move = move.value;
      p.speed = MOVE_DEFAULTS[p.move].speed;
      this.defaultMove = p.move;
      this._render();
      this._changed();
    });
    speed.addEventListener('change', () => {
      const v = Number(speed.value);
      p.speed = Number.isFinite(v) && v > 0 ? Math.min(p.move === 'J' ? 100 : 5000, v) : MOVE_DEFAULTS[p.move].speed;
      speed.value = p.speed;
      this._changed();
    });
    const dwell = h('input.wp-dwell.num', { type: 'number', min: 0, step: 0.1, value: p.dwell, 'aria-label': `Point ${i + 1} dwell seconds` });
    dwell.addEventListener('change', () => {
      const v = Number(dwell.value);
      p.dwell = Number.isFinite(v) && v >= 0 ? v : 0;
      dwell.value = p.dwell;
      this._changed();
    });

    // `edge`: disabled regardless of run state (first row's ↑, last row's ↓).
    const act = (label, title, fn, disabled = false, edge = false) => h('button.icon-btn.wp-act', { type: 'button', title, 'aria-label': title, disabled, dataset: edge ? { edge: '1' } : undefined, on: { click: fn } }, label);
    const li = h('li.wp-row', { class: `wp-row${i === this.activeIndex ? ' active' : ''}`, dataset: { index: i } },
      h('span.wp-idx', String(i + 1)),
      name,
      h('div.wp-params', move, speed, unit, h('span.wp-dwell-wrap', { title: 'Dwell (wait) after reaching this point' }, dwell, h('span.unit', 's'))),
      h('div.wp-actions',
        act('➜', `Move to ${p.name} (${p.move === 'L' ? 'MoveL' : 'MoveJ'})`, () => this.runner.start(this.points, { startIndex: i, single: true }), running),
        act('⦿', `Overwrite ${p.name} with the current pose`, () => {
          p.joints = { ...this.app.getValues() };
          p.pose = this.app.capturePose();
          this._changed();
          toast(`${p.name} updated`, 'ok', 1500);
        }, running),
        act('↑', 'Move up', () => this._move(i, -1), running || i === 0, i === 0),
        act('↓', 'Move down', () => this._move(i, 1), running || i === this.points.length - 1, i === this.points.length - 1),
        act('✕', `Delete ${p.name}`, () => { this.points.splice(i, 1); this._render(); this._changed(); }, running)),
    );
    return li;
  }

  _move(i, d) {
    const j = i + d;
    if (j < 0 || j >= this.points.length) return;
    [this.points[i], this.points[j]] = [this.points[j], this.points[i]];
    this._render();
    this._changed();
  }

  run() {
    if (!this.points.length) return;
    this.runner.start(this.points, { startIndex: 0, loop: this.loop });
  }

  togglePause() {
    const r = this.runner;
    if (r.state === 'paused') r.resume();
    else if (r.state === 'running' || r.state === 'dwell') r.pause();
    else this.run();
  }

  _clear() {
    if (!confirm(`Delete all ${this.points.length} points?`)) return;
    this.points = [];
    this._render();
    this._changed();
  }

  _export() {
    const r = this.app.robot();
    const chain = this.app.chain();
    downloadJSON(serializeProgram(r.model, this.points, { tip: chain ? chain.tipLink : null, loop: this.loop }), `${r.name}-program.json`);
  }

  async _import(file) {
    if (!file) return;
    try {
      const json = JSON.parse(await file.text());
      const { points, warnings } = parseProgram(json, this.app.robot());
      if (this.points.length && !confirm(`Replace the current ${this.points.length} points with ${points.length} imported points?`)) return;
      this.runner.stop();
      this.points = points;
      if (typeof json.loop === 'boolean') this.loop = json.loop;
      this._render();
      this._changed();
      toast(`Imported ${points.length} points`, 'ok');
      for (const w of warnings) toast(w, 'warn', 6000);
    } catch (e) {
      toast(`Import failed: ${e.message}`, 'error', 6000);
    }
  }

  _statusText() {
    const r = this.runner;
    const p = this.points[r.index];
    switch (r.state) {
      case 'running': return p ? `Moving to ${p.name} (${p.move === 'L' ? 'MoveL' : 'MoveJ'})…` : 'Running…';
      case 'dwell': return p ? `At ${p.name}, waiting ${p.dwell}s` : 'Waiting…';
      case 'paused': return 'Paused';
      case 'done': return r.single ? `Reached ${p ? p.name : 'point'}` : 'Program finished';
      case 'error': return this._lastMessage || 'Error';
      default: return this._lastMessage || '';
    }
  }

  _onRunnerState({ state, index, message }) {
    if (message) this._lastMessage = message;
    else if (state === 'running') this._lastMessage = '';
    const prevState = this._prevState;
    this._prevState = state;
    const prevActive = this.activeIndex;
    this.activeIndex = state === 'idle' || state === 'error' || state === 'done' ? -1 : index;
    if (state === 'error') toast(message, 'error', 7000);
    // Update in place: rebuilding the panel mid-run would destroy a field being typed in
    // or a speed-override drag.
    if (state !== prevState || this.activeIndex !== prevActive) {
      this._syncRunState();
      this.app.onProgramChanged(this.points, this.activeIndex, this.loop);
    }
    if (this.statusEl) {
      this.statusEl.textContent = this._statusText();
      this.statusEl.className = `prog-status ${state}`;
    }
  }

  /** Reflect the runner state on existing controls without rebuilding them. */
  _syncRunState() {
    const r = this.runner;
    const running = r.active;
    if (this.runBtn) {
      this.runBtn.textContent = r.state === 'running' || r.state === 'dwell' ? '❚❚ Pause' : r.state === 'paused' ? '▶ Resume' : '▶ Run';
      this.runBtn.disabled = !this.points.length;
    }
    if (this.stopBtn) this.stopBtn.disabled = !running;
    if (this.clearBtn) this.clearBtn.disabled = !this.points.length || running;
    for (const row of this.root.querySelectorAll('.wp-row')) {
      row.classList.toggle('active', Number(row.dataset.index) === this.activeIndex);
      for (const b of row.querySelectorAll('.wp-act')) b.disabled = running || b.dataset.edge === '1';
    }
  }
}
