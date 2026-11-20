// Cartesian control panel: end-effector / reference selection, pose readout and
// entry, jog buttons (step or hold-to-move), gizmo mode, TCP offset.

import { h, holdButton, RAD2DEG, DEG2RAD, store } from './dom.js';

const AXES = ['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz'];
const LIN_STEPS = [0.1, 1, 5, 10, 50, 100];     // mm
const ROT_STEPS = [0.1, 1, 5, 15, 45, 90];      // deg

export class CartesianPanel {
  /**
   * @param {HTMLElement} root
   * @param {CartesianController} cart
   * @param {{ getValues: () => object }} app
   */
  constructor(root, cart, app) {
    this.root = root;
    this.cart = cart;
    this.app = app;
    this.prefs = store.get('urdf-arm-sim:cart-prefs', {}) || {};
    this.jogFrame = this.prefs.jogFrame || 'ref';
    this.jogMode = this.prefs.jogMode || 'step';
    this.linStep = this.prefs.linStep ?? 10;
    this.rotStep = this.prefs.rotStep ?? 5;
    cart.jogSpeed.linear = (this.prefs.linSpeed ?? 100) / 1000;
    cart.jogSpeed.angular = (this.prefs.rotSpeed ?? 20) * DEG2RAD;
    cart.addEventListener('status', (e) => this._status(e.detail));
    cart.addEventListener('chain', () => this.refresh(this.app.getValues()));
  }

  _save() {
    store.set('urdf-arm-sim:cart-prefs', {
      jogFrame: this.jogFrame, jogMode: this.jogMode, linStep: this.linStep, rotStep: this.rotStep,
      linSpeed: this.cart.jogSpeed.linear * 1000, rotSpeed: this.cart.jogSpeed.angular * RAD2DEG,
    });
  }

  setRobot(robot) {
    this.robot = robot;
    this.root.replaceChildren();
    if (!robot || !this.cart.chain) return;
    const cart = this.cart;

    // End-effector / reference / orientation.
    const tipSel = h('select#cart-tip', { 'aria-label': 'End-effector link' },
      robot.movingLinks().map((l) => h('option', { value: l, selected: l === cart.chain.tipLink }, l)));
    tipSel.addEventListener('change', () => {
      cart.setTip(tipSel.value);
      this._renderDof();
      this.refresh(this.app.getValues());
      this.onChainChanged && this.onChainChanged();
    });
    const refSel = h('select#cart-ref', { 'aria-label': 'Reference frame' },
      robot.staticLinks().map((l) => h('option', { value: l, selected: l === cart.reference }, l)));
    refSel.addEventListener('change', () => {
      cart.setReference(refSel.value);
      this.refresh(this.app.getValues());
    });
    this.oriSel = h('select#cart-ori', { 'aria-label': 'IK orientation mode' },
      h('option', { value: 'full' }, 'Position + orientation'),
      h('option', { value: 'position' }, 'Position only'));
    this.oriSel.value = cart.ikMode;
    this.oriSel.addEventListener('change', () => {
      cart.setIkMode(this.oriSel.value);
      this.refresh(this.app.getValues());
    });
    this.dofNote = h('p.hint');

    // Pose axes: slider (live IK while dragging) + numeric field (go-to with IK restarts).
    this.linRange = Math.ceil(Math.max(0.2, cart.chain.reach()) * 110) * 10; // mm, ~1.1 × reach
    this.fields = [];
    this.sliders = [];
    this.dragAxis = -1;
    const poseGrid = h('div.cart-axes');
    AXES.forEach((name, i) => {
      const lin = i < 3;
      const range = lin ? this.linRange : 180;
      const inp = h('input.num', { type: 'number', step: lin ? 1 : 0.5, 'aria-label': `${name} ${lin ? 'mm' : 'degrees'}` });
      inp.addEventListener('keydown', (e) => { if (e.key === 'Enter') this._goTo(); });
      inp.addEventListener('change', () => this._goTo());
      const sl = h('input.slider', { type: 'range', min: -range, max: range, step: lin ? 0.5 : 0.1, 'aria-label': `${name} slider (${lin ? 'mm' : 'degrees'})` });
      sl.addEventListener('pointerdown', () => { this.dragAxis = i; });
      sl.addEventListener('input', () => {
        this.dragAxis = i;
        this._slide(i, Number(sl.value));
      });
      sl.addEventListener('change', () => this._slideEnd());
      this.fields.push(inp);
      this.sliders.push(sl);
      poseGrid.append(h('div.cart-axis-row',
        h('span', { class: `axis-tag axis-${'xyz'[i % 3]}` }, name),
        sl,
        inp,
        h('span.unit', lin ? 'mm' : '°')));
    });

    // Jog controls.
    const frameSeg = this._segmented('jog-frame', [['ref', 'Reference'], ['tool', 'Tool']], this.jogFrame, (v) => { this.jogFrame = v; this._save(); });
    const modeSeg = this._segmented('jog-mode', [['step', 'Step'], ['hold', 'Hold']], this.jogMode, (v) => { this.jogMode = v; this._renderJogSettings(); this._save(); });
    this.jogSettings = h('div.jog-settings');
    const jogGrid = h('div.jog-grid');
    AXES.forEach((name, axis) => {
      for (const sign of [-1, 1]) {
        const btn = h('button.btn.jog-btn', {
          type: 'button',
          class: `btn jog-btn axis-${'xyz'[axis % 3]}`,
          title: `${sign < 0 ? '−' : '+'}${name} (${axis < 3 ? 'translate' : 'rotate'})`,
        }, `${sign < 0 ? '−' : '+'}${name}`);
        holdButton(btn, () => {
          if (this.jogMode === 'step') {
            cart.jogStep(axis, sign, this.jogFrame, this.linStep / 1000, this.rotStep * DEG2RAD);
          } else {
            cart.startJog(axis, sign, this.jogFrame);
          }
        }, () => {
          if (this.jogMode === 'hold') cart.stopJog();
        });
        jogGrid.append(btn);
      }
    });

    // Gizmo.
    const gizmoMode = this._segmented('gizmo-mode', [['translate', 'Move (W)'], ['rotate', 'Rotate (E)']], 'translate', (v) => cart.setGizmoMode(v));
    const gizmoSpace = this._segmented('gizmo-space', [['local', 'Tool axes'], ['world', 'World axes']], 'local', (v) => cart.setGizmoSpace(v));
    this.gizmoModeSeg = gizmoMode;
    this.gizmoSpaceSeg = gizmoSpace;

    // TCP offset.
    const off = cart.tcpOffset;
    const offInputs = [0, 1, 2, 3, 4, 5].map((i) => h('input.num', {
      type: 'number', step: i < 3 ? 1 : 1,
      value: i < 3 ? (off.xyz[i] * 1000).toFixed(1) : (off.rpy[i - 3] * RAD2DEG).toFixed(1),
      'aria-label': `TCP offset ${AXES[i]}`,
    }));
    const applyOffset = () => {
      const xyz = offInputs.slice(0, 3).map((x) => (Number(x.value) || 0) / 1000);
      const rpy = offInputs.slice(3).map((x) => (Number(x.value) || 0) * DEG2RAD);
      cart.setTcpOffset(xyz, rpy);
      this.refresh(this.app.getValues());
      this.onChainChanged && this.onChainChanged();
    };
    offInputs.forEach((x) => x.addEventListener('change', applyOffset));

    this.statusEl = h('div.ik-status', { role: 'status', 'aria-live': 'polite' });

    this.root.append(
      h('div.panel-head', h('h2', 'Cartesian'), this.statusEl),
      h('div.form-grid',
        h('label', 'End-effector', tipSel),
        h('label', 'Reference frame', refSel),
        h('label', 'IK target', this.oriSel),
      ),
      this.dofNote,
      h('h3', 'TCP pose ', h('span.muted', '(in reference frame, RPY fixed-axis XYZ)')),
      poseGrid,
      h('div.btn-row.right',
        h('button.btn.btn-sm', { type: 'button', on: { click: () => this.refresh(this.app.getValues(), true) } }, 'Reset fields'),
        h('button.btn.btn-sm.btn-primary', { type: 'button', on: { click: () => this._goTo() } }, 'Go to pose')),
      h('h3', 'Jog'),
      h('div.seg-row', frameSeg, modeSeg),
      this.jogSettings,
      jogGrid,
      h('h3', 'Gizmo ', h('span.muted', 'drag the target in the 3D view')),
      h('div.seg-row', gizmoMode, gizmoSpace),
      h('details.subtle',
        h('summary', 'TCP offset from end-effector link'),
        h('div.pose-grid',
          AXES.map((name, i) => h('label.pose-field',
            h('span', { class: `axis-tag axis-${'xyz'[i % 3]}` }, name), offInputs[i], h('span.unit', i < 3 ? 'mm' : '°'))))),
    );
    this._renderJogSettings();
    this._renderDof();
    this.refresh(this.app.getValues(), true);
    this._status({ result: null, message: '' });
  }

  _segmented(name, options, value, onChange) {
    const wrap = h('div.segmented', { role: 'radiogroup', 'aria-label': name });
    const buttons = options.map(([v, label]) => {
      const b = h('button', { type: 'button', role: 'radio', 'aria-checked': String(v === value), dataset: { value: v } }, label);
      b.addEventListener('click', () => {
        for (const o of buttons) o.setAttribute('aria-checked', String(o === b));
        onChange(v);
      });
      return b;
    });
    wrap.append(...buttons);
    wrap.setValue = (v) => { for (const o of buttons) o.setAttribute('aria-checked', String(o.dataset.value === v)); };
    return wrap;
  }

  _renderJogSettings() {
    const cart = this.cart;
    this.jogSettings.replaceChildren();
    if (this.jogMode === 'step') {
      const lin = h('select', { 'aria-label': 'Linear step' }, LIN_STEPS.map((s) => h('option', { value: s, selected: s === this.linStep }, `${s} mm`)));
      const rot = h('select', { 'aria-label': 'Rotation step' }, ROT_STEPS.map((s) => h('option', { value: s, selected: s === this.rotStep }, `${s}°`)));
      lin.addEventListener('change', () => { this.linStep = Number(lin.value); this._save(); });
      rot.addEventListener('change', () => { this.rotStep = Number(rot.value); this._save(); });
      this.jogSettings.append(h('label', 'Step', lin), h('label', 'Rotation step', rot));
    } else {
      const lin = h('input', { type: 'range', min: 5, max: 500, step: 5, value: Math.round(cart.jogSpeed.linear * 1000), 'aria-label': 'Linear jog speed' });
      const rot = h('input', { type: 'range', min: 1, max: 90, step: 1, value: Math.round(cart.jogSpeed.angular * RAD2DEG), 'aria-label': 'Rotation jog speed' });
      const linOut = h('output', `${lin.value} mm/s`);
      const rotOut = h('output', `${rot.value}°/s`);
      lin.addEventListener('input', () => { cart.jogSpeed.linear = Number(lin.value) / 1000; linOut.textContent = `${lin.value} mm/s`; this._save(); });
      rot.addEventListener('input', () => { cart.jogSpeed.angular = Number(rot.value) * DEG2RAD; rotOut.textContent = `${rot.value}°/s`; this._save(); });
      this.jogSettings.append(h('label', 'Speed', lin, linOut), h('label', 'Rotation', rot, rotOut));
    }
  }

  _renderDof() {
    const n = this.cart.chain ? this.cart.chain.dof : 0;
    this.oriSel.value = this.cart.ikMode;
    let note = `${n} joint${n === 1 ? '' : 's'} drive this end-effector.`;
    if (n === 0) note = 'No movable joints between the root and this link — pick another end-effector.';
    else if (n < 6) note += ' Fewer than 6, so full-pose targets are generally unreachable; position-only IK is selected.';
    else if (n > 6) note += ' Redundant chain: IK picks the smallest joint motion.';
    this.dofNote.textContent = note;
  }

  _goTo() {
    const vals = this.fields.map((f) => Number(f.value));
    if (vals.some((v) => !Number.isFinite(v))) return;
    const pos = vals.slice(0, 3).map((v) => v / 1000);
    const rpy = vals.slice(3).map((v) => v * DEG2RAD);
    this.cart.goToReferencePose(pos, rpy);
    this.refresh(this.app.getValues(), true);
  }

  /** Slider drag: replace one axis of the current target and track it with local IK. */
  _slide(i, v) {
    // Freeze the other five axes at drag start; re-reading them from the live pose
    // would let IK residuals accumulate into drift over a long drag.
    if (!this.dragBase || this.dragBaseAxis !== i) {
      this.dragBase = this.fields.map((f) => Number(f.value));
      this.dragBaseAxis = i;
    }
    const vals = this.dragBase.slice();
    vals[i] = v;
    if (vals.some((x) => !Number.isFinite(x))) return;
    this.fields[i].value = v.toFixed(i < 3 ? 1 : 2);
    this._setFill(i);
    this.cart.trackReferencePose(vals.slice(0, 3).map((x) => x / 1000), vals.slice(3).map((x) => x * DEG2RAD));
  }

  _slideEnd() {
    const res = this.cart.lastResult;
    this.dragAxis = -1;
    this.dragBase = null;
    if (res && !res.converged) this.cart._emitStatus('Out of reach — sliders returned to the closest pose');
    this.refresh(this.app.getValues(), true);
  }

  _setFill(i) {
    const s = this.sliders[i];
    const min = Number(s.min), max = Number(s.max);
    const pct = ((Math.min(max, Math.max(min, Number(s.value))) - min) / (max - min)) * 100;
    s.style.setProperty('--pct', `${pct}%`);
  }

  /** Update pose fields and sliders from the robot. The axis being edited/dragged is left alone unless `force`. */
  refresh(values, force = false) {
    if (!this.fields || !this.cart.chain) return;
    const pose = this.cart.tcpInReference(values);
    const vals = [pose.position.x * 1000, pose.position.y * 1000, pose.position.z * 1000, ...pose.rpy.map((r) => r * RAD2DEG)];
    this.fields.forEach((f, i) => {
      let v = vals[i];
      if (i >= 3) {
        // Keep angles continuous with what is shown (±180° is the same angle; avoid the display flipping sign).
        const prev = Number(f.value);
        if (f.value !== '' && Number.isFinite(prev)) {
          for (const c of [v - 360, v + 360]) if (Math.abs(c - prev) < Math.abs(v - prev) && Math.abs(c) <= 185) v = c;
        }
      }
      const busy = !force && (document.activeElement === f || this.dragAxis === i);
      if (!busy) f.value = v.toFixed(i < 3 ? 1 : 2);
      if (this.sliders && this.sliders[i] && (force || this.dragAxis !== i)) {
        this.sliders[i].value = String(v);
        this._setFill(i);
      }
    });
  }

  _status({ result, message }) {
    if (!this.statusEl) return;
    let cls = 'ok';
    let text = 'Ready';
    if (result) {
      if (result.converged) {
        text = 'On target';
      } else {
        cls = 'bad';
        text = `${(result.positionError * 1000).toFixed(1)} mm${this.cart.ikMode === 'full' ? ` / ${(result.orientationError * RAD2DEG).toFixed(1)}°` : ''} off`;
      }
    }
    if (message) {
      cls = result && result.converged ? 'ok' : 'bad';
      text = message;
    }
    this.statusEl.className = `ik-status ${cls}`;
    this.statusEl.textContent = text;
    this.statusEl.title = text;
  }
}
