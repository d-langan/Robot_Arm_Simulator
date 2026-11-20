// Joint-space control: one slider + numeric field per controllable joint.

import { h, toDisplay, fromDisplay, jointUnit, RAD2DEG } from './dom.js';

export class JointPanel {
  /**
   * @param {HTMLElement} root
   * @param {{ setJoints: (values: object, source: string) => void, getValues: () => object }} app
   */
  constructor(root, app) {
    this.root = root;
    this.app = app;
    this.rows = new Map();
  }

  setRobot(robot) {
    this.robot = robot;
    this.rows.clear();
    this.root.replaceChildren();
    if (!robot) return;

    const joints = robot.controllable;
    const header = h('div.panel-head',
      h('h2', 'Joints'),
      h('div.btn-row',
        h('button.btn.btn-sm', { type: 'button', title: 'All joints to 0 (or the nearest limit)', on: { click: () => this._zero() } }, 'Zero'),
        h('button.btn.btn-sm', { type: 'button', title: 'Random pose within limits', on: { click: () => this._random() } }, 'Random'),
      ),
    );
    this.root.append(header);

    if (!joints.length) {
      this.root.append(h('p.muted', 'This robot has no movable joints.'));
      return;
    }

    const list = h('div.joint-list');
    for (const j of joints) {
      const unit = jointUnit(j);
      const lo = toDisplay(j, j.limit.lower);
      const hi = toDisplay(j, j.limit.upper);
      const step = j.type === 'prismatic' ? 0.1 : 0.1;
      const slider = h('input.slider', {
        type: 'range', min: lo, max: hi, step: (hi - lo) / 2000,
        'aria-label': `${j.name} slider`,
      });
      const num = h('input.num', {
        type: 'number', min: lo, max: hi, step,
        'aria-label': `${j.name} value in ${unit === '°' ? 'degrees' : 'millimetres'}`,
      });
      const flags = [];
      if (j.type === 'continuous') flags.push(h('span.badge', { title: 'Continuous joint: no hard limits in the URDF; range shown as ±360°' }, '∞'));
      if ((j.type === 'revolute' || j.type === 'prismatic') && !j.limit.specified) {
        flags.push(h('span.badge.badge-warn', { title: 'URDF gives no usable limits for this joint; a default range is used' }, 'no limits'));
      }
      if (j.type === 'prismatic') flags.push(h('span.badge', { title: 'Prismatic (linear) joint' }, 'linear'));
      const followers = (robot.followers.get(j.name) || []).map((f) => f.name);
      const row = h('div.joint-row', { dataset: { joint: j.name } },
        h('div.joint-top',
          h('label.joint-name', { title: `${j.name} (${j.type})${followers.length ? `\nDrives mimic joints: ${followers.join(', ')}` : ''}` }, j.name),
          ...flags,
          h('span.spacer'),
          num,
          h('span.unit', unit),
        ),
        slider,
        h('div.joint-limits',
          h('span', `${lo.toFixed(j.type === 'prismatic' ? 0 : 0)}${unit}`),
          h('span.vel', { title: 'Velocity limit (used for MoveJ timing)' },
            j.type === 'prismatic' ? `${(j.limit.velocity * 1000).toFixed(0)} mm/s` : `${(j.limit.velocity * RAD2DEG).toFixed(0)}°/s`),
          h('span', `${hi.toFixed(0)}${unit}`),
        ),
      );
      const fmt = (x) => toDisplay(j, x).toFixed(j.type === 'prismatic' ? 1 : 2);
      const send = (display, fromField = false) => {
        // An emptied field means "not entered yet", not 0.
        if (String(display).trim() === '') {
          if (fromField) num.value = fmt(this.robot.getJointValue(j.name));
          return;
        }
        const v = fromDisplay(j, Number(display));
        if (!Number.isFinite(v)) return;
        this.app.setJoints({ [j.name]: v }, 'joint-panel');
        // Show the applied (clamped) value even though the field still has focus.
        if (fromField) num.value = fmt(this.robot.getJointValue(j.name));
      };
      slider.addEventListener('input', () => send(slider.value));
      num.addEventListener('change', () => send(num.value, true));
      num.addEventListener('keydown', (e) => { if (e.key === 'Enter') send(num.value, true); });
      this.rows.set(j.name, { joint: j, slider, num, row, lo, hi });
      list.append(row);
    }
    this.root.append(list);
    this.refresh(this.app.getValues());
  }

  /** Update displayed values. `source` avoids rewriting the field the user is typing in. */
  refresh(values, source = '') {
    for (const [name, r] of this.rows) {
      const v = values[name];
      if (!Number.isFinite(v)) continue;
      const d = toDisplay(r.joint, v);
      // Never overwrite a field the user is typing in (a running program refreshes every frame).
      if (document.activeElement !== r.num) {
        r.num.value = d.toFixed(r.joint.type === 'prismatic' ? 1 : 2);
      }
      if (!(source === 'joint-panel' && document.activeElement === r.slider)) r.slider.value = String(d);
      // Near-limit highlight (within 2 % of the range).
      const span = r.hi - r.lo;
      const near = span > 0 && (d - r.lo < span * 0.02 || r.hi - d < span * 0.02);
      r.row.classList.toggle('at-limit', near);
      const pct = span > 0 ? ((d - r.lo) / span) * 100 : 50;
      r.slider.style.setProperty('--pct', `${pct}%`);
    }
  }

  _zero() {
    const out = {};
    for (const [name, r] of this.rows) out[name] = Math.min(r.joint.limit.upper, Math.max(r.joint.limit.lower, 0));
    this.app.setJoints(out, 'joint-panel-btn');
  }

  _random() {
    const out = {};
    for (const [name, r] of this.rows) {
      const { lower, upper } = r.joint.limit;
      // Keep wide-range joints within ±180° so random poses stay readable.
      const lo = r.joint.type === 'prismatic' ? lower : Math.max(lower, -Math.PI);
      const hi = r.joint.type === 'prismatic' ? upper : Math.min(upper, Math.PI);
      out[name] = lo + Math.random() * (hi - lo);
    }
    this.app.setJoints(out, 'joint-panel-btn');
  }
}
