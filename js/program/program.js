// Waypoint programs: storage format, motion planning (MoveJ / MoveL), playback.
//
// A point stores the full controllable joint vector (ground truth) plus the TCP
// pose it produced (for display/export). MoveL targets are recomputed from the
// stored joints with the *current* TCP definition, so changing the end-effector
// or TCP offset keeps programs consistent.

import { Matrix4, Vector3, Quaternion } from 'three';
import { solveIK, rotationError } from '../kinematics/ik.js';

export const PROGRAM_FORMAT = 'urdf-arm-sim-program';
export const PROGRAM_VERSION = 1;

export const MOVE_DEFAULTS = {
  J: { speed: 50, unit: '%' },     // percent of each joint's URDF velocity limit
  L: { speed: 250, unit: 'mm/s' }, // TCP linear speed
};
export const MOVEL_ANGULAR_SPEED = (90 * Math.PI) / 180; // rad/s TCP rotation speed for MoveL
const MOVEL_STEP_POS = 0.002;                 // m between IK samples
const MOVEL_STEP_ROT = (0.5 * Math.PI) / 180; // rad between IK samples
const MOVEL_MAX_SAMPLES = 4000;
const MOVEL_MAX_JUMP_ANGULAR = 0.2;  // rad per sample -> singularity / flip
const MOVEL_MAX_JUMP_LINEAR = 0.02;  // m per sample (prismatic)

/** Smoothstep timing: zero velocity at both ends, peak = 1.5 × average. */
export function smoothstep(u) {
  if (u <= 0) return 0;
  if (u >= 1) return 1;
  return u * u * (3 - 2 * u);
}
const PEAK = 1.5;

export function newPoint({ name, joints, pose, move = 'J', speed, dwell = 0 }) {
  return {
    name,
    move,
    speed: speed ?? MOVE_DEFAULTS[move].speed,
    dwell,
    joints: { ...joints },
    pose: pose ? clonePose(pose) : null,
  };
}

function clonePose(p) {
  return { frame: p.frame, tip: p.tip, position: [...p.position], quaternion: [...p.quaternion] };
}

export function serializeProgram(robotModel, points, extra = {}) {
  return {
    format: PROGRAM_FORMAT,
    version: PROGRAM_VERSION,
    robot: robotModel.name,
    units: { joints: 'rad|m', position: 'm', speedJ: '% of joint velocity limit', speedL: 'mm/s', dwell: 's' },
    ...extra,
    points: points.map((p) => ({
      name: p.name, move: p.move, speed: p.speed, dwell: p.dwell, joints: { ...p.joints }, pose: p.pose ? clonePose(p.pose) : null,
    })),
  };
}

/**
 * Validate a parsed program JSON against the loaded robot.
 * Returns { points, warnings } or throws with a readable message.
 */
export function parseProgram(json, robot) {
  if (!json || typeof json !== 'object') throw new Error('Not a program file');
  if (json.format !== PROGRAM_FORMAT) throw new Error('Not an arm simulator program (missing "format")');
  if (!Array.isArray(json.points)) throw new Error('Program has no "points" array');
  const warnings = [];
  if (json.robot && json.robot !== robot.name) warnings.push(`Program was made for "${json.robot}", loaded robot is "${robot.name}"`);
  const names = new Set(robot.controllable.map((j) => j.name));
  const points = json.points.map((p, i) => {
    const joints = {};
    let matched = 0;
    for (const [k, v] of Object.entries(p.joints || {})) {
      if (names.has(k) && Number.isFinite(v)) { joints[k] = v; matched++; }
    }
    if (matched === 0) throw new Error(`Point ${i + 1} has no joints matching this robot`);
    if (matched < names.size) warnings.push(`Point "${p.name || i + 1}" is missing ${names.size - matched} joint(s); they keep their current value`);
    const move = p.move === 'L' ? 'L' : 'J';
    return newPoint({
      name: String(p.name || `P${i + 1}`),
      move,
      speed: Number.isFinite(p.speed) && p.speed > 0 ? p.speed : MOVE_DEFAULTS[move].speed,
      dwell: Number.isFinite(p.dwell) && p.dwell >= 0 ? p.dwell : 0,
      joints,
      pose: p.pose && Array.isArray(p.pose.position) && Array.isArray(p.pose.quaternion) ? p.pose : null,
    });
  });
  return { points, warnings };
}

// ---------------------------------------------------------------------------
// Planning

export class PlanError extends Error {}

/**
 * Joint-space move. All joints start and stop together; the slowest joint
 * (relative to its velocity limit × speed%) sets the duration.
 */
export function planMoveJ(robot, fromValues, point) {
  const joints = robot.controllable;
  const q0 = joints.map((j) => fromValues[j.name] ?? 0);
  const q1 = joints.map((j, i) => {
    const v = point.joints[j.name];
    const t = Number.isFinite(v) ? v : q0[i];
    return Math.min(j.limit.upper, Math.max(j.limit.lower, t));
  });
  const frac = Math.min(100, Math.max(1, point.speed)) / 100;
  let T = 0;
  joints.forEach((j, i) => {
    const vmax = j.limit.velocity * frac;
    T = Math.max(T, (PEAK * Math.abs(q1[i] - q0[i])) / vmax);
  });
  return {
    type: 'J',
    duration: T,
    sample(t, out = {}) {
      const s = T > 0 ? smoothstep(t / T) : 1;
      joints.forEach((j, i) => { out[j.name] = q0[i] + (q1[i] - q0[i]) * s; });
      return out;
    },
    final: Object.fromEntries(joints.map((j, i) => [j.name, q1[i]])),
  };
}

/**
 * Straight-line TCP move (position lerp + orientation slerp) sampled every
 * ~2 mm / 0.5°, each sample solved by IK from the previous one. Fails if the
 * line leaves the workspace or passes through a singularity (joint jump).
 */
export function planMoveL(robot, chain, fromValues, point, ikMode = 'full') {
  const joints = robot.controllable;
  const qStart = chain.getQ(fromValues);
  const goalValues = { ...fromValues };
  for (const j of joints) if (Number.isFinite(point.joints[j.name])) goalValues[j.name] = point.joints[j.name];
  const qGoal = chain.getQ(goalValues);

  const T0 = chain.fk(qStart, new Matrix4());
  const T1 = chain.fk(qGoal, new Matrix4());
  const p0 = new Vector3(), p1 = new Vector3(), r0 = new Quaternion(), r1 = new Quaternion(), s = new Vector3();
  T0.decompose(p0, r0, s);
  T1.decompose(p1, r1, s);
  const dist = p0.distanceTo(p1);
  const angle = ikMode === 'full' ? rotationError(r1, r0).length() : 0;

  // Joints outside the chain (e.g. gripper fingers) interpolate linearly.
  const chainNames = new Set(chain.variables.map((v) => v.name));
  const others = joints.filter((j) => !chainNames.has(j.name));

  const N = Math.min(MOVEL_MAX_SAMPLES, Math.max(1, Math.ceil(Math.max(dist / MOVEL_STEP_POS, angle / MOVEL_STEP_ROT))));
  const samples = [Float64Array.from(qStart)];
  const target = { position: new Vector3(), quaternion: new Quaternion() };
  let prev = qStart;
  let maxRate = 0; // max over samples & joints of |dq| / vmax
  for (let k = 1; k <= N; k++) {
    const u = k / N;
    target.position.lerpVectors(p0, p1, u);
    target.quaternion.slerpQuaternions(r0, r1, u);
    const res = solveIK(chain, prev, target, { mode: ikMode, maxIterations: 60 });
    if (!res.converged) {
      throw new PlanError(`MoveL to "${point.name}" is not reachable along a straight line (fails at ${Math.round(u * 100)}%). Use MoveJ or add an intermediate point.`);
    }
    for (let i = 0; i < res.q.length; i++) {
      const d = Math.abs(res.q[i] - prev[i]);
      const lim = chain.isAngular[i] ? MOVEL_MAX_JUMP_ANGULAR : MOVEL_MAX_JUMP_LINEAR;
      if (d > lim) {
        throw new PlanError(`MoveL to "${point.name}" passes through a singularity near ${Math.round(u * 100)}% (${chain.variables[i].name} would jump ${(d * 180 / Math.PI).toFixed(0)}°). Use MoveJ or move the point.`);
      }
      maxRate = Math.max(maxRate, d / chain.velocity[i]);
    }
    samples.push(res.q);
    prev = res.q;
  }

  // Finish exactly on the taught joints when IK ended in the same configuration
  // (it only matches the pose to tolerance, which would otherwise drift taught points).
  const lastQ = samples[samples.length - 1];
  const qGoalClamped = chain.clamp(Float64Array.from(qGoal));
  let sameBranch = true;
  for (let i = 0; i < lastQ.length; i++) if (Math.abs(lastQ[i] - qGoalClamped[i]) > 0.05) sameBranch = false;
  if (sameBranch && samples.length > 1) samples[samples.length - 1] = qGoalClamped;

  const v = Math.max(1, point.speed) / 1000; // mm/s -> m/s
  let T = PEAK * Math.max(dist / v, angle / MOVEL_ANGULAR_SPEED);
  // Respect joint velocity limits (conservative: peak timing applied everywhere).
  T = Math.max(T, PEAK * N * maxRate);
  if (dist < 1e-7 && angle < 1e-7) T = 0;

  const otherStart = others.map((j) => fromValues[j.name] ?? 0);
  const otherGoal = others.map((j, i) => (Number.isFinite(point.joints[j.name]) ? point.joints[j.name] : otherStart[i]));
  for (const [i, j] of others.entries()) {
    const d = Math.abs(otherGoal[i] - otherStart[i]);
    T = Math.max(T, (PEAK * d) / j.limit.velocity);
  }

  const last = samples[samples.length - 1];
  const final = { ...fromValues };
  chain.toValues(last, final);
  others.forEach((j, i) => { final[j.name] = otherGoal[i]; });

  return {
    type: 'L',
    duration: T,
    samples,
    sample(t, out = {}) {
      const sv = T > 0 ? smoothstep(t / T) : 1;
      const x = sv * N;
      const k = Math.min(N - 1, Math.floor(x));
      const f = Math.min(1, x - k);
      const a = samples[k], b = samples[Math.min(N, k + 1)];
      for (let i = 0; i < chain.dof; i++) out[chain.variables[i].name] = a[i] + (b[i] - a[i]) * f;
      others.forEach((j, i) => { out[j.name] = otherStart[i] + (otherGoal[i] - otherStart[i]) * sv; });
      return out;
    },
    final,
  };
}

// ---------------------------------------------------------------------------
// Playback

/**
 * Runs a list of points. Drive it by calling update(dt) every frame.
 * Emits 'state' events: detail = { state, index, progress, message }.
 *   state: 'idle' | 'running' | 'paused' | 'dwell' | 'done' | 'error'
 */
export class ProgramRunner extends EventTarget {
  /**
   * @param {{ robot: () => Robot, chain: () => KinematicChain, ikMode: () => string,
   *           getValues: () => object, apply: (values: object) => void }} hooks
   */
  constructor(hooks) {
    super();
    this.hooks = hooks;
    this.state = 'idle';
    this.points = [];
    this.index = -1;
    this.segment = null;
    this.t = 0;
    this.dwellLeft = 0;
    this.loop = false;
    this.speedOverride = 1;
    this.single = false;
    this._buf = {};
  }

  get active() {
    return this.state === 'running' || this.state === 'dwell' || this.state === 'paused';
  }

  _emit(message = '') {
    const progress = this.segment && this.segment.duration > 0 ? Math.min(1, this.t / this.segment.duration) : (this.state === 'dwell' ? 1 : 0);
    this.dispatchEvent(new CustomEvent('state', { detail: { state: this.state, index: this.index, progress, message } }));
  }

  /** Run points[startIndex..] (or just one point when single = true). */
  start(points, { startIndex = 0, loop = false, single = false } = {}) {
    if (!points.length) return;
    this.points = points;
    this.loop = loop && !single;
    this.single = single;
    this.index = startIndex;
    this.segment = null;
    this.t = 0;
    this.dwellLeft = 0;
    this.state = 'running';
    this._emit();
  }

  pause() {
    if (this.state === 'running' || this.state === 'dwell') {
      this._pausedFrom = this.state;
      this.state = 'paused';
      this._emit();
    }
  }

  resume() {
    if (this.state === 'paused') {
      this.state = this._pausedFrom || 'running';
      this._emit();
    }
  }

  stop(message = '') {
    if (this.state === 'idle') return;
    this.state = 'idle';
    this.segment = null;
    this.index = -1;
    this._emit(message);
  }

  _fail(message) {
    this.state = 'error';
    this.segment = null;
    this._emit(message);
  }

  _plan() {
    const point = this.points[this.index];
    const robot = this.hooks.robot();
    const from = this.hooks.getValues();
    try {
      this.segment = point.move === 'L'
        ? planMoveL(robot, this.hooks.chain(), from, point, this.hooks.ikMode())
        : planMoveJ(robot, from, point);
    } catch (e) {
      this._fail(e.message || String(e));
      return false;
    }
    this.t = 0;
    return true;
  }

  _advance() {
    this.segment = null;
    if (this.single) {
      this.state = 'done';
      this._emit();
      return;
    }
    this.index++;
    if (this.index >= this.points.length) {
      if (this.loop) this.index = 0;
      else {
        this.state = 'done';
        this.index = this.points.length - 1;
        this._emit();
        return;
      }
    }
    this.state = 'running';
    this._emit();
  }

  /** Advance playback by dt seconds. Returns true when joints changed. */
  update(dt) {
    if (this.state === 'dwell') {
      this.dwellLeft -= dt * this.speedOverride;
      if (this.dwellLeft <= 0) this._advance();
      return false;
    }
    if (this.state !== 'running') return false;
    if (!this.segment && !this._plan()) return false;
    this.t += dt * this.speedOverride;
    const seg = this.segment;
    if (this.t >= seg.duration) {
      this.hooks.apply(seg.final);
      const dwell = this.points[this.index].dwell || 0;
      this.segment = null;
      if (dwell > 0) {
        this.state = 'dwell';
        this.dwellLeft = dwell;
        this._emit();
      } else {
        this._advance();
      }
      return true;
    }
    this.hooks.apply(seg.sample(this.t, this._buf));
    this._emitThrottled();
    return true;
  }

  _emitThrottled() {
    const now = performance.now();
    if (!this._lastEmit || now - this._lastEmit > 100) {
      this._lastEmit = now;
      this._emit();
    }
  }
}
