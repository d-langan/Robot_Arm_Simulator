// Kinematic chain between two links of a parsed URDF model.
// Pure math on top of three.js Matrix4/Vector3 — independent of the scene graph,
// so IK can iterate without touching Object3D matrices.

import { Matrix4, Vector3, Quaternion, Euler } from 'three';
import { MOVABLE_TYPES, jointPath } from '../urdf/parser.js';

const _q = new Quaternion();
const _e = new Euler();
const _v = new Vector3();
const _one = new Vector3(1, 1, 1);

/** URDF <origin xyz rpy> -> Matrix4. rpy is fixed-axis roll-pitch-yaw: R = Rz(y) * Ry(p) * Rx(r). */
export function originMatrix(origin, out = new Matrix4()) {
  _e.set(origin.rpy[0], origin.rpy[1], origin.rpy[2], 'ZYX');
  _q.setFromEuler(_e);
  _v.set(origin.xyz[0], origin.xyz[1], origin.xyz[2]);
  return out.compose(_v, _q, _one);
}

/** Motion transform of a single joint at value v (radians or metres). */
export function jointMotionMatrix(type, axis, v, out = new Matrix4()) {
  if (type === 'revolute' || type === 'continuous') {
    return out.makeRotationAxis(axis, v);
  }
  if (type === 'prismatic') {
    return out.makeTranslation(axis.x * v, axis.y * v, axis.z * v);
  }
  return out.identity();
}

export class ChainError extends Error {}

export class KinematicChain {
  /**
   * @param {object} model parsed URDF model (parser.js)
   * @param {string} baseLink ancestor link (frame the chain is expressed in)
   * @param {string} tipLink end-effector link
   */
  constructor(model, baseLink, tipLink) {
    const path = jointPath(model, baseLink, tipLink);
    if (!path) throw new ChainError(`"${baseLink}" is not an ancestor of "${tipLink}"`);
    this.model = model;
    this.baseLink = baseLink;
    this.tipLink = tipLink;
    /** TCP offset relative to the tip link frame. */
    this.tcpOffset = new Matrix4();

    this.variables = [];          // controllable joint objects that move this chain
    const index = new Map();
    this.elements = path.map((name) => {
      const joint = model.joints.get(name);
      const el = {
        joint,
        type: joint.type,
        origin: originMatrix(joint.origin),
        axis: new Vector3(joint.axis[0], joint.axis[1], joint.axis[2]),
        varIndex: -1,
        coef: 0,
        offset: 0,
      };
      if (MOVABLE_TYPES.has(joint.type)) {
        let master = joint;
        let coef = 1;
        let offset = 0;
        if (joint.mimicResolved) {
          master = model.joints.get(joint.mimicResolved.master);
          coef = joint.mimicResolved.multiplier;
          offset = joint.mimicResolved.offset;
        }
        if (!index.has(master.name)) {
          index.set(master.name, this.variables.length);
          this.variables.push(master);
        }
        el.varIndex = index.get(master.name);
        el.coef = coef;
        el.offset = offset;
      }
      return el;
    });

    this.lower = Float64Array.from(this.variables, (j) => j.limit.lower);
    this.upper = Float64Array.from(this.variables, (j) => j.limit.upper);
    this.velocity = Float64Array.from(this.variables, (j) => j.limit.velocity);
    this.isAngular = this.variables.map((j) => j.type !== 'prismatic');

    // Scratch.
    this._frames = this.elements.map(() => new Matrix4());
    this._m = new Matrix4();
    this._axis = new Vector3();
    this._pj = new Vector3();
    this._pt = new Vector3();
    this._cross = new Vector3();
  }

  get dof() {
    return this.variables.length;
  }

  /** Read chain variables out of a name->value map (Map or plain object). */
  getQ(values, out = new Float64Array(this.dof)) {
    const get = values instanceof Map ? (k) => values.get(k) : (k) => values[k];
    for (let i = 0; i < this.dof; i++) {
      const v = get(this.variables[i].name);
      out[i] = Number.isFinite(v) ? v : 0;
    }
    return out;
  }

  /** Write chain variables into a plain object. */
  toValues(q, out = {}) {
    for (let i = 0; i < this.dof; i++) out[this.variables[i].name] = q[i];
    return out;
  }

  clamp(q) {
    for (let i = 0; i < q.length; i++) {
      if (q[i] < this.lower[i]) q[i] = this.lower[i];
      else if (q[i] > this.upper[i]) q[i] = this.upper[i];
    }
    return q;
  }

  _jointValue(el, q) {
    return el.varIndex < 0 ? 0 : el.coef * q[el.varIndex] + el.offset;
  }

  /**
   * Forward kinematics: TCP pose (tip * tcpOffset) in the base-link frame.
   * If `frames` is given, frames[i] receives the frame of joint i *before* its motion
   * (i.e. parent * origin), which carries the joint axis.
   */
  fk(q, out = new Matrix4(), frames = null) {
    out.identity();
    for (let i = 0; i < this.elements.length; i++) {
      const el = this.elements[i];
      out.multiply(el.origin);
      if (frames) frames[i].copy(out);
      if (el.varIndex >= 0) {
        out.multiply(jointMotionMatrix(el.type, el.axis, this._jointValue(el, q), this._m));
      }
    }
    return out.multiply(this.tcpOffset);
  }

  /** Tip link pose without the TCP offset. */
  fkTip(q, out = new Matrix4()) {
    this.fk(q, out);
    return out.multiply(this._m.copy(this.tcpOffset).invert());
  }

  /**
   * Geometric Jacobian at the TCP point, in the base frame.
   * J is row-major Float64Array(6 * dof): rows 0..2 linear, 3..5 angular.
   * Returns the TCP pose in `tcpOut`.
   */
  jacobian(q, J, tcpOut = new Matrix4()) {
    const n = this.dof;
    J.fill(0);
    this.fk(q, tcpOut, this._frames);
    this._pt.setFromMatrixPosition(tcpOut);
    for (let i = 0; i < this.elements.length; i++) {
      const el = this.elements[i];
      if (el.varIndex < 0) continue;
      const f = this._frames[i];
      const a = this._axis.copy(el.axis).transformDirection(f);
      const c = el.coef;
      const col = el.varIndex;
      if (el.type === 'prismatic') {
        J[0 * n + col] += c * a.x;
        J[1 * n + col] += c * a.y;
        J[2 * n + col] += c * a.z;
      } else {
        this._pj.setFromMatrixPosition(f);
        const v = this._cross.subVectors(this._pt, this._pj).crossVectors(a, this._cross);
        J[0 * n + col] += c * v.x;
        J[1 * n + col] += c * v.y;
        J[2 * n + col] += c * v.z;
        J[3 * n + col] += c * a.x;
        J[4 * n + col] += c * a.y;
        J[5 * n + col] += c * a.z;
      }
    }
    return tcpOut;
  }

  /** Rough reach of the chain (sum of joint offset lengths) — used to scale IK weights. */
  reach() {
    let s = 0;
    for (const el of this.elements) s += _v.setFromMatrixPosition(el.origin).length();
    s += _v.setFromMatrixPosition(this.tcpOffset).length();
    return Math.max(s, 1e-3);
  }
}
