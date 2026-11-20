// Cartesian control: TCP target with a transform gizmo, numeric go-to, and
// jogging (step or continuous) in the reference or tool frame. All poses are
// computed in the robot root frame (== three.js world; the robot sits at the origin).

import {
  Object3D, Group, Matrix4, Vector3, Quaternion, Euler, AxesHelper, Mesh, SphereGeometry, MeshBasicMaterial,
} from 'three';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { KinematicChain, originMatrix } from './kinematics/chain.js';
import { solveIK, solveIKRobust, IK_DEFAULTS } from './kinematics/ik.js';
import { defaultTipLink } from './urdf/robot.js';

const _m = new Matrix4();
const _m2 = new Matrix4();
const _p = new Vector3();
const _q = new Quaternion();
const _s = new Vector3();

export class CartesianController extends EventTarget {
  /**
   * @param {Viewer} viewer
   * @param {{ getValues: () => object, apply: (values: object, source: string) => void }} hooks
   */
  constructor(viewer, hooks) {
    super();
    this.viewer = viewer;
    this.hooks = hooks;
    this.robot = null;
    this.chain = null;
    this.reference = null;
    this.ikMode = 'full';
    this.enabled = false;
    this.dragging = false;
    this.lastResult = null;
    this.jog = null;               // { axis, sign, frame } while continuous jogging
    this.jogSpeed = { linear: 0.1, angular: (20 * Math.PI) / 180 }; // m/s, rad/s
    this.tcpOffset = { xyz: [0, 0, 0], rpy: [0, 0, 0] };

    // TCP marker (actual pose) and target (gizmo).
    this.tcpMarker = new AxesHelper(0.12);
    this.tcpMarker.material.depthTest = false;
    this.tcpMarker.renderOrder = 12;
    this.tcpMarker.visible = false;
    this.target = new Object3D();
    this.target.name = 'ik-target';
    this.targetViz = new Group();
    const ball = new Mesh(new SphereGeometry(0.018, 20, 12), new MeshBasicMaterial({ color: 0x3fd7ff, transparent: true, opacity: 0.75, depthTest: false }));
    ball.renderOrder = 13;
    this.targetBall = ball;
    this.targetViz.add(ball);
    this.target.add(this.targetViz);
    this.target.visible = false;
    viewer.scene.add(this.tcpMarker, this.target);

    this.gizmo = new TransformControls(viewer.camera, viewer.renderer.domElement);
    this.gizmo.setSize(0.85);
    this.gizmo.setSpace('local');
    this.gizmo.attach(this.target);
    this.gizmoHelper = this.gizmo.getHelper();
    this.gizmoHelper.visible = false;
    this.gizmo.enabled = false;
    viewer.scene.add(this.gizmoHelper);

    this.gizmo.addEventListener('change', () => viewer.requestRender());
    this.gizmo.addEventListener('dragging-changed', (e) => {
      this.dragging = e.value;
      viewer.controls.enabled = !e.value;
      if (!e.value) {
        if (this.lastResult && !this.lastResult.converged) {
          this._emitStatus('Target out of reach — snapped back to the closest pose');
        }
        this.syncTarget();
      }
    });
    this.gizmo.addEventListener('objectChange', () => {
      if (!this.dragging) return;
      this.target.updateMatrixWorld();
      this._track(this.target.position, this.target.quaternion, 'gizmo');
    });

    this._stopFrame = viewer.onFrame((dt) => this._jogFrame(dt));
  }

  setRobot(robot) {
    this.robot = robot;
    this.jog = null;
    this.lastResult = null;
    if (!robot) {
      this.chain = null;
      return;
    }
    const tip = defaultTipLink(robot.model);
    this.reference = robot.model.root;
    this.tcpOffset = { xyz: [0, 0, 0], rpy: [0, 0, 0] }; // a tool offset belongs to the previous robot
    this.setTip(tip);
  }

  setTip(link) {
    this.chain = new KinematicChain(this.robot.model, this.robot.model.root, link);
    this._applyTcpOffset();
    this.ikMode = this.chain.dof >= 6 ? 'full' : 'position';
    this.syncTarget();
    this.dispatchEvent(new CustomEvent('chain'));
  }

  setReference(link) {
    this.reference = link;
    this.syncTarget();
    this.dispatchEvent(new CustomEvent('chain'));
  }

  setIkMode(mode) {
    this.ikMode = mode === 'position' ? 'position' : 'full';
    this.syncTarget();
  }

  setTcpOffset(xyz, rpy) {
    this.tcpOffset = { xyz: [...xyz], rpy: [...rpy] };
    this._applyTcpOffset();
    this.syncTarget();
    this.dispatchEvent(new CustomEvent('chain'));
  }

  _applyTcpOffset() {
    if (this.chain) originMatrix(this.tcpOffset, this.chain.tcpOffset);
  }

  setEnabled(on) {
    this.enabled = on;
    this.gizmo.enabled = on;
    this.gizmoHelper.visible = on;
    this.target.visible = on;
    if (!on) this.jog = null;
    this.syncTarget();
  }

  setGizmoMode(mode) {
    this.gizmo.setMode(mode);
    this.viewer.requestRender();
  }

  setGizmoSpace(space) {
    this.gizmo.setSpace(space);
    this.viewer.requestRender();
  }

  setTcpVisible(on) {
    this.tcpVisible = on;
    this.tcpMarker.visible = on;
    this.viewer.requestRender();
  }

  /** Current TCP pose in the root/world frame. */
  tcpMatrix(out = new Matrix4(), values = this.hooks.getValues()) {
    if (!this.chain) return out.identity();
    return this.chain.fk(this.chain.getQ(values), out);
  }

  referenceMatrix(out = new Matrix4()) {
    if (!this.robot || !this.reference) return out.identity();
    return this.robot.linkWorldMatrix(this.reference, out);
  }

  /** TCP pose relative to the reference frame: { position (m), quaternion, rpy (rad) }. */
  tcpInReference(values) {
    const T = this.tcpMatrix(new Matrix4(), values);
    const R = this.referenceMatrix(new Matrix4()).invert();
    const L = R.multiply(T);
    const position = new Vector3();
    const quaternion = new Quaternion();
    L.decompose(position, quaternion, _s);
    const e = new Euler().setFromQuaternion(quaternion, 'ZYX');
    return { position, quaternion, rpy: [e.x, e.y, e.z] };
  }

  /** Move the gizmo/target and TCP marker onto the actual TCP. */
  syncTarget() {
    if (!this.chain) return;
    const T = this.tcpMatrix(_m);
    T.decompose(_p, _q, _s);
    this.tcpMarker.position.copy(_p);
    this.tcpMarker.quaternion.copy(_q);
    if (!this.dragging) {
      this.target.position.copy(_p);
      this.target.quaternion.copy(_q);
      this.targetBall.material.color.set(0x3fd7ff);
    }
    const reach = this.chain.reach();
    this.tcpMarker.scale.setScalar(Math.max(0.4, Math.min(2.5, reach / 1.2)));
    this.viewer.requestRender();
  }

  _emitStatus(message = '') {
    this.dispatchEvent(new CustomEvent('status', { detail: { result: this.lastResult, message, dof: this.chain ? this.chain.dof : 0 } }));
  }

  /** Interactive tracking: local IK from the current configuration, best effort. */
  _track(position, quaternion, source) {
    const values = this.hooks.getValues();
    const q0 = this.chain.getQ(values);
    const res = solveIK(this.chain, q0, { position, quaternion }, { mode: this.ikMode, maxIterations: 40 });
    this.lastResult = res;
    this.targetBall.material.color.set(res.converged ? 0x3fd7ff : 0xff4d4d);
    this.hooks.apply(this.chain.toValues(res.q, {}), source);
    this._emitStatus(res.converged ? '' : 'Out of reach');
    return res;
  }

  /**
   * Go to a pose given in the reference frame (position m, rpy rad). Uses restarts.
   * Returns the IK result; joints are applied only when it converged.
   */
  _referenceToWorld(position, rpy) {
    const local = new Matrix4().compose(
      new Vector3(...position),
      new Quaternion().setFromEuler(new Euler(rpy[0], rpy[1], rpy[2], 'ZYX')),
      new Vector3(1, 1, 1),
    );
    const world = this.referenceMatrix(new Matrix4()).multiply(local);
    const p = new Vector3();
    const q = new Quaternion();
    world.decompose(p, q, new Vector3());
    return { p, q };
  }

  /** Live tracking of a reference-frame pose (axis sliders): local IK, best effort, like the gizmo. */
  trackReferencePose(position, rpy) {
    if (!this.chain) return null;
    const { p, q } = this._referenceToWorld(position, rpy);
    return this._track(p, q, 'cart-slider');
  }

  goToReferencePose(position, rpy) {
    const { p, q } = this._referenceToWorld(position, rpy);
    const values = this.hooks.getValues();
    const res = solveIKRobust(this.chain, this.chain.getQ(values), { position: p, quaternion: q }, { mode: this.ikMode });
    this.lastResult = res;
    if (res.converged) {
      this.hooks.apply(this.chain.toValues(res.q, {}), 'cart-input');
      this._emitStatus('');
    } else {
      this._emitStatus(`No IK solution: ${(res.positionError * 1000).toFixed(1)} mm / ${(res.orientationError * 180 / Math.PI).toFixed(1)}° from target`);
    }
    this.syncTarget();
    return res;
  }

  /** Pose after applying a jog delta to the current TCP. axis: 0..2 = X,Y,Z translation; 3..5 = rotation. */
  _jogged(base, axis, amount, frame) {
    base.decompose(_p, _q, _s);
    const pos = _p.clone();
    const rot = _q.clone();
    const dir = new Vector3(axis % 3 === 0 ? 1 : 0, axis % 3 === 1 ? 1 : 0, axis % 3 === 2 ? 1 : 0);
    if (frame === 'tool') {
      if (axis < 3) pos.add(dir.applyQuaternion(rot).multiplyScalar(amount));
      else rot.multiply(new Quaternion().setFromAxisAngle(dir, amount));
    } else {
      const refQ = new Quaternion();
      this.referenceMatrix(_m2).decompose(new Vector3(), refQ, new Vector3());
      dir.applyQuaternion(refQ);
      if (axis < 3) pos.addScaledVector(dir, amount);
      else rot.premultiply(new Quaternion().setFromAxisAngle(dir, amount));
    }
    return { position: pos, quaternion: rot };
  }

  /**
   * IK for a jog target: iterate to tight tolerances (so small steps are not swallowed
   * by the solver tolerance and residuals do not accumulate) but judge success with the
   * normal ones. Rotation jogs always need orientation, even when the chain is in
   * position-only mode — otherwise they would "succeed" without moving.
   */
  _solveJog(axis, target, q0, maxIterations) {
    const mode = axis >= 3 ? 'full' : this.ikMode;
    const res = solveIK(this.chain, q0, target, { mode, maxIterations, positionTolerance: 1e-7, orientationTolerance: 1e-6 });
    res.converged = res.positionError < IK_DEFAULTS.positionTolerance
      && (mode !== 'full' || res.orientationError < IK_DEFAULTS.orientationTolerance);
    return res;
  }

  _blockedMessage(axis) {
    return axis >= 3 && this.ikMode === 'position'
      ? 'Rotation not reachable with this chain'
      : 'Jog blocked: joint limit, singularity or out of reach';
  }

  /** One jog increment. Returns true if the robot moved. */
  jogStep(axis, sign, frame, linearStep, angularStep) {
    if (!this.chain) return false;
    const amount = sign * (axis < 3 ? linearStep : angularStep);
    const target = this._jogged(this.tcpMatrix(new Matrix4()), axis, amount, frame);
    const values = this.hooks.getValues();
    const res = this._solveJog(axis, target, this.chain.getQ(values), 100);
    this.lastResult = res;
    if (res.converged) {
      this.hooks.apply(this.chain.toValues(res.q, {}), 'jog');
      this._emitStatus('');
      this.syncTarget();
      return true;
    }
    this._emitStatus(this._blockedMessage(axis));
    return false;
  }

  startJog(axis, sign, frame) {
    if (!this.chain) return;
    const T = this.tcpMatrix(new Matrix4());
    const p = new Vector3();
    const q = new Quaternion();
    T.decompose(p, q, new Vector3());
    this.jog = { axis, sign, frame, target: { position: p, quaternion: q } };
  }

  stopJog() {
    this.jog = null;
    this.syncTarget();
  }

  _jogFrame(dt) {
    if (!this.jog || !this.chain) return false;
    const { axis, sign, frame, target } = this.jog;
    const rate = axis < 3 ? this.jogSpeed.linear : this.jogSpeed.angular;
    const base = new Matrix4().compose(target.position, target.quaternion, new Vector3(1, 1, 1));
    const q0 = this.chain.getQ(this.hooks.getValues());
    // Joint speed this step would need, relative to each joint's URDF velocity limit.
    const ratioOf = (q) => {
      let r = 0;
      for (let i = 0; i < q.length; i++) r = Math.max(r, Math.abs(q[i] - q0[i]) / (Math.max(dt, 1e-3) * this.chain.velocity[i]));
      return r;
    };
    let step = sign * rate * dt;
    let next = this._jogged(base, axis, step, frame);
    let res = this._solveJog(axis, next, q0, 40);
    let slowed = false;
    if (res.converged) {
      const ratio = ratioOf(res.q);
      if (ratio > 1) {
        // Near a singularity: shrink the Cartesian increment so joints stay within their limits.
        step /= ratio * 1.05;
        next = this._jogged(base, axis, step, frame);
        res = this._solveJog(axis, next, q0, 40);
        slowed = true;
        if (res.converged && ratioOf(res.q) > 1.1) res.converged = false;
      }
    }
    this.lastResult = res;
    if (!res.converged) {
      this._emitStatus(slowed ? 'Jog blocked near a singularity' : this._blockedMessage(axis));
      return false; // keep the last good target; robot stays put
    }
    this.jog.target = next;
    this.hooks.apply(this.chain.toValues(res.q, {}), 'jog');
    if (slowed !== this._jogSlowed) {
      this._jogSlowed = slowed;
      this._emitStatus(slowed ? 'Slowed near a singularity' : '');
    }
    return true;
  }

  dispose() {
    this._stopFrame();
    this.gizmo.detach();
    this.gizmo.dispose();
  }
}
