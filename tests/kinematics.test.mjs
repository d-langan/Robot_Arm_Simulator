// Kinematics checks against an independent model: the published UR10 DH table.
// ROS `tool0` is the UR controller's tool flange frame, and `base_link` is the
// controller's `base` frame rotated by π about Z, so FK(world -> tool0) must equal
// Rz(π) · DH(q) for every q.

import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { DOMParser } from '@xmldom/xmldom';
import { Matrix4, Vector3, Quaternion } from 'three';
import { parseURDF, controllableJoints } from '../js/urdf/parser.js';
import { KinematicChain } from '../js/kinematics/chain.js';
import { solveIK, solveIKRobust, rotationError } from '../js/kinematics/ik.js';

const urdf = readFileSync(new URL('../robots/ur10/ur10.urdf', import.meta.url), 'utf8');
const model = parseURDF(urdf, { DOMParser });

// UR10 (CB3) DH parameters, Universal Robots support article "DH parameters for calculations of kinematics".
const DH = {
  a: [0, -0.612, -0.5723, 0, 0, 0],
  d: [0.1273, 0, 0, 0.163941, 0.1157, 0.0922],
  alpha: [Math.PI / 2, 0, 0, Math.PI / 2, -Math.PI / 2, 0],
};

function dhFK(q) {
  const T = new Matrix4();
  const A = new Matrix4();
  for (let i = 0; i < 6; i++) {
    const ct = Math.cos(q[i]), st = Math.sin(q[i]);
    const ca = Math.cos(DH.alpha[i]), sa = Math.sin(DH.alpha[i]);
    A.set(
      ct, -st * ca, st * sa, DH.a[i] * ct,
      st, ct * ca, -ct * sa, DH.a[i] * st,
      0, sa, ca, DH.d[i],
      0, 0, 0, 1,
    );
    T.multiply(A);
  }
  return new Matrix4().makeRotationZ(Math.PI).multiply(T);
}

function rand(seed) {
  let s = seed;
  return () => ((s = (s * 16807) % 2147483647) / 2147483647);
}

function maxAbsDiff(a, b) {
  let m = 0;
  for (let i = 0; i < 16; i++) m = Math.max(m, Math.abs(a.elements[i] - b.elements[i]));
  return m;
}

test('UR10 parses with 6 controllable joints and the documented limits', () => {
  const joints = controllableJoints(model);
  assert.deepEqual(joints.map((j) => j.name), [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
  ]);
  assert.equal(model.root, 'world');
  const elbow = model.joints.get('elbow_joint');
  assert.ok(Math.abs(elbow.limit.upper - Math.PI) < 1e-9);
  assert.ok(Math.abs(model.joints.get('shoulder_pan_joint').limit.lower + 2 * Math.PI) < 1e-9);
  assert.ok(elbow.limit.specified);
});

test('FK(world -> tool0) matches Rz(pi) * DH(q) for random configurations', () => {
  const chain = new KinematicChain(model, 'world', 'tool0');
  assert.equal(chain.dof, 6);
  const r = rand(7);
  for (let k = 0; k < 200; k++) {
    const q = Array.from({ length: 6 }, () => (r() * 2 - 1) * Math.PI);
    const a = chain.fk(q);
    const b = dhFK(q);
    // DH kinematics yaml carries ~1e-11 m residuals and 1e-9 rad rounding (1.570796327).
    assert.ok(maxAbsDiff(a, b) < 1e-6, `config ${k}: diff ${maxAbsDiff(a, b)}`);
  }
});

test('Jacobian matches finite differences', () => {
  const chain = new KinematicChain(model, 'world', 'tool0');
  const n = chain.dof;
  const J = new Float64Array(6 * n);
  const q = [0.3, -1.1, 1.4, -0.7, 1.2, 0.4];
  const T0 = chain.jacobian(q, J);
  const p0 = new Vector3().setFromMatrixPosition(T0);
  const q0 = new Quaternion().setFromRotationMatrix(T0);
  const h = 1e-6;
  for (let c = 0; c < n; c++) {
    const qh = q.slice();
    qh[c] += h;
    const T1 = chain.fk(qh);
    const p1 = new Vector3().setFromMatrixPosition(T1);
    const q1 = new Quaternion().setFromRotationMatrix(T1);
    const dp = p1.sub(p0).divideScalar(h);
    const dr = rotationError(q1, q0).divideScalar(h);
    const num = [dp.x, dp.y, dp.z, dr.x, dr.y, dr.z];
    for (let r = 0; r < 6; r++) {
      assert.ok(Math.abs(num[r] - J[r * n + c]) < 1e-4, `J[${r},${c}] analytic ${J[r * n + c]} vs numeric ${num[r]}`);
    }
  }
});

test('IK recovers reachable poses from nearby seeds (local) and from far seeds (robust)', () => {
  const chain = new KinematicChain(model, 'world', 'tool0');
  const r = rand(11);
  let localOk = 0;
  let robustOk = 0;
  const N = 60;
  for (let k = 0; k < N; k++) {
    const qGoal = [ (r() * 2 - 1) * 3, -r() * 3, (r() * 2 - 1) * 2.8, (r() * 2 - 1) * 3, (r() * 2 - 1) * 3, (r() * 2 - 1) * 3 ];
    const T = chain.fk(qGoal);
    const target = { position: new Vector3().setFromMatrixPosition(T), quaternion: new Quaternion().setFromRotationMatrix(T) };
    const seedNear = qGoal.map((v) => v + (r() - 0.5) * 0.3);
    const a = solveIK(chain, seedNear, target);
    if (a.converged) localOk++;
    const seedFar = [0, -Math.PI / 2, 0, -Math.PI / 2, 0, 0];
    const b = solveIKRobust(chain, seedFar, target, { seed: k + 1 });
    if (b.converged) robustOk++;
    if (b.converged) {
      const Tb = chain.fk(b.q);
      const pe = new Vector3().setFromMatrixPosition(Tb).distanceTo(target.position);
      assert.ok(pe < 2e-4, `robust solution position error ${pe}`);
    }
  }
  assert.ok(localOk >= N * 0.95, `local IK converged ${localOk}/${N}`);
  assert.ok(robustOk >= N * 0.9, `robust IK converged ${robustOk}/${N}`);
});

test('IK respects joint limits', () => {
  const chain = new KinematicChain(model, 'world', 'tool0');
  const target = { position: new Vector3(0.4, 0.3, 0.6), quaternion: new Quaternion() };
  const res = solveIKRobust(chain, [0, -1.5, 1.5, -1.5, -1.5, 0], target);
  for (let i = 0; i < chain.dof; i++) {
    assert.ok(res.q[i] >= chain.lower[i] - 1e-12 && res.q[i] <= chain.upper[i] + 1e-12);
  }
});

test('Unreachable targets report non-convergence instead of throwing', () => {
  const chain = new KinematicChain(model, 'world', 'tool0');
  const target = { position: new Vector3(5, 0, 0), quaternion: new Quaternion() };
  const res = solveIK(chain, [0, -1.5, 1.5, -1.5, -1.5, 0], target);
  assert.equal(res.converged, false);
  assert.ok(res.positionError > 3);
});
