// Numerical inverse kinematics for an arbitrary KinematicChain.
//
// Damped least squares (Levenberg–Marquardt step acceptance):
//   dq = Jᵀ (J Jᵀ + λ² I)⁻¹ e
// with per-iteration accept/reject on the true error, an adaptive λ, and an
// active-set treatment of joint limits (joints pinned at a limit and pushed
// outward are removed from J for that step). Works for any DOF: redundant
// chains get the minimum-norm step, under-actuated ones a least-squares fit.

import { Matrix4, Vector3, Quaternion } from 'three';

const TWO_PI = 2 * Math.PI;

export const IK_DEFAULTS = {
  mode: 'full',              // 'full' (position + orientation) | 'position'
  maxIterations: 120,
  positionTolerance: 1e-4,   // m   (0.1 mm)
  orientationTolerance: 1e-3, // rad (~0.06°)
  orientationWeight: null,   // m per rad; null -> derived from chain reach
  lambda: 0.05,
  lambdaMin: 1e-4,
  lambdaMax: 1e3,
  maxStep: 0.3,              // max |dq_i| per iteration (rad or m)
};

/** Rotation error vector (axis * angle) that takes `current` to `target`, in the common frame. */
export function rotationError(target, current, out = new Vector3()) {
  // qe = qt * qc⁻¹
  const qx = target.x, qy = target.y, qz = target.z, qw = target.w;
  const cx = -current.x, cy = -current.y, cz = -current.z, cw = current.w;
  let x = qw * cx + qx * cw + qy * cz - qz * cy;
  let y = qw * cy - qx * cz + qy * cw + qz * cx;
  let z = qw * cz + qx * cy - qy * cx + qz * cw;
  let w = qw * cw - qx * cx - qy * cy - qz * cz;
  if (w < 0) { x = -x; y = -y; z = -z; w = -w; }
  const s = Math.hypot(x, y, z);
  if (s < 1e-12) return out.set(2 * x, 2 * y, 2 * z);
  const angle = 2 * Math.atan2(s, w);
  const k = angle / s;
  return out.set(x * k, y * k, z * k);
}

/** Solve A x = b for small dense m×m A (row-major), Gaussian elimination with partial pivoting. In-place on copies. */
function solveDense(A, b, m, x) {
  // A and b are scratch copies and get destroyed.
  for (let c = 0; c < m; c++) {
    let p = c;
    let best = Math.abs(A[c * m + c]);
    for (let r = c + 1; r < m; r++) {
      const v = Math.abs(A[r * m + c]);
      if (v > best) { best = v; p = r; }
    }
    if (best < 1e-300) return false;
    if (p !== c) {
      for (let k = 0; k < m; k++) {
        const t = A[c * m + k]; A[c * m + k] = A[p * m + k]; A[p * m + k] = t;
      }
      const t = b[c]; b[c] = b[p]; b[p] = t;
    }
    const piv = A[c * m + c];
    for (let r = c + 1; r < m; r++) {
      const f = A[r * m + c] / piv;
      if (f === 0) continue;
      for (let k = c; k < m; k++) A[r * m + k] -= f * A[c * m + k];
      b[r] -= f * b[c];
    }
  }
  for (let r = m - 1; r >= 0; r--) {
    let s = b[r];
    for (let k = r + 1; k < m; k++) s -= A[r * m + k] * x[k];
    x[r] = s / A[r * m + r];
  }
  return true;
}

/** Scratch buffers cached per chain so interactive IK allocates nothing per frame. */
function scratch(chain) {
  if (chain._ikScratch && chain._ikScratch.n === chain.dof) return chain._ikScratch;
  const n = chain.dof;
  chain._ikScratch = {
    n,
    J: new Float64Array(6 * n),
    Jw: new Float64Array(6 * n),
    A: new Float64Array(36),
    e: new Float64Array(6),
    y: new Float64Array(6),
    dq: new Float64Array(n),
    qTry: new Float64Array(n),
    locked: new Uint8Array(n),
    T: new Matrix4(),
    pos: new Vector3(),
    quat: new Quaternion(),
    scl: new Vector3(),
    rot: new Vector3(),
  };
  return chain._ikScratch;
}

function poseError(chain, q, target, opts, S, w) {
  chain.fk(q, S.T);
  S.T.decompose(S.pos, S.quat, S.scl);
  const ex = target.position.x - S.pos.x;
  const ey = target.position.y - S.pos.y;
  const ez = target.position.z - S.pos.z;
  const posErr = Math.hypot(ex, ey, ez);
  let rotErr = 0;
  S.e[0] = ex; S.e[1] = ey; S.e[2] = ez;
  if (opts.mode === 'full') {
    rotationError(target.quaternion, S.quat, S.rot);
    rotErr = S.rot.length();
    S.e[3] = w * S.rot.x; S.e[4] = w * S.rot.y; S.e[5] = w * S.rot.z;
  }
  const cost = opts.mode === 'full'
    ? ex * ex + ey * ey + ez * ez + S.e[3] * S.e[3] + S.e[4] * S.e[4] + S.e[5] * S.e[5]
    : ex * ex + ey * ey + ez * ez;
  return { posErr, rotErr, cost };
}

/**
 * Local IK solve starting from q0.
 * @param {KinematicChain} chain
 * @param {ArrayLike<number>} q0 initial chain variables
 * @param {{position: Vector3, quaternion: Quaternion}} target TCP pose in the chain base frame
 * @param {object} [options] see IK_DEFAULTS
 * @returns {{q: Float64Array, converged: boolean, positionError: number, orientationError: number, iterations: number}}
 */
export function solveIK(chain, q0, target, options = {}) {
  const opts = { ...IK_DEFAULTS, ...options };
  const n = chain.dof;
  const q = chain.clamp(Float64Array.from(q0));
  if (n === 0) {
    const S0 = scratch(chain);
    const r = poseError(chain, q, target, opts, S0, 1);
    return { q, converged: r.posErr < opts.positionTolerance && (opts.mode !== 'full' || r.rotErr < opts.orientationTolerance), positionError: r.posErr, orientationError: r.rotErr, iterations: 0 };
  }
  const S = scratch(chain);
  const w = opts.orientationWeight ?? Math.min(1, Math.max(0.05, 0.25 * chain.reach()));
  const m = opts.mode === 'full' ? 6 : 3;
  const { J, Jw, A, e, y, dq, qTry, locked } = S;
  const lo = chain.lower, hi = chain.upper;

  let cur = poseError(chain, q, target, opts, S, w);
  const eCur = new Float64Array(6);
  eCur.set(S.e);
  let lambda = opts.lambda;
  let it = 0;
  const done = (r) => r.posErr < opts.positionTolerance && (opts.mode !== 'full' || r.rotErr < opts.orientationTolerance);

  for (; it < opts.maxIterations && !done(cur); it++) {
    chain.jacobian(q, J);
    locked.fill(0);
    let stepOk = false;

    for (let pass = 0; pass < 3; pass++) {
      // Weighted, limit-masked Jacobian (rows 3..5 scaled by w).
      for (let r = 0; r < m; r++) {
        const rw = r >= 3 ? w : 1;
        for (let c = 0; c < n; c++) Jw[r * n + c] = locked[c] ? 0 : rw * J[r * n + c];
      }
      // A = Jw Jwᵀ + λ² I
      const l2 = lambda * lambda;
      for (let r = 0; r < m; r++) {
        for (let c = r; c < m; c++) {
          let s = 0;
          for (let k = 0; k < n; k++) s += Jw[r * n + k] * Jw[c * n + k];
          A[r * m + c] = s;
          A[c * m + r] = s;
        }
        A[r * m + r] += l2;
      }
      for (let r = 0; r < m; r++) e[r] = eCur[r];
      if (!solveDense(A, e, m, y)) break;
      // dq = Jwᵀ y
      let maxAbs = 0;
      for (let c = 0; c < n; c++) {
        let s = 0;
        for (let r = 0; r < m; r++) s += Jw[r * n + c] * y[r];
        dq[c] = s;
        if (Math.abs(s) > maxAbs) maxAbs = Math.abs(s);
      }
      if (maxAbs > opts.maxStep) {
        const k = opts.maxStep / maxAbs;
        for (let c = 0; c < n; c++) dq[c] *= k;
      }
      // Active set: lock joints sitting on a limit and pushed further into it.
      let newlyLocked = false;
      for (let c = 0; c < n; c++) {
        if (locked[c]) continue;
        if ((q[c] <= lo[c] + 1e-12 && dq[c] < 0) || (q[c] >= hi[c] - 1e-12 && dq[c] > 0)) {
          locked[c] = 1;
          newlyLocked = true;
        }
      }
      stepOk = true;
      if (!newlyLocked) break;
    }
    if (!stepOk) break;

    for (let c = 0; c < n; c++) {
      let v = q[c] + (locked[c] ? 0 : dq[c]);
      if (v < lo[c]) v = lo[c];
      else if (v > hi[c]) v = hi[c];
      qTry[c] = v;
    }
    const trial = poseError(chain, qTry, target, opts, S, w);
    if (trial.cost < cur.cost) {
      q.set(qTry);
      cur = trial;
      eCur.set(S.e);
      lambda = Math.max(opts.lambdaMin, lambda * 0.5);
    } else {
      lambda *= 4;
      if (lambda > opts.lambdaMax) break; // stuck in a local minimum / at limits
    }
  }

  return {
    q,
    converged: done(cur),
    positionError: cur.posErr,
    orientationError: cur.rotErr,
    iterations: it,
  };
}

/** Deterministic PRNG (mulberry32) so restarts are reproducible. */
function rng(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/**
 * For angular joints whose range spans more than 2π, pick the 2π-equivalent of
 * each value closest to the reference configuration (avoids needless unwinding).
 */
export function unwrapToward(chain, q, ref) {
  for (let i = 0; i < q.length; i++) {
    if (!chain.isAngular[i] || (chain.periodic && !chain.periodic[i])) continue;
    let best = q[i];
    let bestD = Math.abs(q[i] - ref[i]);
    for (let k = -2; k <= 2; k++) {
      if (k === 0) continue;
      const v = q[i] + k * TWO_PI;
      if (v < chain.lower[i] || v > chain.upper[i]) continue;
      const d = Math.abs(v - ref[i]);
      if (d < bestD - 1e-9) { best = v; bestD = d; }
    }
    q[i] = best;
  }
  return q;
}

function jointDistance(chain, a, b) {
  let s = 0;
  for (let i = 0; i < a.length; i++) {
    const d = (a[i] - b[i]) / (chain.isAngular[i] ? 1 : 0.25);
    s += d * d;
  }
  return Math.sqrt(s);
}

/**
 * Global-ish IK: local solve from q0, then random restarts within limits.
 * Returns the converged solution closest to q0 (or the best residual if none converged).
 */
export function solveIKRobust(chain, q0, target, options = {}) {
  const { restarts = 16, seed = 1, ...rest } = options;
  const first = solveIK(chain, q0, target, rest);
  if (first.converged) {
    unwrapToward(chain, first.q, q0);
    return { ...first, attempts: 1 };
  }
  const rand = rng(seed);
  const n = chain.dof;
  let best = first;
  let bestScore = first.positionError + first.orientationError * 0.1;
  let bestDist = Infinity;
  const seedQ = new Float64Array(n);
  let attempts = 1;
  for (let r = 0; r < restarts; r++) {
    for (let i = 0; i < n; i++) {
      const lo = chain.lower[i], hi = chain.upper[i];
      // Sample angular joints inside ±π of the start where possible — keeps solutions near the current branch.
      const a = chain.isAngular[i] ? Math.max(lo, q0[i] - Math.PI) : lo;
      const b = chain.isAngular[i] ? Math.min(hi, q0[i] + Math.PI) : hi;
      seedQ[i] = a + (b - a) * rand();
    }
    const res = solveIK(chain, seedQ, target, rest);
    attempts++;
    if (res.converged) {
      unwrapToward(chain, res.q, q0);
      const d = jointDistance(chain, res.q, q0);
      if (!best.converged || d < bestDist) {
        best = res;
        bestDist = d;
      }
    } else if (!best.converged) {
      const score = res.positionError + res.orientationError * 0.1;
      if (score < bestScore) {
        best = res;
        bestScore = score;
      }
    }
    // Good enough: a converged solution that does not move far.
    if (best.converged && bestDist < 1.0) break;
  }
  return { ...best, attempts };
}
