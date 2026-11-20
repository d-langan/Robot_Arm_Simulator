// three.js scene graph for a parsed URDF model.
//
//   link group
//    ├─ visual group     (one child per <visual>, positioned at its origin)
//    ├─ collision group  (hidden by default)
//    ├─ frame axes       (hidden by default)
//    └─ joint origin (parent→joint transform)
//         ├─ joint axis arrow
//         └─ joint motion (rotation/translation from the joint value)
//              └─ child link group …

import {
  Group, Object3D, AxesHelper, ArrowHelper, Vector3, Matrix4, MeshStandardMaterial,
  BufferGeometry, LineSegments, LineBasicMaterial, Float32BufferAttribute, Mesh, SphereGeometry,
} from 'three';
import { MOVABLE_TYPES, controllableJoints, clampJoint } from './parser.js';
import { originMatrix } from '../kinematics/chain.js';
import { MeshLoader, makePrimitive, makeUrdfMaterial } from './meshes.js';

const COLLISION_MATERIAL = new MeshStandardMaterial({
  color: 0xff7a1a, transparent: true, opacity: 0.35, depthWrite: false, roughness: 0.8,
});

function applyOrigin(obj, origin) {
  originMatrix(origin).decompose(obj.position, obj.quaternion, obj.scale);
}

export class Robot {
  /**
   * @param {object} model parsed URDF (parser.js)
   * @param {UrlResolver|FileResolver} resolver
   */
  constructor(model, resolver) {
    this.model = model;
    this.name = model.name;
    this.object = new Group();
    this.object.name = `robot:${model.name}`;
    this.links = new Map();   // name -> { group, visual, collision, axes }
    this.joints = new Map();  // name -> { joint, origin, motion, arrow }
    this.controllable = controllableJoints(model);
    this.values = new Map();  // every movable joint (incl. mimic) -> value
    this.followers = new Map(); // master name -> [mimic joint]
    this.meshLoader = new MeshLoader(resolver);
    this.pending = [];
    this.stats = { visuals: 0, collisions: 0, meshes: 0 };

    for (const j of model.joints.values()) {
      if (j.mimicResolved) {
        const list = this.followers.get(j.mimicResolved.master) || [];
        list.push(j);
        this.followers.set(j.mimicResolved.master, list);
      }
    }

    this.object.add(this._buildLink(model.root));
    this.skeleton = this._buildSkeleton();
    this.object.add(this.skeleton);

    for (const j of this.controllable) this.setJointValue(j.name, clampJoint(j, 0));

    /** Resolves when every mesh has loaded (or failed). */
    this.ready = Promise.all(this.pending).then(() => {
      this.pending = [];
      return this;
    });
  }

  _buildLink(name) {
    const link = this.model.links.get(name);
    const group = new Group();
    group.name = name;
    group.userData.link = name;

    const visual = new Group();
    visual.name = 'visual';
    const collision = new Group();
    collision.name = 'collision';
    collision.visible = false;
    group.add(visual, collision);

    const addGeometry = (parent, g, isCollision) => {
      const holder = new Object3D();
      applyOrigin(holder, g.origin);
      parent.add(holder);
      const setup = (obj) => {
        obj.traverse((c) => {
          if (!c.isMesh) return;
          c.userData.link = name;
          if (isCollision) {
            c.material = COLLISION_MATERIAL;
            c.castShadow = false;
          } else {
            c.castShadow = true;
            c.receiveShadow = true;
          }
        });
        holder.add(obj);
      };
      if (g.geometry.type === 'mesh') {
        holder.scale.set(...g.geometry.scale);
        this.stats.meshes++;
        this.pending.push(
          this.meshLoader.load(g.geometry, isCollision ? null : g.material).then((obj) => { if (obj) setup(obj); }),
        );
      } else {
        const mat = isCollision ? COLLISION_MATERIAL : makeUrdfMaterial(g.material, this.meshLoader.resolver, this.meshLoader.manager);
        const prim = makePrimitive(g.geometry, mat);
        if (prim) setup(prim);
      }
    };
    for (const v of link.visuals) { addGeometry(visual, v, false); this.stats.visuals++; }
    for (const c of link.collisions) { addGeometry(collision, c, true); this.stats.collisions++; }

    const axes = new AxesHelper(0.08);
    axes.visible = false;
    axes.renderOrder = 10;
    axes.material.depthTest = false;
    group.add(axes);

    this.links.set(name, { group, visual, collision, axes });

    for (const jn of link.childJoints) {
      const joint = this.model.joints.get(jn);
      const origin = new Object3D();
      origin.name = `joint:${jn}`;
      applyOrigin(origin, joint.origin);
      const motion = new Object3D();
      motion.name = `motion:${jn}`;
      origin.add(motion);
      let arrow = null;
      if (MOVABLE_TYPES.has(joint.type)) {
        arrow = new ArrowHelper(new Vector3(...joint.axis), new Vector3(), 0.15,
          joint.type === 'prismatic' ? 0x2bd9a8 : 0xffcc33, 0.04, 0.025);
        arrow.visible = false;
        arrow.renderOrder = 11;
        arrow.traverse((c) => { if (c.material) { c.material.depthTest = false; c.material.transparent = true; } });
        origin.add(arrow);
      }
      motion.add(this._buildLink(joint.child));
      group.add(origin);
      this.joints.set(jn, { joint, origin, motion, arrow, axis: new Vector3(...joint.axis) });
      if (MOVABLE_TYPES.has(joint.type)) this.values.set(jn, 0);
    }
    return group;
  }

  /** Line skeleton between joint frames — readable even when meshes are missing. */
  _buildSkeleton() {
    const geo = new BufferGeometry();
    const line = new LineSegments(geo, new LineBasicMaterial({ color: 0x5ac8ff, depthTest: false, transparent: true }));
    line.renderOrder = 9;
    line.visible = false;
    line.frustumCulled = false;
    line.name = 'skeleton';
    const dots = new Group();
    const dotGeo = new SphereGeometry(0.012, 12, 8);
    const dotMat = new MeshStandardMaterial({ color: 0x5ac8ff, depthTest: false, transparent: true });
    const pairs = [];
    for (const j of this.model.joints.values()) {
      if (!this.joints.has(j.name)) continue;
      pairs.push(j);
      const d = new Mesh(dotGeo, dotMat);
      d.renderOrder = 9;
      d.userData.joint = j.name;
      dots.add(d);
    }
    const root = new Group();
    root.name = 'skeleton';
    root.visible = false;
    root.add(line, dots);
    root.userData.update = () => {
      this.object.updateMatrixWorld(true);
      const inv = new Matrix4().copy(this.object.matrixWorld).invert();
      const pos = [];
      const a = new Vector3();
      const b = new Vector3();
      let i = 0;
      for (const j of pairs) {
        const parent = this.links.get(j.parent).group;
        a.setFromMatrixPosition(parent.matrixWorld).applyMatrix4(inv);
        b.setFromMatrixPosition(this.joints.get(j.name).origin.matrixWorld).applyMatrix4(inv);
        pos.push(a.x, a.y, a.z, b.x, b.y, b.z);
        dots.children[i++].position.copy(b);
      }
      geo.setAttribute('position', new Float32BufferAttribute(pos, 3));
      geo.computeBoundingSphere();
    };
    line.visible = true;
    return root;
  }

  _apply(name, v) {
    const j = this.joints.get(name);
    if (!j) return;
    this.values.set(name, v);
    if (j.joint.type === 'prismatic') {
      j.motion.position.copy(j.axis).multiplyScalar(v);
    } else {
      j.motion.quaternion.setFromAxisAngle(j.axis, v);
    }
  }

  /** Set a controllable joint (clamped to limits); mimic followers update too. Returns the applied value. */
  setJointValue(name, value) {
    const j = this.model.joints.get(name);
    if (!j || !MOVABLE_TYPES.has(j.type) || j.mimicResolved) return null;
    const v = clampJoint(j, Number(value) || 0);
    this._apply(name, v);
    for (const f of this.followers.get(name) || []) {
      this._apply(f.name, f.mimicResolved.multiplier * v + f.mimicResolved.offset);
    }
    if (this.skeleton.visible) this.skeleton.userData.update();
    return v;
  }

  /** Set several joints from a {name: value} object or Map. Unknown names are ignored. */
  setJointValues(values) {
    const entries = values instanceof Map ? values.entries() : Object.entries(values);
    for (const [k, v] of entries) this.setJointValue(k, v);
  }

  getJointValue(name) {
    return this.values.get(name) ?? 0;
  }

  /** {name: value} for the controllable joints. */
  getJointValues() {
    const out = {};
    for (const j of this.controllable) out[j.name] = this.values.get(j.name) ?? 0;
    return out;
  }

  /** World matrix of a link frame (forces a matrix update). */
  linkWorldMatrix(name, out = new Matrix4()) {
    const l = this.links.get(name);
    if (!l) return out.identity();
    l.group.updateWorldMatrix(true, false);
    return out.copy(l.group.matrixWorld);
  }

  setDisplay({ visual, collision, frames, axes, skeleton } = {}) {
    for (const l of this.links.values()) {
      if (visual !== undefined) l.visual.visible = visual;
      if (collision !== undefined) l.collision.visible = collision;
      if (frames !== undefined) l.axes.visible = frames;
    }
    if (axes !== undefined) for (const j of this.joints.values()) if (j.arrow) j.arrow.visible = axes;
    if (skeleton !== undefined) {
      this.skeleton.visible = skeleton;
      if (skeleton) this.skeleton.userData.update();
    }
  }

  /** Link names that have at least one movable joint above them (candidate end-effectors). */
  movingLinks() {
    const out = [];
    const visit = (name, moving) => {
      if (moving) out.push(name);
      for (const jn of this.model.links.get(name).childJoints) {
        const j = this.model.joints.get(jn);
        visit(j.child, moving || MOVABLE_TYPES.has(j.type));
      }
    };
    visit(this.model.root, false);
    return out;
  }

  /** Links rigidly attached to the root (valid Cartesian reference frames). */
  staticLinks() {
    const out = [];
    const visit = (name) => {
      out.push(name);
      for (const jn of this.model.links.get(name).childJoints) {
        const j = this.model.joints.get(jn);
        if (!MOVABLE_TYPES.has(j.type) && j.type === 'fixed') visit(j.child);
      }
    };
    visit(this.model.root);
    return out;
  }

  dispose() {
    this.object.traverse((c) => {
      if (c.geometry) c.geometry.dispose();
      if (c.material) {
        for (const m of Array.isArray(c.material) ? c.material : [c.material]) {
          if (m === COLLISION_MATERIAL) continue;
          if (m.map) m.map.dispose();
          m.dispose();
        }
      }
    });
    this.meshLoader.dispose();
    this.object.removeFromParent();
  }
}

// Movable joints that actuate a tool (gripper fingers, jaws) rather than position it.
const TOOL_JOINT = /(finger|gripper|jaw|claw|knuckle|thumb|grip)/i;
// Conventional end-effector frame names, best first.
const PREFERRED_TIPS = ['tcp', 'tool0', 'tool_center_point', 'tool', 'end_effector', 'endeffector', 'eef', 'ee_link', 'ee', 'gripper', 'hand', 'flange'];

/**
 * Pick a sensible default end-effector:
 *  1. ignore the subtrees of tool joints (named like finger/gripper/jaw, or mimic joints),
 *     so finger slides never become part of the arm chain;
 *  2. among links with the most arm joints above them, prefer conventional frame names
 *     (exact name, then a whole name part such as "panda_hand_tcp" -> "tcp"), then the deepest frame.
 * Falls back to counting every joint when the robot is only a hand/gripper.
 */
export function defaultTipLink(model) {
  // Joint name only: a wrist joint's child link is often called "gripper_link".
  const isTool = (j) => !!j.mimicResolved || TOOL_JOINT.test(j.name);
  const score = (name, depth) => {
    const n = name.toLowerCase();
    const parts = n.split(/[^a-z0-9]+/);
    let s = 0;
    PREFERRED_TIPS.forEach((p, i) => {
      if (n === p) s = Math.max(s, 1000 - i * 10);
      else if (parts.includes(p) || (p.includes('_') && n.includes(p))) s = Math.max(s, 500 - i * 10);
    });
    return s + depth * 0.01;
  };
  const pick = (skipTools) => {
    const info = [];
    const visit = (name, dof, depth) => {
      info.push({ name, dof, depth });
      for (const jn of model.links.get(name).childJoints) {
        const j = model.joints.get(jn);
        const movable = MOVABLE_TYPES.has(j.type);
        if (skipTools && movable && isTool(j)) continue;
        visit(j.child, dof + (movable && !j.mimicResolved ? 1 : 0), depth + 1);
      }
    };
    visit(model.root, 0, 0);
    const maxD = Math.max(...info.map((x) => x.dof));
    if (maxD === 0) return null;
    let best = null;
    for (const x of info) {
      if (x.dof !== maxD) continue;
      const s = score(x.name, x.depth);
      if (!best || s > best.s) best = { name: x.name, s };
    }
    return best.name;
  };
  return pick(true) || pick(false) || model.root;
}
