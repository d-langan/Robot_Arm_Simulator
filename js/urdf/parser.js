// URDF parser: XML text -> plain data model. No three.js dependency, no DOM
// beyond a DOMParser (browser global, or injected for Node tests).
//
// Spec reference: http://wiki.ros.org/urdf/XML
//
// Only direct children of <robot> are read as links/joints/materials, so the
// <joint> elements nested inside <transmission> and <gazebo> are ignored.

export const MOVABLE_TYPES = new Set(['revolute', 'continuous', 'prismatic']);
const KNOWN_JOINT_TYPES = new Set(['revolute', 'continuous', 'prismatic', 'fixed', 'floating', 'planar']);

// Ranges used when a URDF does not give usable limits (missing <limit>, or
// lower == upper, which SolidWorks-exported URDFs emit for "no limit").
export const DEFAULT_REVOLUTE_RANGE = Math.PI;      // +/- rad
export const DEFAULT_PRISMATIC_RANGE = 0.5;         // +/- m
export const CONTINUOUS_RANGE = 2 * Math.PI;        // UI / IK soft range for continuous joints
export const DEFAULT_REVOLUTE_VELOCITY = Math.PI / 2; // rad/s, used when velocity is 0/missing
export const DEFAULT_PRISMATIC_VELOCITY = 0.25;       // m/s

export class URDFParseError extends Error {}

function elementChildren(el, tag) {
  const out = [];
  for (let n = el.firstChild; n; n = n.nextSibling) {
    if (n.nodeType === 1 && (tag === undefined || n.nodeName === tag)) out.push(n);
  }
  return out;
}

function firstChild(el, tag) {
  for (let n = el.firstChild; n; n = n.nextSibling) {
    if (n.nodeType === 1 && n.nodeName === tag) return n;
  }
  return null;
}

function parseNumberList(str, count, fallback, warn, what) {
  if (str === null || str === undefined || str.trim() === '') return fallback.slice();
  const parts = str.trim().split(/[\s,]+/).map(Number);
  if (parts.length !== count || parts.some((v) => !Number.isFinite(v))) {
    warn(`${what}: could not parse "${str}" as ${count} numbers; using ${fallback.join(' ')}`);
    return fallback.slice();
  }
  return parts;
}

function parseNumber(str, fallback) {
  if (str === null || str === undefined || str.trim() === '') return fallback;
  const v = Number(str.trim());
  return Number.isFinite(v) ? v : fallback;
}

function attr(el, name) {
  return el && el.hasAttribute(name) ? el.getAttribute(name) : null;
}

function parseOrigin(el, warn, what) {
  const o = el ? firstChild(el, 'origin') : null;
  return {
    xyz: parseNumberList(attr(o, 'xyz'), 3, [0, 0, 0], warn, `${what} origin xyz`),
    rpy: parseNumberList(attr(o, 'rpy'), 3, [0, 0, 0], warn, `${what} origin rpy`),
  };
}

function parseColor(materialEl, warn, what) {
  const c = firstChild(materialEl, 'color');
  if (!c) return null;
  const rgba = parseNumberList(attr(c, 'rgba'), 4, [0.7, 0.7, 0.7, 1], warn, `${what} color rgba`);
  return rgba.map((v) => Math.min(1, Math.max(0, v)));
}

function parseMaterialEl(materialEl, warn, what) {
  const tex = firstChild(materialEl, 'texture');
  return {
    name: attr(materialEl, 'name') || '',
    color: parseColor(materialEl, warn, what),
    texture: tex ? attr(tex, 'filename') : null,
  };
}

function parseGeometry(geomEl, warn, what) {
  if (!geomEl) {
    warn(`${what}: missing <geometry>`);
    return null;
  }
  const shape = elementChildren(geomEl)[0];
  if (!shape) {
    warn(`${what}: empty <geometry>`);
    return null;
  }
  switch (shape.nodeName) {
    case 'box':
      return { type: 'box', size: parseNumberList(attr(shape, 'size'), 3, [0.1, 0.1, 0.1], warn, `${what} box size`) };
    case 'cylinder':
      return {
        type: 'cylinder',
        radius: parseNumber(attr(shape, 'radius'), 0.05),
        length: parseNumber(attr(shape, 'length'), 0.1),
      };
    case 'capsule': // not in the URDF spec, but used by some tools (e.g. Drake, MuJoCo exports)
      return {
        type: 'capsule',
        radius: parseNumber(attr(shape, 'radius'), 0.05),
        length: parseNumber(attr(shape, 'length'), 0.1),
      };
    case 'sphere':
      return { type: 'sphere', radius: parseNumber(attr(shape, 'radius'), 0.05) };
    case 'mesh': {
      const filename = attr(shape, 'filename');
      if (!filename) {
        warn(`${what}: <mesh> without filename`);
        return null;
      }
      let scale = [1, 1, 1];
      const s = attr(shape, 'scale');
      if (s && s.trim()) {
        const parts = s.trim().split(/[\s,]+/).map(Number);
        if (parts.length === 1 && Number.isFinite(parts[0])) scale = [parts[0], parts[0], parts[0]];
        else scale = parseNumberList(s, 3, [1, 1, 1], warn, `${what} mesh scale`);
      }
      return { type: 'mesh', filename: filename.trim(), scale };
    }
    default:
      warn(`${what}: unsupported geometry <${shape.nodeName}>`);
      return null;
  }
}

function looksLikeXacro(doc, text) {
  if (/xmlns:xacro\s*=/.test(text) && /<xacro:/.test(text)) return true;
  // Unexpanded property substitutions in attributes.
  return /="[^"]*\$\{[^}]*\}[^"]*"/.test(text) || /="[^"]*\$\((find|arg)\s/.test(text);
}

/**
 * Parse URDF XML text.
 * @param {string} text
 * @param {{ DOMParser?: any }} [opts]
 * @returns {RobotModel}
 */
export function parseURDF(text, opts = {}) {
  const DP = opts.DOMParser || globalThis.DOMParser;
  if (!DP) throw new URDFParseError('No DOMParser available');

  const warnings = [];
  const warn = (msg) => warnings.push(msg);

  let doc;
  try {
    doc = new DP().parseFromString(text, 'application/xml');
  } catch (e) {
    throw new URDFParseError(`XML parse error: ${e.message}`);
  }
  const perr = doc.getElementsByTagName('parsererror');
  if (perr.length) {
    throw new URDFParseError(`XML parse error: ${(perr[0].textContent || '').trim().split('\n')[0]}`);
  }

  // Root may be <robot>, or the file may wrap it (rare). Take the first <robot>.
  let robotEl = doc.documentElement;
  if (!robotEl || robotEl.nodeName !== 'robot') {
    const robots = doc.getElementsByTagName('robot');
    robotEl = robots.length ? robots[0] : null;
  }
  if (!robotEl) {
    if (doc.documentElement && doc.documentElement.nodeName === 'sdf') {
      throw new URDFParseError('This is an SDF file, not URDF. Convert it to URDF first (e.g. with sdformat_urdf or gz sdf).');
    }
    throw new URDFParseError('No <robot> element found — is this a URDF file?');
  }

  const isXacro = looksLikeXacro(doc, text);
  if (isXacro) {
    const hasMacros = /<xacro:(macro|include|property|if|unless|call)\b/.test(text);
    if (hasMacros || elementChildren(robotEl, 'link').length === 0) {
      throw new URDFParseError(
        'This file is xacro, not plain URDF. Expand it first:  xacro robot.urdf.xacro > robot.urdf  ' +
        '(ROS: `ros2 run xacro xacro …`, or `pip install xacro`).'
      );
    }
    warn('File contains unexpanded xacro expressions; results may be wrong. Run it through xacro first.');
  }

  const model = {
    name: attr(robotEl, 'name') || 'robot',
    links: new Map(),
    joints: new Map(),
    materials: new Map(),
    root: null,
    warnings,
  };

  // Global materials.
  for (const m of elementChildren(robotEl, 'material')) {
    const mat = parseMaterialEl(m, warn, `material "${attr(m, 'name')}"`);
    if (!mat.name) {
      warn('Global <material> without a name ignored');
      continue;
    }
    model.materials.set(mat.name, mat);
  }

  // Links.
  for (const l of elementChildren(robotEl, 'link')) {
    const name = attr(l, 'name');
    if (!name) {
      warn('<link> without a name ignored');
      continue;
    }
    if (model.links.has(name)) {
      warn(`Duplicate link "${name}" ignored`);
      continue;
    }
    const link = { name, visuals: [], collisions: [], parentJoint: null, childJoints: [] };

    for (const [tag, list] of [['visual', link.visuals], ['collision', link.collisions]]) {
      elementChildren(l, tag).forEach((v, i) => {
        const what = `link "${name}" ${tag}[${i}]`;
        const geometry = parseGeometry(firstChild(v, 'geometry'), warn, what);
        if (!geometry) return;
        let material = null;
        if (tag === 'visual') {
          const mEl = firstChild(v, 'material');
          if (mEl) {
            const local = parseMaterialEl(mEl, warn, `${what} material`);
            const global = local.name ? model.materials.get(local.name) : null;
            material = {
              name: local.name,
              color: local.color || (global && global.color) || null,
              texture: local.texture || (global && global.texture) || null,
            };
            if (!local.color && !local.texture && local.name && !global) {
              warn(`${what}: material "${local.name}" is not defined; using default color`);
            }
          }
        }
        list.push({ name: attr(v, 'name') || '', origin: parseOrigin(v, warn, what), geometry, material });
      });
    }
    model.links.set(name, link);
  }

  // Joints.
  for (const j of elementChildren(robotEl, 'joint')) {
    const name = attr(j, 'name');
    const what = `joint "${name}"`;
    if (!name) {
      warn('<joint> without a name ignored');
      continue;
    }
    if (model.joints.has(name)) {
      warn(`Duplicate ${what} ignored`);
      continue;
    }
    let type = (attr(j, 'type') || '').trim();
    if (!KNOWN_JOINT_TYPES.has(type)) {
      warn(`${what}: unknown type "${type}", treated as fixed`);
      type = 'fixed';
    }
    const parentEl = firstChild(j, 'parent');
    const childEl = firstChild(j, 'child');
    const parent = attr(parentEl, 'link');
    const child = attr(childEl, 'link');
    if (!parent || !child) {
      warn(`${what}: missing parent or child link; ignored`);
      continue;
    }
    if (!model.links.has(parent) || !model.links.has(child)) {
      warn(`${what}: references unknown link "${model.links.has(parent) ? child : parent}"; ignored`);
      continue;
    }
    if (parent === child) {
      warn(`${what}: parent and child are the same link; ignored`);
      continue;
    }

    let axis = parseNumberList(attr(firstChild(j, 'axis'), 'xyz'), 3, [1, 0, 0], warn, `${what} axis`);
    const norm = Math.hypot(axis[0], axis[1], axis[2]);
    if (norm < 1e-12) {
      if (MOVABLE_TYPES.has(type)) warn(`${what}: zero-length axis, using 1 0 0`);
      axis = [1, 0, 0];
    } else {
      axis = axis.map((v) => v / norm);
    }

    const limitEl = firstChild(j, 'limit');
    const limit = {
      lower: 0,
      upper: 0,
      effort: parseNumber(attr(limitEl, 'effort'), 0),
      velocity: parseNumber(attr(limitEl, 'velocity'), 0),
      specified: false,        // true when the URDF gave a usable lower < upper
      velocitySpecified: false,
    };
    if (type === 'revolute' || type === 'prismatic') {
      const range = type === 'revolute' ? DEFAULT_REVOLUTE_RANGE : DEFAULT_PRISMATIC_RANGE;
      let lo = parseNumber(attr(limitEl, 'lower'), 0);
      let hi = parseNumber(attr(limitEl, 'upper'), 0);
      if (lo > hi) {
        warn(`${what}: lower > upper, swapped`);
        [lo, hi] = [hi, lo];
      }
      if (!limitEl) {
        warn(`${what}: no <limit>; using ±${type === 'revolute' ? '180°' : range + ' m'}`);
        lo = -range; hi = range;
      } else if (hi - lo < 1e-9) {
        warn(`${what}: lower == upper (${lo}); treating as unspecified, using ±${type === 'revolute' ? '180°' : range + ' m'}`);
        lo = -range; hi = range;
      } else {
        limit.specified = true;
      }
      limit.lower = lo;
      limit.upper = hi;
    } else if (type === 'continuous') {
      limit.lower = -CONTINUOUS_RANGE;
      limit.upper = CONTINUOUS_RANGE;
    }
    if (limit.velocity > 0) {
      limit.velocitySpecified = true;
    } else if (MOVABLE_TYPES.has(type)) {
      limit.velocity = type === 'prismatic' ? DEFAULT_PRISMATIC_VELOCITY : DEFAULT_REVOLUTE_VELOCITY;
    }

    // Safety controller soft limits (informational).
    const safetyEl = firstChild(j, 'safety_controller');
    const safety = safetyEl
      ? {
          softLower: parseNumber(attr(safetyEl, 'soft_lower_limit'), null),
          softUpper: parseNumber(attr(safetyEl, 'soft_upper_limit'), null),
        }
      : null;

    const mimicEl = firstChild(j, 'mimic');
    const mimic = mimicEl && attr(mimicEl, 'joint')
      ? {
          joint: attr(mimicEl, 'joint'),
          multiplier: parseNumber(attr(mimicEl, 'multiplier'), 1),
          offset: parseNumber(attr(mimicEl, 'offset'), 0),
        }
      : null;

    if (type === 'floating' || type === 'planar') {
      warn(`${what}: ${type} joints are shown at their zero position and are not controllable`);
    }

    model.joints.set(name, {
      name,
      type,
      parent,
      child,
      origin: parseOrigin(j, warn, what),
      axis,
      limit,
      safety,
      mimic,
    });
  }

  // Tree structure.
  for (const joint of model.joints.values()) {
    const child = model.links.get(joint.child);
    if (child.parentJoint) {
      warn(`Link "${joint.child}" has more than one parent joint ("${child.parentJoint}", "${joint.name}"); "${joint.name}" ignored`);
      model.joints.delete(joint.name);
      continue;
    }
    child.parentJoint = joint.name;
    model.links.get(joint.parent).childJoints.push(joint.name);
  }

  // Cycle check: walk up from every link.
  for (const link of model.links.values()) {
    const seen = new Set();
    let cur = link;
    while (cur && cur.parentJoint) {
      if (seen.has(cur.name)) {
        throw new URDFParseError(`Kinematic loop detected involving link "${cur.name}" — URDF must be a tree`);
      }
      seen.add(cur.name);
      cur = model.links.get(model.joints.get(cur.parentJoint).parent);
    }
  }

  const roots = [...model.links.values()].filter((l) => !l.parentJoint);
  if (roots.length === 0) {
    throw new URDFParseError(model.links.size ? 'No root link found' : 'URDF has no links');
  }
  if (roots.length > 1) {
    // Pick the root with the most descendants; the rest are disconnected.
    const count = (l) => l.childJoints.reduce((s, jn) => s + 1 + count(model.links.get(model.joints.get(jn).child)), 0);
    roots.sort((a, b) => count(b) - count(a));
    warn(`URDF has ${roots.length} root links (${roots.map((r) => r.name).join(', ')}); using "${roots[0].name}", others are not shown`);
  }
  model.root = roots[0].name;

  // Mimic resolution: follow chains, detect cycles / bad targets.
  for (const joint of model.joints.values()) {
    if (!joint.mimic) continue;
    if (!MOVABLE_TYPES.has(joint.type)) {
      joint.mimic = null;
      continue;
    }
    let multiplier = 1;
    let offset = 0;
    let cur = joint;
    const seen = new Set([joint.name]);
    let ok = true;
    while (cur.mimic) {
      const target = model.joints.get(cur.mimic.joint);
      if (!target || !MOVABLE_TYPES.has(target.type)) {
        warn(`joint "${joint.name}": mimics "${cur.mimic.joint}" which is missing or not movable; mimic ignored`);
        ok = false;
        break;
      }
      if (seen.has(target.name)) {
        warn(`joint "${joint.name}": mimic cycle; mimic ignored`);
        ok = false;
        break;
      }
      seen.add(target.name);
      // value(cur) = m * value(target) + o ;  value(joint) = M * value(cur) + O
      offset = multiplier * cur.mimic.offset + offset;
      multiplier = multiplier * cur.mimic.multiplier;
      cur = target;
    }
    if (ok) {
      joint.mimicResolved = { master: cur.name, multiplier, offset };
    } else {
      joint.mimic = null;
    }
  }

  return model;
}

/** Joints that the user controls directly (movable, not mimic), in depth-first tree order. */
export function controllableJoints(model) {
  const out = [];
  const visit = (linkName) => {
    const link = model.links.get(linkName);
    for (const jn of link.childJoints) {
      const j = model.joints.get(jn);
      if (MOVABLE_TYPES.has(j.type) && !j.mimicResolved) out.push(j);
      visit(j.child);
    }
  };
  visit(model.root);
  return out;
}

/** Links reachable from the root (the ones that get displayed). */
export function reachableLinks(model) {
  const out = [];
  const visit = (name) => {
    out.push(name);
    for (const jn of model.links.get(name).childJoints) visit(model.joints.get(jn).child);
  };
  visit(model.root);
  return out;
}

/** Path of joint names from ancestor link `base` down to link `tip`, or null if base is not an ancestor. */
export function jointPath(model, base, tip) {
  const path = [];
  let cur = model.links.get(tip);
  while (cur && cur.name !== base) {
    if (!cur.parentJoint) return null;
    const j = model.joints.get(cur.parentJoint);
    path.push(j.name);
    cur = model.links.get(j.parent);
  }
  return cur ? path.reverse() : null;
}

/** Clamp a value into the joint's range. */
export function clampJoint(joint, v) {
  if (!MOVABLE_TYPES.has(joint.type)) return 0;
  return Math.min(joint.limit.upper, Math.max(joint.limit.lower, v));
}
