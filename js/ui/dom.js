// Small DOM helpers shared by the panels.

/** h('div.class#id', {attrs, on: {click}}, ...children) */
export function h(tag, props = {}, ...children) {
  const m = /^([a-z0-9-]+)?((?:[.#][\w-]+)*)$/i.exec(tag);
  const el = document.createElement((m && m[1]) || 'div');
  if (m && m[2]) {
    for (const part of m[2].match(/[.#][\w-]+/g) || []) {
      if (part[0] === '.') el.classList.add(part.slice(1));
      else el.id = part.slice(1);
    }
  }
  if (props === null || typeof props !== 'object' || props instanceof Node || Array.isArray(props)) {
    children.unshift(props);
    props = {};
  }
  for (const [k, v] of Object.entries(props)) {
    if (v === undefined || v === null || v === false) continue;
    if (k === 'on') for (const [ev, fn] of Object.entries(v)) el.addEventListener(ev, fn);
    else if (k === 'class') el.className = v;
    else if (k === 'style' && typeof v === 'object') Object.assign(el.style, v);
    else if (k === 'dataset') Object.assign(el.dataset, v);
    else if (k in el && typeof v !== 'string') el[k] = v;
    else el.setAttribute(k, v === true ? '' : v);
  }
  append(el, children);
  return el;
}

/** Replace an element's children, skipping null/false (native replaceChildren would print "null"). */
export function setChildren(el, ...children) {
  el.replaceChildren();
  append(el, children);
  return el;
}

function append(el, children) {
  for (const c of children.flat(Infinity)) {
    if (c === null || c === undefined || c === false) continue;
    el.append(c instanceof Node ? c : document.createTextNode(String(c)));
  }
}

export const RAD2DEG = 180 / Math.PI;
export const DEG2RAD = Math.PI / 180;

/** Format a joint value for display: degrees for angular joints, millimetres for prismatic. */
export function fmtJoint(joint, v, digits = 1) {
  return joint.type === 'prismatic' ? (v * 1000).toFixed(digits) : (v * RAD2DEG).toFixed(digits);
}
export function jointUnit(joint) {
  return joint.type === 'prismatic' ? 'mm' : '°';
}
export function toDisplay(joint, v) {
  return joint.type === 'prismatic' ? v * 1000 : v * RAD2DEG;
}
export function fromDisplay(joint, d) {
  return joint.type === 'prismatic' ? d / 1000 : d * DEG2RAD;
}

let toastRoot = null;
/** Transient message. kind: 'info' | 'warn' | 'error' | 'ok' */
export function toast(message, kind = 'info', ms = 3800) {
  if (!toastRoot) toastRoot = document.getElementById('toasts');
  const t = h(`div.toast.toast-${kind}`, { role: kind === 'error' ? 'alert' : 'status' }, message);
  toastRoot.append(t);
  requestAnimationFrame(() => t.classList.add('show'));
  setTimeout(() => {
    t.classList.remove('show');
    setTimeout(() => t.remove(), 300);
  }, ms);
}

/** Wire press-and-hold on a button: onStart/onEnd, with pointer capture. */
export function holdButton(btn, onStart, onEnd) {
  let active = false;
  const end = () => {
    if (!active) return;
    active = false;
    btn.classList.remove('held');
    onEnd();
  };
  btn.addEventListener('pointerdown', (e) => {
    if (e.button !== 0) return;
    e.preventDefault();
    active = true;
    btn.setPointerCapture(e.pointerId);
    btn.classList.add('held');
    onStart();
  });
  btn.addEventListener('pointerup', end);
  btn.addEventListener('pointercancel', end);
  btn.addEventListener('lostpointercapture', end);
  btn.addEventListener('keydown', (e) => {
    if (e.key !== ' ' && e.key !== 'Enter') return;
    // Always claim the key (repeats too) so global shortcuts don't also act on it.
    e.preventDefault();
    if (!active && !e.repeat) {
      active = true;
      btn.classList.add('held');
      onStart();
    }
  });
  btn.addEventListener('keyup', (e) => { if (e.key === ' ' || e.key === 'Enter') end(); });
  btn.addEventListener('blur', end);
}

/** Download a JSON object as a file. */
export function downloadJSON(obj, filename) {
  const blob = new Blob([JSON.stringify(obj, null, 2)], { type: 'application/json' });
  const url = URL.createObjectURL(blob);
  const a = h('a', { href: url, download: filename });
  document.body.append(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

/** localStorage that never throws. */
export const store = {
  get(key, fallback = null) {
    try {
      const v = localStorage.getItem(key);
      return v === null ? fallback : JSON.parse(v);
    } catch {
      return fallback;
    }
  },
  set(key, value) {
    try {
      localStorage.setItem(key, JSON.stringify(value));
      return true;
    } catch {
      return false;
    }
  },
};
