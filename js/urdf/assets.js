// Asset resolvers: turn a URDF mesh/texture reference (package://, file://,
// model://, http(s)://, or a relative path) into bytes.
//
//  - UrlResolver:  the URDF was loaded from a URL (built-in robots, ?urdf=...).
//  - FileResolver: the URDF came from user-supplied files (picker, folder, drag & drop, zip).
//
// Both expose the same interface:
//    fetch(ref) -> Promise<{ data: ArrayBuffer, path: string, base: string }>
//    modifyUrl(url) -> string      (for three.js LoadingManager; resolves textures referenced by meshes)
//    dispose()

export class AssetNotFoundError extends Error {
  constructor(ref, tried = []) {
    super(`Asset not found: ${ref}`);
    this.ref = ref;
    this.tried = tried;
  }
}

/** Normalise a slash path: backslashes -> '/', drop '.', resolve '..', strip leading '/'. */
export function normalizePath(p) {
  const out = [];
  for (const seg of String(p).replace(/\\/g, '/').split('/')) {
    if (seg === '' || seg === '.') continue;
    if (seg === '..') {
      if (out.length) out.pop();
      continue;
    }
    out.push(seg);
  }
  return out.join('/');
}

export function dirname(p) {
  const n = normalizePath(p);
  const i = n.lastIndexOf('/');
  return i < 0 ? '' : n.slice(0, i);
}

export function extname(p) {
  const clean = String(p).split(/[?#]/)[0];
  const base = clean.slice(clean.lastIndexOf('/') + 1);
  const i = base.lastIndexOf('.');
  return i <= 0 ? '' : base.slice(i + 1).toLowerCase();
}

/**
 * Split a URDF reference into its scheme and the path part.
 *   package://pkg/a/b.stl -> { scheme: 'package', pkg: 'pkg', path: 'a/b.stl' }
 *   model://pkg/a/b.stl   -> same with scheme 'model'
 *   file:///abs/x.stl     -> { scheme: 'file', path: 'abs/x.stl' }
 *   http(s)://...         -> { scheme: 'http', url }
 *   meshes/x.stl          -> { scheme: 'relative', path: 'meshes/x.stl' }
 */
export function parseRef(ref) {
  const s = String(ref).trim();
  let m = /^(package|model):\/\/([^/]*)\/?(.*)$/i.exec(s);
  if (m) return { scheme: m[1].toLowerCase(), pkg: m[2], path: normalizePath(m[3]) };
  m = /^file:\/\/(.*)$/i.exec(s);
  if (m) {
    // file:///C:/x/y.stl or file:///home/x/y.stl
    return { scheme: 'file', path: normalizePath(decodeURIComponent(m[1]).replace(/^\/([a-zA-Z]:)/, '$1')) };
  }
  if (/^(https?|blob|data):/i.test(s)) return { scheme: 'http', url: s };
  return { scheme: 'relative', path: s.replace(/\\/g, '/') };
}

// ---------------------------------------------------------------------------

export class UrlResolver {
  /**
   * @param {string} urdfUrl absolute or page-relative URL of the URDF file
   * @param {Record<string,string>} [packages] optional map pkg name -> package root URL
   */
  constructor(urdfUrl, packages = {}) {
    this.urdfUrl = new URL(urdfUrl, globalThis.location ? globalThis.location.href : undefined).href;
    this.packages = packages;
    this.kind = 'url';
  }

  candidates(ref) {
    const r = parseRef(ref);
    const u = this.urdfUrl;
    switch (r.scheme) {
      case 'http':
        return [r.url];
      case 'relative':
        return [new URL(r.path, u).href];
      case 'package':
      case 'model': {
        const out = [];
        if (this.packages[r.pkg]) out.push(new URL(r.path, ensureSlash(new URL(this.packages[r.pkg], u).href)).href);
        // URL contains /<pkg>/ -> package root is there.
        const marker = `/${r.pkg}/`;
        const i = u.lastIndexOf(marker);
        if (r.pkg && i >= 0) out.push(u.slice(0, i + marker.length) + r.path);
        // Typical layout: <pkg>/urdf/robot.urdf -> package root is the URDF's parent dir.
        out.push(new URL('../' + r.path, u).href);
        // URDF at the package root.
        out.push(new URL(r.path, u).href);
        // Package folder next to the URDF.
        if (r.pkg) out.push(new URL(`${r.pkg}/${r.path}`, u).href);
        return [...new Set(out)];
      }
      case 'file': {
        // Absolute local paths cannot be fetched; try the trailing segments relative to the URDF.
        const segs = r.path.split('/');
        const out = [];
        for (let k = Math.min(4, segs.length); k >= 1; k--) {
          const tail = segs.slice(-k).join('/');
          out.push(new URL('../' + tail, u).href, new URL(tail, u).href);
        }
        return [...new Set(out)];
      }
      default:
        return [];
    }
  }

  async fetch(ref) {
    const tried = this.candidates(ref);
    for (const url of tried) {
      try {
        const res = await fetch(url);
        if (!res.ok) continue;
        const data = await res.arrayBuffer();
        return { data, path: url, base: url.slice(0, url.lastIndexOf('/') + 1) };
      } catch {
        /* try next */
      }
    }
    throw new AssetNotFoundError(ref, tried);
  }

  modifyUrl(url) {
    return url;
  }

  dispose() {}
}

function ensureSlash(u) {
  return u.endsWith('/') ? u : u + '/';
}

// ---------------------------------------------------------------------------

const VFS = 'vfs:/';

export class FileResolver {
  /**
   * @param {{path: string, file: Blob}[]} entries all user-supplied files, with relative paths
   * @param {string} urdfPath path (within entries) of the URDF being loaded
   */
  constructor(entries, urdfPath) {
    this.kind = 'files';
    this.entries = entries.map((e) => ({ path: normalizePath(e.path), file: e.file }));
    this.byPath = new Map(this.entries.map((e) => [e.path, e]));
    this.byLower = new Map();
    for (const e of this.entries) {
      const k = e.path.toLowerCase();
      if (!this.byLower.has(k)) this.byLower.set(k, e);
    }
    this.urdfDir = dirname(urdfPath);
    this.objectUrls = new Map(); // entry path -> blob URL
    this.cache = new Map();
  }

  /** Best entry whose path ends with the most trailing segments of `path` (case-insensitive). */
  suffixMatch(path, pkg) {
    const target = normalizePath(path).toLowerCase().split('/');
    if (!target.length || !target[target.length - 1]) return null;
    let best = null;
    let bestScore = 0;
    for (const e of this.entries) {
      const segs = e.path.toLowerCase().split('/');
      let k = 0;
      while (k < target.length && k < segs.length && segs[segs.length - 1 - k] === target[target.length - 1 - k]) k++;
      if (k === 0) continue;
      let score = k * 10;
      if (pkg && segs.slice(0, segs.length - k).includes(pkg.toLowerCase())) score += 5;
      if (e.path.split('/').slice(-k).join('/') === normalizePath(path).split('/').slice(-k).join('/')) score += 1; // exact case
      score -= segs.length * 0.01; // prefer shallower
      if (score > bestScore) {
        bestScore = score;
        best = e;
      }
    }
    return best;
  }

  /** Resolve a reference to an entry. `fromDir` is the directory relative references start from. */
  resolveEntry(ref, fromDir = this.urdfDir) {
    const key = `${fromDir}|${ref}`;
    if (this.cache.has(key)) return this.cache.get(key);
    const r = parseRef(ref);
    let entry = null;
    if (r.scheme === 'relative') {
      const joined = normalizePath((fromDir ? fromDir + '/' : '') + r.path);
      entry = this.byPath.get(joined) || this.byLower.get(joined.toLowerCase()) || this.suffixMatch(r.path);
    } else if (r.scheme === 'package' || r.scheme === 'model') {
      const full = normalizePath(`${r.pkg}/${r.path}`);
      entry = this.byPath.get(full) || this.suffixMatch(full, r.pkg);
    } else if (r.scheme === 'file') {
      entry = this.suffixMatch(r.path);
    } else if (r.scheme === 'http') {
      entry = this.suffixMatch(r.url.split(/[?#]/)[0].replace(/^[a-z]+:\/\/[^/]*/i, ''));
    }
    this.cache.set(key, entry);
    return entry;
  }

  async fetch(ref, fromDir) {
    const e = this.resolveEntry(ref, fromDir);
    if (!e) throw new AssetNotFoundError(ref);
    const data = await e.file.arrayBuffer();
    return { data, path: e.path, base: VFS + (dirname(e.path) ? dirname(e.path) + '/' : '') };
  }

  objectUrl(entry) {
    let u = this.objectUrls.get(entry.path);
    if (!u) {
      u = URL.createObjectURL(entry.file);
      this.objectUrls.set(entry.path, u);
    }
    return u;
  }

  /** LoadingManager URL modifier: map vfs:/dir/file.png (textures inside meshes) to blob URLs. */
  modifyUrl(url) {
    if (url.startsWith(VFS)) {
      const rest = url.slice(VFS.length);
      // rest = "<meshdir>/<texture ref>" where the texture ref itself may be absolute or a URL.
      const e = this.byPath.get(normalizePath(rest)) || this.byLower.get(normalizePath(rest).toLowerCase()) || this.suffixMatch(rest);
      if (e) return this.objectUrl(e);
      return 'data:,missing';
    }
    if (/^(blob|data):/i.test(url)) return url;
    // Absolute URL to somewhere else (e.g. file:// inside a DAE) — try the files we have.
    const e = this.suffixMatch(url.split(/[?#]/)[0].replace(/^[a-z]+:\/\/[^/]*/i, ''));
    return e ? this.objectUrl(e) : url;
  }

  dispose() {
    for (const u of this.objectUrls.values()) URL.revokeObjectURL(u);
    this.objectUrls.clear();
  }
}
