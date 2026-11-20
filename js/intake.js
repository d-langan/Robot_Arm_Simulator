// Collect user-supplied files (file picker, folder picker, drag & drop, .zip)
// into a flat list of { path, file } and find the URDF(s) among them.

import { normalizePath, extname } from './urdf/assets.js';

const FFLATE_URL = 'https://cdn.jsdelivr.net/npm/fflate@0.8.2/esm/browser.js';

/** From an <input type=file> FileList (folder inputs provide webkitRelativePath). */
export function entriesFromFileList(fileList) {
  return [...fileList].map((f) => ({ path: normalizePath(f.webkitRelativePath || f.name), file: f }));
}

/** From a drop event's DataTransfer — walks dropped folders recursively. */
export async function entriesFromDataTransfer(dt) {
  const items = dt.items ? [...dt.items] : [];
  const roots = items
    .filter((it) => it.kind === 'file')
    .map((it) => (it.webkitGetAsEntry ? it.webkitGetAsEntry() : null));
  if (!roots.length || roots.some((r) => !r)) return entriesFromFileList(dt.files);
  const out = [];
  const walk = async (entry, prefix) => {
    if (entry.isFile) {
      const file = await new Promise((res, rej) => entry.file(res, rej));
      out.push({ path: normalizePath(prefix + entry.name), file });
    } else if (entry.isDirectory) {
      const reader = entry.createReader();
      // readEntries returns batches (Chrome: 100 at a time) until empty.
      for (;;) {
        const batch = await new Promise((res, rej) => reader.readEntries(res, rej));
        if (!batch.length) break;
        for (const e of batch) await walk(e, `${prefix}${entry.name}/`);
      }
    }
  };
  for (const r of roots) await walk(r, '');
  return out;
}

/** Replace any .zip entries with their contents (paths prefixed by the zip's folder). */
export async function expandZips(entries) {
  const zips = entries.filter((e) => extname(e.path) === 'zip');
  if (!zips.length) return entries;
  const { unzipSync } = await import(FFLATE_URL);
  const out = entries.filter((e) => extname(e.path) !== 'zip');
  for (const z of zips) {
    const buf = new Uint8Array(await z.file.arrayBuffer());
    const files = unzipSync(buf, { filter: (f) => !f.name.endsWith('/') && !f.name.startsWith('__MACOSX/') });
    const base = z.path.replace(/\.zip$/i, '');
    for (const [name, data] of Object.entries(files)) {
      out.push({ path: normalizePath(`${base}/${name}`), file: new Blob([data]) });
    }
  }
  return out;
}

/**
 * Find loadable robot descriptions among the entries.
 * Returns { urdfs: entry[], xacros: entry[], sdfs: entry[] }.
 */
export async function findRobotFiles(entries) {
  const urdfs = [];
  const xacros = [];
  const sdfs = [];
  for (const e of entries) {
    const ext = extname(e.path);
    const lower = e.path.toLowerCase();
    if (lower.endsWith('.urdf.xacro') || ext === 'xacro') xacros.push(e);
    else if (ext === 'urdf') urdfs.push(e);
    else if (ext === 'sdf') sdfs.push(e);
    else if (ext === 'xml' && e.file.size < 20e6) {
      // xacro output keeps xmlns:xacro and comments, so only real <xacro:...> tags mean xacro.
      const head = (await e.file.slice(0, 4096).text()).replace(/<!--[\s\S]*?(-->|$)/g, '');
      if (/<robot[\s>]/.test(head)) (/<xacro:/.test(head) ? xacros : urdfs).push(e);
    }
  }
  // Prefer shallow paths and files whose name matches their folder.
  const depth = (p) => p.split('/').length;
  urdfs.sort((a, b) => depth(a.path) - depth(b.path) || a.path.localeCompare(b.path));
  return { urdfs, xacros, sdfs };
}
