// 3D markers for program points and a preview of the path between them.
// MoveL segments draw as straight lines; MoveJ segments are sampled through FK,
// so the preview shows the real (curved) TCP path.

import {
  Group, Mesh, SphereGeometry, MeshBasicMaterial, AxesHelper, BufferGeometry, Line,
  LineBasicMaterial, LineDashedMaterial, Float32BufferAttribute, Matrix4, Vector3, Quaternion,
} from 'three';
import { CSS2DObject } from 'three/addons/renderers/CSS2DRenderer.js';
import { smoothstep } from './program.js';

const COLORS = { J: 0x4ea8ff, L: 0xffa23d, active: 0x3ddc84 };

export class WaypointMarkers {
  constructor(scene) {
    this.group = new Group();
    this.group.name = 'waypoints';
    scene.add(this.group);
    this.sphere = new SphereGeometry(0.014, 16, 10);
    this.visible = true;
  }

  clear() {
    for (const c of [...this.group.children]) {
      c.traverse((o) => {
        if (o.isCSS2DObject) o.element.remove();
        if (o.geometry && o.geometry !== this.sphere) o.geometry.dispose();
        if (o.material) o.material.dispose();
      });
      this.group.remove(c);
    }
  }

  setVisible(on) {
    this.visible = on;
    this.group.visible = on;
    this.group.traverse((o) => { if (o.isCSS2DObject) o.element.style.display = on ? '' : 'none'; });
  }

  /**
   * @param {object[]} points program points
   * @param {KinematicChain} chain current chain (TCP definition)
   * @param {object} baseValues full joint values used for joints a point does not store
   * @param {number} activeIndex highlighted point (-1 for none)
   * @param {boolean} loop draw the closing segment
   */
  update(points, chain, baseValues, activeIndex = -1, loop = false) {
    this.clear();
    if (!chain || !points.length) return;
    const scale = Math.max(0.5, Math.min(2.5, chain.reach() / 1.2));
    const T = new Matrix4();
    const pos = new Vector3();
    const quat = new Quaternion();
    const scl = new Vector3();
    const qOf = (p) => chain.getQ({ ...baseValues, ...p.joints });

    points.forEach((p, i) => {
      chain.fk(qOf(p), T);
      T.decompose(pos, quat, scl);
      const g = new Group();
      g.position.copy(pos);
      g.quaternion.copy(quat);
      const color = i === activeIndex ? COLORS.active : COLORS[p.move] || COLORS.J;
      const ball = new Mesh(this.sphere, new MeshBasicMaterial({ color, depthTest: false, transparent: true, opacity: 0.9 }));
      ball.renderOrder = 14;
      ball.scale.setScalar(scale);
      const axes = new AxesHelper(0.05 * scale);
      axes.material.depthTest = false;
      axes.material.transparent = true;
      axes.material.opacity = 0.8;
      axes.renderOrder = 14;
      const el = document.createElement('div');
      el.className = `wp-label${i === activeIndex ? ' active' : ''}`;
      el.textContent = p.name;
      if (!this.visible) el.style.display = 'none';
      const label = new CSS2DObject(el);
      label.position.set(0, 0, 0.035 * scale);
      label.center.set(0, 1);
      g.add(ball, axes, label);
      this.group.add(g);
    });

    // Path preview between consecutive points (segment i ends at point i).
    const segs = [];
    for (let i = 1; i < points.length; i++) segs.push([i - 1, i]);
    if (loop && points.length > 2) segs.push([points.length - 1, 0]);
    for (const [a, b] of segs) {
      const pa = points[a];
      const pb = points[b];
      const qa = qOf(pa);
      const qb = qOf(pb);
      const verts = [];
      const n = pb.move === 'L' ? 1 : 32;
      const q = new Float64Array(qa.length);
      for (let k = 0; k <= n; k++) {
        const s = pb.move === 'L' ? k / n : smoothstep(k / n);
        for (let i = 0; i < q.length; i++) q[i] = qa[i] + (qb[i] - qa[i]) * s;
        chain.fk(q, T);
        pos.setFromMatrixPosition(T);
        verts.push(pos.x, pos.y, pos.z);
      }
      const geo = new BufferGeometry();
      geo.setAttribute('position', new Float32BufferAttribute(verts, 3));
      const closing = b === 0 && a === points.length - 1;
      const mat = closing
        ? new LineDashedMaterial({ color: COLORS[pb.move], dashSize: 0.02, gapSize: 0.015, depthTest: false, transparent: true, opacity: 0.7 })
        : new LineBasicMaterial({ color: COLORS[pb.move], depthTest: false, transparent: true, opacity: 0.85 });
      const line = new Line(geo, mat);
      if (closing) line.computeLineDistances();
      line.renderOrder = 13;
      line.frustumCulled = false;
      this.group.add(line);
    }
  }
}
