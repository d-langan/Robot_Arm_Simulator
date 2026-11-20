// three.js viewport: Z-up world (ROS convention), orbit camera, lights, grid,
// on-demand rendering, CSS2D labels.

import {
  WebGLRenderer, Scene, PerspectiveCamera, Color, HemisphereLight, DirectionalLight,
  GridHelper, Mesh, PlaneGeometry, ShadowMaterial, Box3, Vector3, Group, AxesHelper,
  PCFSoftShadowMap, ACESFilmicToneMapping, SRGBColorSpace, Sphere,
} from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { CSS2DRenderer } from 'three/addons/renderers/CSS2DRenderer.js';

const THEMES = {
  dark: { bg: 0x16191e, grid1: 0x3a414c, grid2: 0x262b33, shadow: 0.35 },
  light: { bg: 0xe9edf2, grid1: 0x9aa4b2, grid2: 0xc9d0d9, shadow: 0.22 },
};

export class Viewer {
  constructor(container) {
    this.container = container;
    this.renderer = new WebGLRenderer({ antialias: true, preserveDrawingBuffer: false });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.outputColorSpace = SRGBColorSpace;
    this.renderer.toneMapping = ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.0;
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = PCFSoftShadowMap;
    container.appendChild(this.renderer.domElement);

    this.labels = new CSS2DRenderer();
    this.labels.domElement.className = 'label-layer';
    container.appendChild(this.labels.domElement);

    this.scene = new Scene();
    this.camera = new PerspectiveCamera(45, 1, 0.01, 200);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(2.2, -2.0, 1.6);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0, 0.5);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.12;
    this.controls.screenSpacePanning = true;
    this.controls.addEventListener('change', () => this.requestRender());
    this.controls.update();

    // Lights.
    const hemi = new HemisphereLight(0xffffff, 0x444a55, 1.6);
    hemi.position.set(0, 0, 1);
    this.scene.add(hemi);
    this.sun = new DirectionalLight(0xffffff, 2.2);
    this.sun.position.set(3, -2, 5);
    this.sun.castShadow = true;
    this.sun.shadow.mapSize.set(2048, 2048);
    this.sun.shadow.bias = -0.0004;
    this.sun.shadow.normalBias = 0.02;
    this.scene.add(this.sun, this.sun.target);
    const fill = new DirectionalLight(0xffffff, 0.6);
    fill.position.set(-3, 2, 2);
    this.scene.add(fill);

    // Ground.
    this.world = new Group();
    this.world.name = 'world';
    this.scene.add(this.world);
    this.grid = new GridHelper(10, 50);
    this.grid.rotation.x = Math.PI / 2;
    this.grid.material.transparent = true;
    this.grid.material.opacity = 0.9;
    this.scene.add(this.grid);
    this.ground = new Mesh(new PlaneGeometry(40, 40), new ShadowMaterial({ opacity: 0.3 }));
    this.ground.receiveShadow = true;
    this.ground.position.z = -0.0005;
    this.scene.add(this.ground);
    this.worldAxes = new AxesHelper(0.25);
    this.worldAxes.material.depthTest = false;
    this.worldAxes.renderOrder = 5;
    this.scene.add(this.worldAxes);

    this._dirty = true;
    this._frameCallbacks = new Set();
    this._lastTime = performance.now();

    this.setTheme(document.documentElement.dataset.theme === 'dark' ? 'dark' : 'light');

    this._resizeObserver = new ResizeObserver(() => this.resize());
    this._resizeObserver.observe(container);
    this.resize();

    const loop = (t) => {
      requestAnimationFrame(loop);
      const dt = Math.min(0.1, (t - this._lastTime) / 1000);
      this._lastTime = t;
      for (const cb of this._frameCallbacks) {
        if (cb(dt) === true) this._dirty = true;
      }
      this.controls.update();
      if (this._dirty) {
        this._dirty = false;
        this.renderer.render(this.scene, this.camera);
        this.labels.render(this.scene, this.camera);
      }
    };
    requestAnimationFrame(loop);
  }

  setTheme(name) {
    const t = THEMES[name] || THEMES.light;
    this.theme = name;
    this.scene.background = new Color(t.bg);
    const colors = this.grid.geometry.getAttribute('color');
    const c1 = new Color(t.grid1);
    const c2 = new Color(t.grid2);
    // GridHelper colors: centre lines use color1, others color2. Rebuild by vertex.
    const center = 50 / 2;
    for (let i = 0, k = 0; i <= 50; i++) {
      const c = i === center ? c1 : c2;
      for (let v = 0; v < 4; v++, k++) colors.setXYZ(k, c.r, c.g, c.b);
    }
    colors.needsUpdate = true;
    this.ground.material.opacity = t.shadow;
    this.requestRender();
  }

  /** Register a per-frame callback (dt seconds). Return true from it to request a render. */
  onFrame(cb) {
    this._frameCallbacks.add(cb);
    return () => this._frameCallbacks.delete(cb);
  }

  requestRender() {
    this._dirty = true;
  }

  resize() {
    const w = Math.max(1, this.container.clientWidth);
    const h = Math.max(1, this.container.clientHeight);
    this.renderer.setSize(w, h, false);
    this.renderer.domElement.style.width = '100%';
    this.renderer.domElement.style.height = '100%';
    this.labels.setSize(w, h);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.requestRender();
  }

  /** Frame an object: orbit target at its bounding-box centre, distance from its size. */
  fitTo(object, { keepDirection = true } = {}) {
    object.updateWorldMatrix(true, true);
    const box = new Box3().setFromObject(object, true);
    if (box.isEmpty()) return;
    const sphere = box.getBoundingSphere(new Sphere());
    const r = Math.max(sphere.radius, 0.1);
    const fov = (this.camera.fov * Math.PI) / 180;
    const dist = (r / Math.sin(fov / 2)) * 1.05;
    const dir = keepDirection
      ? new Vector3().subVectors(this.camera.position, this.controls.target).normalize()
      : new Vector3(1, -0.9, 0.7).normalize();
    if (!Number.isFinite(dir.x) || dir.lengthSq() < 1e-6) dir.set(1, -0.9, 0.7).normalize();
    this.controls.target.copy(sphere.center);
    this.camera.position.copy(sphere.center).addScaledVector(dir, dist);
    this.camera.near = Math.max(0.001, dist / 500);
    this.camera.far = dist * 50;
    this.camera.updateProjectionMatrix();
    this.controls.update();

    // Shadow camera and grid follow the robot size.
    const s = this.sun.shadow.camera;
    const ext = Math.max(r * 2.5, 1);
    s.left = -ext; s.right = ext; s.top = ext; s.bottom = -ext;
    s.near = 0.1; s.far = ext * 8;
    s.updateProjectionMatrix();
    this.sun.position.set(sphere.center.x + ext * 0.8, sphere.center.y - ext * 0.6, sphere.center.z + ext * 1.6);
    this.sun.target.position.copy(sphere.center);
    const gridScale = Math.max(0.2, Math.pow(10, Math.round(Math.log10(r * 4))) / 10);
    this.grid.scale.setScalar(gridScale);
    this.worldAxes.scale.setScalar(Math.max(0.3, r * 0.6));
    this.requestRender();
  }
}
