// Geometry construction for URDF <visual>/<collision> elements.
// Primitives are built directly; meshes are fetched through an asset resolver
// and decoded by extension. Decoded meshes are cached per resolved path and
// cloned per use (geometry/material buffers are shared).

import {
  BoxGeometry, SphereGeometry, CylinderGeometry, CapsuleGeometry,
  Mesh, Group, MeshStandardMaterial, Color, LoadingManager, TextureLoader, SRGBColorSpace,
} from 'three';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { ColladaLoader } from 'three/addons/loaders/ColladaLoader.js';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { PLYLoader } from 'three/addons/loaders/PLYLoader.js';
import { extname, AssetNotFoundError } from './assets.js';

export const SUPPORTED_MESH_EXTENSIONS = ['stl', 'dae', 'obj', 'glb', 'gltf', 'ply'];

const DEFAULT_COLOR = new Color(0.72, 0.74, 0.78);

export function makeUrdfMaterial(material, resolver, manager) {
  const mat = new MeshStandardMaterial({ color: DEFAULT_COLOR.clone(), metalness: 0.15, roughness: 0.55 });
  if (material && material.color) {
    const [r, g, b, a] = material.color;
    mat.color.setRGB(r, g, b, SRGBColorSpace);
    if (a < 1) {
      mat.transparent = true;
      mat.opacity = a;
      mat.depthWrite = a > 0.5;
    }
  }
  if (material && material.texture && resolver) {
    resolver.fetch(material.texture).then(({ data, path }) => {
      const blobUrl = URL.createObjectURL(new Blob([data]));
      new TextureLoader(manager).load(blobUrl, (tex) => {
        tex.colorSpace = SRGBColorSpace;
        mat.map = tex;
        mat.color.set(0xffffff);
        mat.needsUpdate = true;
        URL.revokeObjectURL(blobUrl);
      }, undefined, () => URL.revokeObjectURL(blobUrl));
      return path;
    }).catch(() => { /* texture missing: keep color */ });
  }
  return mat;
}

/** Build a primitive geometry object (URDF cylinders/capsules are along Z; three.js builds them along Y). */
export function makePrimitive(geometry, material) {
  let geo;
  switch (geometry.type) {
    case 'box':
      geo = new BoxGeometry(...geometry.size);
      break;
    case 'sphere':
      geo = new SphereGeometry(geometry.radius, 32, 16);
      break;
    case 'cylinder':
      geo = new CylinderGeometry(geometry.radius, geometry.radius, geometry.length, 32);
      geo.rotateX(Math.PI / 2);
      break;
    case 'capsule':
      geo = new CapsuleGeometry(geometry.radius, geometry.length, 8, 24);
      geo.rotateX(Math.PI / 2);
      break;
    default:
      return null;
  }
  return new Mesh(geo, material);
}

export class MeshLoader {
  /**
   * @param {UrlResolver|FileResolver} resolver
   */
  constructor(resolver) {
    this.resolver = resolver;
    this.manager = new LoadingManager();
    this.manager.setURLModifier((url) => resolver.modifyUrl(url));
    this.cache = new Map();   // resolved path -> Promise<{ kind, value }>
    this.missing = new Set(); // refs that could not be resolved
    this.failed = new Map();  // ref -> error message (resolved but undecodable)
  }

  /**
   * Load a URDF mesh geometry as an Object3D. `material` is the URDF material
   * (used for formats without their own materials: STL, OBJ, PLY).
   * Resolves to null when the file is missing or unsupported (recorded in missing/failed).
   */
  async load(geometry, material) {
    const ref = geometry.filename;
    const ext = extname(ref);
    if (!SUPPORTED_MESH_EXTENSIONS.includes(ext)) {
      this.failed.set(ref, `unsupported mesh format ".${ext}"`);
      return null;
    }
    let asset;
    try {
      asset = await this.resolver.fetch(ref);
    } catch (e) {
      if (e instanceof AssetNotFoundError) this.missing.add(ref);
      else this.failed.set(ref, e.message);
      return null;
    }
    let entry = this.cache.get(asset.path);
    if (!entry) {
      entry = this.decode(ext, asset).catch((e) => {
        this.failed.set(ref, e.message || String(e));
        return null;
      });
      this.cache.set(asset.path, entry);
    }
    const decoded = await entry;
    if (!decoded) return null;

    if (decoded.kind === 'geometry') {
      const mesh = new Mesh(decoded.value, makeUrdfMaterial(material, this.resolver, this.manager));
      if (decoded.value.hasAttribute('color')) mesh.material.vertexColors = true;
      return mesh;
    }
    // Object (DAE / glTF / OBJ): clone shares geometry & materials.
    const obj = decoded.value.clone(true);
    if (decoded.applyUrdfMaterial && material && material.color) {
      const mat = makeUrdfMaterial(material, this.resolver, this.manager);
      obj.traverse((c) => { if (c.isMesh) c.material = mat; });
    }
    return obj;
  }

  async decode(ext, { data, base }) {
    switch (ext) {
      case 'stl': {
        const geo = new STLLoader().parse(data);
        if (!geo.hasAttribute('normal')) geo.computeVertexNormals();
        return { kind: 'geometry', value: geo };
      }
      case 'ply': {
        const geo = new PLYLoader().parse(data);
        if (!geo.hasAttribute('normal')) geo.computeVertexNormals();
        return { kind: 'geometry', value: geo };
      }
      case 'dae': {
        const text = new TextDecoder().decode(data);
        const loader = new ColladaLoader(this.manager);
        const silence = console.warn;
        let dae;
        try {
          // ColladaLoader warns on every Z_UP file; we handle the axis ourselves below.
          console.warn = (...a) => { if (!String(a[0]).includes('Z-UP')) silence(...a); };
          dae = loader.parse(text, base);
        } finally {
          console.warn = silence;
        }
        if (!dae || !dae.scene) throw new Error('COLLADA parse failed');
        const scene = dae.scene;
        // ROS tools use COLLADA vertex data as-is in the link frame and ignore
        // <up_axis>; URDFs are authored against that. ColladaLoader rotates Z_UP
        // assets into three.js Y-up, so undo it (unit scaling is kept). Checked on
        // Baxter: its Y_UP torso visual matches its Z_UP collision mesh to 1 mm only
        // when the tag is ignored (605 mm off when Y_UP is rotated to Z-up).
        scene.rotation.set(0, 0, 0);
        stripLightsAndCameras(scene);
        return { kind: 'object', value: scene, applyUrdfMaterial: false };
      }
      case 'obj': {
        const text = new TextDecoder().decode(data);
        const group = new OBJLoader(this.manager).parse(text);
        const mat = new MeshStandardMaterial({ color: DEFAULT_COLOR.clone(), metalness: 0.15, roughness: 0.55 });
        group.traverse((c) => { if (c.isMesh) c.material = mat; });
        return { kind: 'object', value: group, applyUrdfMaterial: true };
      }
      case 'glb':
      case 'gltf': {
        const gltf = await new Promise((resolve, reject) => {
          new GLTFLoader(this.manager).parse(data, base, resolve, reject);
        });
        // Vertex data as-is, same as COLLADA above (assimp-based ROS tools do not
        // re-orient glTF either; URDF authors compensate in <origin rpy>).
        const g = new Group();
        g.add(gltf.scene);
        stripLightsAndCameras(g);
        return { kind: 'object', value: g, applyUrdfMaterial: false };
      }
      default:
        throw new Error(`unsupported mesh format ".${ext}"`);
    }
  }

  dispose() {
    this.resolver.dispose();
  }
}

function stripLightsAndCameras(root) {
  const remove = [];
  root.traverse((c) => { if (c.isLight || c.isCamera) remove.push(c); });
  for (const c of remove) c.parent && c.parent.remove(c);
}
