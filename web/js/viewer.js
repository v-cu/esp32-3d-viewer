// viewer.js - shows an R3DM model the way the board draws it: per-vertex matcap uv from
// the view-space normal (u = 0.5 + 0.5 nx, v = 0.5 - 0.5 ny), texel x baked AO.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { parseR3DM } from './r3dm.js';

const VERT = /* glsl */`
  attribute float ao;
  varying vec2 vUv;
  varying float vAo;
  void main() {
    vec3 n = normalize(normalMatrix * normal);
    vUv = vec2(0.5 + 0.5 * n.x, 0.5 - 0.5 * n.y);
    vAo = ao;
    gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
  }`;
const FRAG = /* glsl */`
  uniform sampler2D matcap;
  varying vec2 vUv;
  varying float vAo;
  void main() {
    vec2 uv = clamp(vUv, vec2(0.004), vec2(0.996));
    gl_FragColor = vec4(texture2D(matcap, uv).rgb * vAo, 1.0);
  }`;

function matcapTexture(m) {
  const n = m.size * m.size, px = new Uint8Array(n * 4);
  for (let i = 0; i < n; i++) {
    const c = m.rgb565[i], r = c >> 11, g = (c >> 5) & 63, b = c & 31;
    px[4 * i] = (r << 3) | (r >> 2); px[4 * i + 1] = (g << 2) | (g >> 4); px[4 * i + 2] = (b << 3) | (b >> 2); px[4 * i + 3] = 255;
  }
  const t = new THREE.DataTexture(px, m.size, m.size, THREE.RGBAFormat);
  t.colorSpace = THREE.NoColorSpace;
  t.magFilter = THREE.LinearFilter;
  t.minFilter = THREE.LinearFilter;
  t.needsUpdate = true;
  return t;
}

// geometry + materials for one parsed model, centred like the device does (wheels on y = 0)
export function buildModelObject(model) {
  const group = new THREE.Group();
  const nm = model.mats.length;
  const byMat = Array.from({ length: nm }, () => []);
  for (let t = 0; t < model.numTriangles; t++) byMat[model.trimat[t]]?.push(t);
  model.mats.forEach((m, i) => {
    const tris = byMat[i];
    if (!tris.length) return;
    const idx = new Uint32Array(tris.length * 3);
    tris.forEach((t, k) => { idx[3 * k] = model.idx[3 * t]; idx[3 * k + 1] = model.idx[3 * t + 1]; idx[3 * k + 2] = model.idx[3 * t + 2]; });
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(model.pos, 3));
    g.setAttribute('normal', new THREE.BufferAttribute(model.nrm, 3));
    g.setAttribute('ao', new THREE.BufferAttribute(model.ao, 1));
    g.setIndex(new THREE.BufferAttribute(idx, 1));
    const mat = new THREE.ShaderMaterial({
      uniforms: { matcap: { value: matcapTexture(m) } }, vertexShader: VERT, fragmentShader: FRAG,
      side: m.dsided ? THREE.DoubleSide : THREE.FrontSide,
    });
    group.add(new THREE.Mesh(g, mat));
  });
  return group;
}

function disposeObject(o) {
  o.traverse((c) => {
    if (c.geometry) c.geometry.dispose();
    if (c.material) { c.material.uniforms?.matcap?.value?.dispose(); c.material.dispose(); }
  });
}

/*
 * new ModelViewer(canvasParent, { autoRotate, interactive })
 *   .show(bytes)   -> parsed model info
 */
export class ModelViewer {
  constructor(parent, opts = {}) {
    this.parent = parent;
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false, preserveDrawingBuffer: false });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.setClearColor(0x000000, 1);
    parent.appendChild(this.renderer.domElement);
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(30, 1, 0.01, 100);
    this.pivot = new THREE.Group();
    this.scene.add(this.pivot);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.enablePan = false;
    this.controls.autoRotate = opts.autoRotate !== false;
    this.controls.autoRotateSpeed = 2.4;
    this.controls.enabled = opts.interactive !== false;
    this.controls.minPolarAngle = 0.15;
    this.controls.maxPolarAngle = Math.PI * 0.62;
    let idle;
    this.controls.addEventListener('start', () => { this.controls.autoRotate = false; clearTimeout(idle); });
    this.controls.addEventListener('end', () => {
      clearTimeout(idle);
      idle = setTimeout(() => { this.controls.autoRotate = opts.autoRotate !== false; }, 6000);
    });
    this.obj = null;
    this.ro = new ResizeObserver(() => this.resize());
    this.ro.observe(parent);
    this.resize();
    this.visible = true;
    this.io = new IntersectionObserver((e) => { this.visible = e[0].isIntersecting; });
    this.io.observe(parent);
    const loop = () => {
      this.raf = requestAnimationFrame(loop);
      if (!this.visible || document.hidden) return;
      this.controls.update();
      this.renderer.render(this.scene, this.camera);
    };
    loop();
  }

  resize() {
    const w = this.parent.clientWidth || 1, h = this.parent.clientHeight || 1;
    this.renderer.setSize(w, h, false);
    this.renderer.domElement.style.width = '100%';
    this.renderer.domElement.style.height = '100%';
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  clear() {
    if (this.obj) { this.pivot.remove(this.obj); disposeObject(this.obj); this.obj = null; }
  }

  show(bytes) {
    const model = parseR3DM(bytes);
    this.clear();
    this.obj = buildModelObject(model);
    // look at the middle of the model; frame the bounding sphere
    const box = new THREE.Box3().setFromObject(this.obj);
    const c = box.getCenter(new THREE.Vector3());
    const r = box.getBoundingSphere(new THREE.Sphere()).radius || 1;
    this.obj.position.sub(c);
    this.pivot.add(this.obj);
    const dist = r / Math.sin(THREE.MathUtils.degToRad(this.camera.fov / 2)) * (this.camera.aspect < 1 ? 1.1 / this.camera.aspect : 0.86);
    // default view like the board: 3/4 from the front, slightly above
    const yaw = THREE.MathUtils.degToRad(35), pitch = THREE.MathUtils.degToRad(16);
    this.camera.position.set(Math.cos(pitch) * Math.cos(yaw) * dist, Math.sin(pitch) * dist, Math.cos(pitch) * Math.sin(yaw) * dist);
    this.controls.target.set(0, 0, 0);
    this.controls.minDistance = r * 1.2;
    this.controls.maxDistance = dist * 3;
    this.controls.update();
    return model;
  }

  dispose() {
    cancelAnimationFrame(this.raf);
    this.ro.disconnect();
    this.io.disconnect();
    this.clear();
    this.renderer.dispose();
    this.renderer.domElement.remove();
  }
}
