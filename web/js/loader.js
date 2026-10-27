// loader.js - reads .glb / .gltf / .obj (+ .mtl, .bin, textures) with three.js and turns
// the scene into the neutral part list the converter worker expects.
import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { DRACOLoader } from 'three/addons/loaders/DRACOLoader.js';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';
import { MTLLoader } from 'three/addons/loaders/MTLLoader.js';
import { MeshoptDecoder } from 'three/addons/libs/meshopt_decoder.module.js';

const MODEL_EXT = ['glb', 'gltf', 'obj'];
const ext = (n) => (n.split('.').pop() || '').toLowerCase();
const base = (u) => decodeURIComponent(u.split(/[?#]/)[0].split(/[\\/]/).pop() || '').toLowerCase();

export function isModelFile(name) { return MODEL_EXT.includes(ext(name)); }

const s2l = (c) => (c <= 0.04045 ? c / 12.92 : Math.pow((c + 0.055) / 1.055, 2.4));
const l2s = (c) => { c = Math.min(Math.max(c, 0), 1); return c <= 0.0031308 ? c * 12.92 : 1.055 * Math.pow(c, 1 / 2.4) - 0.055; };

// texture image -> {data, w, h} (max 1024 px, enough for colour regions)
const imageCache = new WeakMap();
function texturePixels(tex) {
  const img = tex && tex.image;
  if (!img || !(img.width > 0)) return null;
  if (imageCache.has(img)) return imageCache.get(img);
  let w = img.width, h = img.height;
  const s = Math.min(1, 1024 / Math.max(w, h));
  w = Math.max(1, Math.round(w * s)); h = Math.max(1, Math.round(h * s));
  let out = null;
  try {
    const cv = new OffscreenCanvas(w, h);
    const ctx = cv.getContext('2d', { willReadFrequently: true });
    ctx.drawImage(img, 0, 0, w, h);
    out = { data: ctx.getImageData(0, 0, w, h).data, w, h };
  } catch (e) {
    out = null;            // compressed (KTX2) or tainted image: no texture colours
  }
  imageCache.set(img, out);
  return out;
}

function materialInfo(mat, hasMtl) {
  const info = { name: (mat && mat.name) || 'default', color: [0.5, 0.5, 0.5], alpha: 1, metallic: 0, roughness: 0.5,
    doubleSided: false };
  if (!mat) return info;
  if (mat.color && (hasMtl !== false)) info.color = [mat.color.r, mat.color.g, mat.color.b];   // linear
  info.alpha = mat.opacity ?? 1;
  if (mat.transparent && info.alpha > 0.95) info.alpha = 0.5;
  if ((mat.transmission ?? 0) > 0.5) info.alpha = Math.min(info.alpha, 0.3);
  if (typeof mat.metalness === 'number') info.metallic = mat.metalness;
  if (typeof mat.roughness === 'number') info.roughness = mat.roughness;
  info.doubleSided = mat.side === THREE.DoubleSide;
  return info;
}

function sampler(tex, pix) {
  tex.updateMatrix();
  const m = tex.matrix.elements, flip = tex.flipY;
  return (u, v, out, o) => {
    const uu = m[0] * u + m[3] * v + m[6], vv = m[1] * u + m[4] * v + m[7];
    const fu = uu - Math.floor(uu), fv = vv - Math.floor(vv);
    const x = Math.min(pix.w - 1, Math.max(0, Math.floor(fu * pix.w)));
    const y = Math.min(pix.h - 1, Math.max(0, Math.floor((flip ? 1 - fv : fv) * pix.h)));
    const k = 4 * (y * pix.w + x);
    out[o] += pix.data[k] / 255; out[o + 1] += pix.data[k + 1] / 255; out[o + 2] += pix.data[k + 2] / 255;
  };
}

/*
 * One part per (mesh, material group): world-space positions, triangle indices,
 * material info and per-face sRGB colours (texture and / or vertex colours).
 */
function extractParts(root, hasMtl) {
  root.updateMatrixWorld(true);
  const parts = [];
  const v = new THREE.Vector3();
  root.traverse((obj) => {
    if (!obj.isMesh || !obj.geometry) return;
    const geo = obj.geometry;
    const pos = geo.attributes.position;
    if (!pos || pos.count < 3) return;
    if (geo.drawRange && geo.drawRange.count !== Infinity && geo.drawRange.count < 3) return;
    const nv = pos.count;
    const V = new Float32Array(nv * 3);
    for (let i = 0; i < nv; i++) {
      v.fromBufferAttribute(pos, i).applyMatrix4(obj.matrixWorld);
      V[3 * i] = v.x; V[3 * i + 1] = v.y; V[3 * i + 2] = v.z;
    }
    let I;
    if (geo.index) I = geo.index.array;
    else { I = new Uint32Array(nv); for (let i = 0; i < nv; i++) I[i] = i; }
    const mats = Array.isArray(obj.material) ? obj.material : [obj.material];
    const groups = geo.groups && geo.groups.length && Array.isArray(obj.material)
      ? geo.groups : [{ start: 0, count: I.length, materialIndex: 0 }];
    const flipWinding = obj.matrixWorld.determinant() < 0;
    const name = obj.name || (obj.parent && obj.parent.name) || 'part';
    for (const g of groups) {
      const mat = mats[g.materialIndex ?? 0] || mats[0];
      const start = Math.max(0, g.start), end = Math.min(I.length, g.start + g.count);
      const nf = Math.floor((end - start) / 3);
      if (nf < 1) continue;
      const F = new Uint32Array(nf * 3);
      for (let f = 0; f < nf; f++) {
        const a = I[start + 3 * f], b = I[start + 3 * f + 1], c = I[start + 3 * f + 2];
        F[3 * f] = a;
        F[3 * f + 1] = flipWinding ? c : b;
        F[3 * f + 2] = flipWinding ? b : c;
      }
      const info = materialInfo(mat, hasMtl);
      // per-face colours
      const tex = mat && mat.map;
      const pix = tex ? texturePixels(tex) : null;
      const uvAttr = tex ? (geo.attributes[tex.channel ? `uv${tex.channel}` : 'uv'] || geo.attributes.uv) : null;
      const col = geo.attributes.color && mat && mat.vertexColors ? geo.attributes.color : null;
      let faceCol = null;
      if ((pix && uvAttr) || col) {
        faceCol = new Float32Array(nf * 3);
        const samp = pix && uvAttr ? sampler(tex, pix) : null;
        const acc = new Float64Array(3);
        const bcf = info.color;
        for (let f = 0; f < nf; f++) {
          const a = F[3 * f], b = F[3 * f + 1], c = F[3 * f + 2];
          let lin = [1, 1, 1];
          if (samp) {
            acc.fill(0);
            const cu = (uvAttr.getX(a) + uvAttr.getX(b) + uvAttr.getX(c)) / 3;
            const cv = (uvAttr.getY(a) + uvAttr.getY(b) + uvAttr.getY(c)) / 3;
            samp(cu, cv, acc, 0);                         // centroid + half way to each corner
            for (const k of [a, b, c]) samp(0.5 * (cu + uvAttr.getX(k)), 0.5 * (cv + uvAttr.getY(k)), acc, 0);
            lin = [s2l(acc[0] / 4), s2l(acc[1] / 4), s2l(acc[2] / 4)];
          }
          if (col) {
            for (let ch = 0; ch < 3; ch++) {
              const g3 = ch === 0 ? 'getX' : ch === 1 ? 'getY' : 'getZ';
              lin[ch] *= (col[g3](a) + col[g3](b) + col[g3](c)) / 3;
            }
          }
          faceCol[3 * f] = l2s(lin[0] * bcf[0]); faceCol[3 * f + 1] = l2s(lin[1] * bcf[1]); faceCol[3 * f + 2] = l2s(lin[2] * bcf[2]);
        }
        // average colour of the material = mean face colour (linear)
        const m = [0, 0, 0];
        for (let f = 0; f < nf; f++) for (let ch = 0; ch < 3; ch++) m[ch] += s2l(faceCol[3 * f + ch]);
        info.color = m.map((x) => x / nf);
      } else if (tex && !pix) {
        // texture we can not read: keep the factor colour
      }
      parts.push({ name, mat: info, V, F, faceCol });
    }
  });
  return parts;
}

// compact each part's vertex buffer (a mesh with groups shares one big buffer)
function compactPart(p) {
  const nv = p.V.length / 3, remap = new Int32Array(nv).fill(-1);
  let n = 0;
  for (let i = 0; i < p.F.length; i++) {
    const v = p.F[i];
    if (v >= nv) throw new Error('index out of range in the model file');
    if (remap[v] < 0) remap[v] = n++;
    p.F[i] = remap[v];
  }
  let identity = n === nv;
  for (let i = 0; identity && i < nv; i++) if (remap[i] !== i) identity = false;
  if (identity) return p;
  const V = new Float32Array(n * 3);
  for (let i = 0; i < nv; i++) {
    const r = remap[i];
    if (r >= 0) { V[3 * r] = p.V[3 * i]; V[3 * r + 1] = p.V[3 * i + 1]; V[3 * r + 2] = p.V[3 * i + 2]; }
  }
  p.V = V;
  return p;
}

let dracoLoader = null;
function getDraco() {
  if (!dracoLoader) {
    dracoLoader = new DRACOLoader();
    dracoLoader.setDecoderPath(new URL('../vendor/three/addons/libs/draco/', import.meta.url).href);
  }
  return dracoLoader;
}

/*
 * files: File[] dropped together. Picks the first .glb/.gltf/.obj as the model; the other
 * files (.bin, .mtl, textures) are found by file name.
 * returns { name, parts, triangles, file }
 */
export async function loadModel(files) {
  const main = files.find((f) => isModelFile(f.name));
  if (!main) throw new Error('No .glb, .gltf or .obj file in the selection');
  const urls = new Map();
  const made = [];
  for (const f of files) {
    const u = URL.createObjectURL(f);
    made.push(u);
    urls.set(f.name.toLowerCase(), u);
  }
  const missing = new Set();
  const manager = new THREE.LoadingManager();
  manager.setURLModifier((url) => {
    if (/^(data|blob):/.test(url)) return url;
    const b = base(url);
    if (urls.has(b)) return urls.get(b);
    missing.add(b);
    return url;
  });
  const mainUrl = 'model/' + encodeURIComponent(main.name);
  try {
    const e = ext(main.name);
    let root, hasMtl = true;
    if (e === 'glb' || e === 'gltf') {
      const loader = new GLTFLoader(manager);
      loader.setDRACOLoader(getDraco());
      loader.setMeshoptDecoder(MeshoptDecoder);
      const gltf = await loader.loadAsync(mainUrl);
      root = gltf.scene || (gltf.scenes && gltf.scenes[0]);
    } else {
      const text = await main.text();
      const obj = new OBJLoader(manager);
      const mtlName = (text.match(/^\s*mtllib\s+(.+?)\s*$/m) || [])[1];
      const mtlFile = (mtlName && files.find((f) => f.name.toLowerCase() === base(mtlName)))
        || files.find((f) => ext(f.name) === 'mtl');
      if (mtlFile) {
        const mtl = new MTLLoader(manager);
        const mats = mtl.parse(await mtlFile.text(), '');
        mats.preload();
        obj.setMaterials(mats);
        // wait for the textures referenced by the .mtl
        await new Promise((res) => { if (manager.itemsLoaded >= manager.itemsTotal) res(); else manager.onLoad = res; setTimeout(res, 8000); });
      } else hasMtl = false;
      root = obj.parse(text);
    }
    if (!root) throw new Error('The file contains no scene');
    // textures load asynchronously for OBJ; give them a moment
    await new Promise((res) => setTimeout(res, 0));
    const parts = extractParts(root, hasMtl).map(compactPart);
    if (!parts.length) throw new Error('No triangle meshes found in the file');
    const triangles = parts.reduce((s, p) => s + p.F.length / 3, 0);
    const name = main.name.replace(/\.[^.]+$/, '');
    return { name, parts, triangles, file: main.name, missing: [...missing] };
  } finally {
    setTimeout(() => made.forEach((u) => URL.revokeObjectURL(u)), 30000);
  }
}
