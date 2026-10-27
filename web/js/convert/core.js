// core.js - any 3D model -> R3DM, a faithful port of tools/model_convert.py to JavaScript.
// Runs in a Web Worker (no DOM). Input is a neutral list of parts produced by ../loader.js.
import { MeshoptSimplifier } from '../../vendor/meshoptimizer/meshopt_simplifier.js';
import {
  LUM, srgbToLin, linToSrgb, linToSrgb1, srgbToLin1, dot3, ptp3, srgbToOklab, rng, PointGrid, IntKeyMap,
  faceAdjacency, UnionFind, faceAreas, subMesh, compactMesh, weld, dedupeFaces, fixWinding, faceComponents,
  concatF64,
} from './util.js';
import { visibleTriangles, bakeAO } from './raster.js';
import { CLS, CLS_NAMES, renderMatcap } from './matcap.js';
import { writeR3DM } from '../r3dm.js';

export { CLS, CLS_NAMES };
const CLS_DROP = -1;

const CLS_IMPORTANCE = {
  [CLS.PAINT]: 2.6, [CLS.GLASS]: 1.1, [CLS.CHROME]: 1.0, [CLS.METAL]: 0.6, [CLS.TIRE]: 0.55,
  [CLS.PLASTIC]: 0.55, [CLS.LIGHT_RED]: 1.1, [CLS.LIGHT_CLEAR]: 1.2, [CLS.INTERIOR]: 0.3,
  [CLS.CARBON]: 0.8, [CLS.GENERIC]: 0.9,
};
const CLS_MATCAP = { [CLS.PAINT]: 128, [CLS.GLASS]: 128, [CLS.CHROME]: 128 };

// --------------------------------------------------------------------------------------
// colour predicates
// --------------------------------------------------------------------------------------
const isReddish = (c) => c[0] > 2.2 * Math.max(c[1], c[2]) && c[0] > 0.08;
const isAmber = (c) => c[0] > 0.15 && c[1] > 0.3 * c[0] && c[1] < 0.85 * c[0] && c[2] < 0.45 * c[1];
const isRed = (c) => c[0] > 0.15 && c[0] > 1.8 * c[1] && (c[2] > 0.4 * c[1] || c[1] < 0.04);

// --------------------------------------------------------------------------------------
// classification by names / PBR values
// --------------------------------------------------------------------------------------
const RULES = [
  [CLS.METAL, /hub|disk|disc|caliper|felge|(?<![a-z])rims?(?![a-z])|brake(?!.*(light|lamp|led|lens))/],
  [CLS.INTERIOR, /interior|leather|seat|carpet|dashboard|dash|steering|cockpit|fabric|alcantara|inner|innen/],
  [CLS.LIGHT_RED, /(tail|brake|stop).*(light|lamp|led|lens)|(rear|back).*(light|lamp|led)|(light|lamp|led|lens)[ _-]?red|red[ _-]?(light|lamp|led|lens)|rueckleucht|tylne|taillight/],
  [CLS.LIGHT_CLEAR, /head.?light|headlamp|projector|front.?light|lamp|lens|light|led|indicator|turn.?signal|reflector|scheinwerfer|reflektor/],
  [CLS.GLASS, /glass|window|windshield|windscreen|scheibe|szyb|glas/],
  [CLS.CHROME, /chrome|chrom|mirror_refl|polished/],
  [CLS.TIRE, /tire|tyre|rubber|reifen|opon|guma|llanta/],
  [CLS.CARBON, /carbon/],
  [CLS.METAL, /rim|wheel|alloy|felge|disc|disk|brake|caliper|metal|steel|alu|exhaust|felg/],
  [CLS.PAINT, /paint|body|carpaint|car_paint|lack|karoser|exterior|coat/],
  [CLS.PLASTIC, /plastic|trim|black|grill|grille|bumper_low|matte|kunststoff/],
];

export function classify(part) {
  const name = (part.mat.name + ' ' + part.name).toLowerCase();
  const mi = part.mat;
  for (const [cls, rx] of RULES) {
    if (rx.test(name)) {
      if ((cls === CLS.LIGHT_CLEAR || cls === CLS.GLASS) && isReddish(linToSrgb(mi.color))) return CLS.LIGHT_RED;
      return cls;
    }
  }
  if (mi.alpha < 0.7) return CLS.GLASS;
  if (mi.metallic > 0.8 && mi.roughness < 0.25) return CLS.CHROME;
  if (mi.metallic > 0.6) return CLS.METAL;
  if (dot3(mi.color, LUM) < 0.02) return CLS.PLASTIC;
  return CLS.GENERIC;
}

// --------------------------------------------------------------------------------------
// texture colour regions
// --------------------------------------------------------------------------------------
function faceColor(p, f) {
  return [p.faceCol[3 * f], p.faceCol[3 * f + 1], p.faceCol[3 * f + 2]];
}

// k-means++ (like scipy kmeans2 minit="++", 10 iterations)
function kmeans(X, n, dim, k, rand) {
  const cent = new Float64Array(k * dim);
  const d2 = new Float64Array(n).fill(Infinity);
  let first = Math.floor(rand() * n);
  for (let c = 0; c < dim; c++) cent[c] = X[first * dim + c];
  for (let j = 1; j < k; j++) {
    let sum = 0;
    for (let i = 0; i < n; i++) {
      let d = 0;
      for (let c = 0; c < dim; c++) { const e = X[i * dim + c] - cent[(j - 1) * dim + c]; d += e * e; }
      if (d < d2[i]) d2[i] = d;
      sum += d2[i];
    }
    let r = rand() * sum, pick = n - 1;
    for (let i = 0; i < n; i++) { r -= d2[i]; if (r <= 0) { pick = i; break; } }
    for (let c = 0; c < dim; c++) cent[j * dim + c] = X[pick * dim + c];
  }
  const lab = new Int32Array(n), acc = new Float64Array(k * dim), cnt = new Int32Array(k);
  for (let it = 0; it < 10; it++) {
    acc.fill(0); cnt.fill(0);
    for (let i = 0; i < n; i++) {
      let best = 0, bd = Infinity;
      for (let j = 0; j < k; j++) {
        let d = 0;
        for (let c = 0; c < dim; c++) { const e = X[i * dim + c] - cent[j * dim + c]; d += e * e; }
        if (d < bd) { bd = d; best = j; }
      }
      lab[i] = best; cnt[best]++;
      for (let c = 0; c < dim; c++) acc[best * dim + c] += X[i * dim + c];
    }
    for (let j = 0; j < k; j++) if (cnt[j]) for (let c = 0; c < dim; c++) cent[j * dim + c] = acc[j * dim + c] / cnt[j];
  }
  const out = [];
  for (let j = 0; j < k; j++) out.push(Array.from(cent.subarray(j * dim, j * dim + dim)));
  return out;
}

// cluster the per-face texture colours into regions -> {lbl, colors(sRGB)} or null
function splitByTexture(p, maxColors) {
  if (!p.faceCol) return null;
  const F = p.F, V = p.V, nf = F.length / 3;
  const area = faceAreas(V, F);
  const lab = new Float64Array(nf * 3);
  for (let f = 0; f < nf; f++) {
    srgbToOklab(p.faceCol[3 * f], p.faceCol[3 * f + 1], p.faceCol[3 * f + 2], lab, 3 * f);
    lab[3 * f + 1] *= 1.6; lab[3 * f + 2] *= 1.6;
  }
  const rand = rng(1);
  const nFit = Math.min(nf, 40000);
  // area weighted sampling with replacement
  const cdf = new Float64Array(nf);
  let s = 0;
  for (let f = 0; f < nf; f++) { s += area[f]; cdf[f] = s; }
  const sample = new Float64Array(nFit * 3);
  for (let i = 0; i < nFit; i++) {
    const r = rand() * s;
    let lo = 0, hi = nf - 1;
    while (lo < hi) { const mid = (lo + hi) >> 1; if (cdf[mid] < r) lo = mid + 1; else hi = mid; }
    sample[3 * i] = lab[3 * lo]; sample[3 * i + 1] = lab[3 * lo + 1]; sample[3 * i + 2] = lab[3 * lo + 2];
  }
  const k = Math.max(2, maxColors);
  let cent = kmeans(sample, nFit, 3, Math.min(k, nFit), rand);
  const keep = [];
  for (const c of cent) if (keep.every((q) => Math.hypot(c[0] - q[0], c[1] - q[1], c[2] - q[2]) > 0.06)) keep.push(c);
  cent = keep;
  const K = cent.length;
  let lbl = new Int32Array(nf);
  for (let f = 0; f < nf; f++) {
    let best = 0, bd = Infinity;
    for (let j = 0; j < K; j++) {
      const e0 = lab[3 * f] - cent[j][0], e1 = lab[3 * f + 1] - cent[j][1], e2 = lab[3 * f + 2] - cent[j][2];
      const d = e0 * e0 + e1 * e1 + e2 * e2;
      if (d < bd) { bd = d; best = j; }
    }
    lbl[f] = best;
  }
  // adjacency over position-welded vertices (UV seams must not break it)
  const w = weldIndexOnly(V, F, 1e-6);
  const adj = faceAdjacency(w.F, w.nv);
  const cnt = new Float64Array(nf * K);
  for (let it = 0; it < 4; it++) {
    cnt.fill(0);
    for (let i = 0; i < adj.length; i += 2) {
      const a = adj[i], b = adj[i + 1];
      cnt[a * K + lbl[b]] += 1; cnt[b * K + lbl[a]] += 1;
    }
    const nl = new Int32Array(nf);
    for (let f = 0; f < nf; f++) {
      cnt[f * K + lbl[f]] += 1.5;
      let best = 0, bv = -1;
      for (let j = 0; j < K; j++) if (cnt[f * K + j] > bv) { bv = cnt[f * K + j]; best = j; }
      nl[f] = best;
    }
    lbl = nl;
  }
  // absorb tiny islands into their neighbours
  let totA = 0;
  for (let f = 0; f < nf; f++) totA += area[f];
  const minArea = totA * 0.0015;
  for (let it = 0; it < 3; it++) {
    const uf = new UnionFind(nf);
    for (let i = 0; i < adj.length; i += 2) if (lbl[adj[i]] === lbl[adj[i + 1]]) uf.union(adj[i], adj[i + 1]);
    const comp = uf.labels();
    const compArea = new Float64Array(uf.count);
    for (let f = 0; f < nf; f++) compArea[comp[f]] += area[f];
    let anySmall = false;
    for (let f = 0; f < nf; f++) if (compArea[comp[f]] < minArea) { anySmall = true; break; }
    if (!anySmall) break;
    cnt.fill(0);
    for (let i = 0; i < adj.length; i += 2) {
      const a = adj[i], b = adj[i + 1];
      if (lbl[a] === lbl[b]) continue;
      cnt[a * K + lbl[b]] += area[b]; cnt[b * K + lbl[a]] += area[a];
    }
    // per small component: majority (by face count) of its faces' best neighbour labels
    const votes = new Map();
    for (let f = 0; f < nf; f++) {
      if (!(compArea[comp[f]] < minArea)) continue;
      let best = 0, bv = 0;
      for (let j = 0; j < K; j++) if (cnt[f * K + j] > bv) { bv = cnt[f * K + j]; best = j; }
      if (bv <= 0) continue;
      let v = votes.get(comp[f]);
      if (!v) { v = new Int32Array(K); votes.set(comp[f], v); }
      v[best]++;
    }
    const target = new Map();
    for (const [c, v] of votes) {
      let best = 0;
      for (let j = 1; j < K; j++) if (v[j] > v[best]) best = j;
      target.set(c, best);
    }
    for (let f = 0; f < nf; f++) if (target.has(comp[f])) lbl[f] = target.get(comp[f]);
  }
  // compact labels (sorted like np.unique)
  const used = [...new Set(lbl)].sort((a, b) => a - b);
  const remap = new Int32Array(K).fill(-1);
  used.forEach((u, i) => { remap[u] = i; });
  for (let f = 0; f < nf; f++) lbl[f] = remap[lbl[f]];
  const colors = used.map((_, r) => {
    const acc = [0, 0, 0];
    let ws = 0;
    for (let f = 0; f < nf; f++) {
      if (lbl[f] !== r) continue;
      const wgt = area[f] + 1e-12;
      for (let c = 0; c < 3; c++) acc[c] += srgbToLin1(p.faceCol[3 * f + c]) * wgt;
      ws += wgt;
    }
    return acc.map((v) => linToSrgb1(v / ws));
  });
  return { lbl, colors };
}

function weldIndexOnly(V, F, tol) {
  // face-aligned index buffer over position-welded vertices (no faces dropped)
  const nv = V.length / 3, map = new IntKeyMap(3, nv), key = new Int32Array(3), id = new Int32Array(nv);
  const inv = 1 / tol;
  for (let i = 0; i < nv; i++) {
    key[0] = Math.round(V[3 * i] * inv); key[1] = Math.round(V[3 * i + 1] * inv); key[2] = Math.round(V[3 * i + 2] * inv);
    id[i] = map.getOrAdd(key, 0);
  }
  const F2 = new Uint32Array(F.length);
  for (let i = 0; i < F.length; i++) F2[i] = id[F[i]];
  return { F: F2, nv: map.size };
}

function lightRegionClass(c, cx) {
  const lum = dot3(c, LUM), sat = ptp3(c);
  const rear = cx < -0.25;
  if (isAmber(c) && !rear) return [CLS.LIGHT_CLEAR, [1.0, 0.55, 0.08]];
  if (rear) {
    if (lum < 0.12 && sat < 0.08) return [CLS.PLASTIC, c];
    if (lum > 0.5 && sat < 0.12) return [CLS.LIGHT_CLEAR, [1, 1, 1]];
    return [CLS.LIGHT_RED, c];
  }
  if (isReddish(c)) return [CLS.LIGHT_RED, c];
  if (lum < 0.06) return [CLS.PLASTIC, c];
  return [CLS.LIGHT_CLEAR, [1, 1, 1]];
}

function atlasRegionClass(c, cx) {
  const lum = dot3(c, LUM), sat = ptp3(c);
  const rear = cx < -0.55, front = cx > 0.55;
  if (isRed(c)) return [rear ? CLS.LIGHT_RED : CLS.GENERIC, c];
  if (isAmber(c)) return (front || rear) ? [CLS.LIGHT_CLEAR, [1.0, 0.55, 0.08]] : [CLS.GENERIC, c];
  if (sat < 0.12) {
    if (lum > 0.6) return front ? [CLS.LIGHT_CLEAR, [1, 1, 1]] : [CLS.CHROME, c];
    if (lum > 0.3) return [CLS.METAL, c];
    return [null, c];
  }
  return [CLS.GENERIC, c];
}

function isAtlas(p) {
  const nf = p.F.length / 3;
  if (nf < 1500 || !p.faceCol) return false;
  let s = 0, s2 = 0, sat = 0;
  for (let f = 0; f < nf; f++) {
    const c = faceColor(p, f), l = dot3(c, LUM);
    s += l; s2 += l * l;
    if (ptp3(c) > 0.25) sat++;
  }
  const mean = s / nf, std = Math.sqrt(Math.max(s2 / nf - mean * mean, 0));
  return std > 0.12 || sat / nf > 0.02;
}

function applyTextureSplit(parts, isCar, maxColors, log) {
  if (maxColors <= 1) return parts;
  let carH = 1;
  if (isCar) {
    carH = -Infinity;
    for (const p of parts) for (let i = 1; i < p.V.length; i += 3) carH = Math.max(carH, p.V[i]);
  }
  const out = [];
  for (const p of parts) {
    let mode = null;
    if (p.faceCol) {
      if (!isCar) mode = 'object';
      else if (p.cls === CLS.LIGHT_RED || p.cls === CLS.LIGHT_CLEAR) mode = 'light';
      else if ([CLS.GENERIC, CLS.PLASTIC, CLS.PAINT, CLS.INTERIOR].includes(p.cls)) mode = isAtlas(p) ? 'atlas' : null;
    }
    let res = null;
    if (mode) {
      try {
        const k = mode === 'object' ? maxColors : mode === 'light' ? Math.min(maxColors, 6) : Math.min(maxColors, 10);
        res = splitByTexture(p, k);
      } catch (e) {
        log(`  texture split skipped for '${p.name}': ${e.message}`);
      }
    }
    const nf = p.F.length / 3;
    if (!res || res.colors.length < 2) { out.push(p); continue; }
    const { lbl, colors } = res;
    const cen = new Float64Array(nf * 3);
    for (let f = 0; f < nf; f++)
      for (let c = 0; c < 3; c++)
        cen[3 * f + c] = (p.V[3 * p.F[3 * f] + c] + p.V[3 * p.F[3 * f + 1] + c] + p.V[3 * p.F[3 * f + 2] + c]) / 3;
    const summary = [];
    colors.forEach((c, r) => {
      const fm = new Uint8Array(nf);
      let n = 0, sx = 0;
      for (let f = 0; f < nf; f++) if (lbl[f] === r) { fm[f] = 1; n++; sx += cen[3 * f]; }
      if (!n) return;
      const cx = sx / n;
      let pieces;
      if (mode === 'object') {
        const lum = dot3(srgbToLin(c), LUM);
        pieces = [[fm, lum < 0.02 ? CLS.PLASTIC : CLS.GENERIC, c]];
      } else if (mode === 'light') {
        const [cls, col] = lightRegionClass(c, cx);
        pieces = [[fm, cls, col]];
      } else {
        const [cls, col] = atlasRegionClass(c, cx);
        if (cls === null) {
          const hi = new Uint8Array(nf), lo = new Uint8Array(nf);
          for (let f = 0; f < nf; f++) if (fm[f]) { if (cen[3 * f + 1] > 0.58 * carH) hi[f] = 1; else lo[f] = 1; }
          pieces = [[hi, CLS.GLASS, col], [lo, CLS.PLASTIC, col]];
        } else pieces = [[fm, cls, col]];
      }
      pieces.forEach(([m, cls, col], j) => {
        let cntm = 0;
        for (let f = 0; f < nf; f++) cntm += m[f];
        if (!cntm) return;
        const sm = subMesh(p.V, p.F, m);
        const fc = new Float32Array(cntm * 3);
        let q = 0;
        for (let f = 0; f < nf; f++) if (m[f]) { fc[q++] = p.faceCol[3 * f]; fc[q++] = p.faceCol[3 * f + 1]; fc[q++] = p.faceCol[3 * f + 2]; }
        const mi = { ...p.mat, color: srgbToLin(col), name: `${p.mat.name}_c${r}${pieces.length > 1 ? 'abcd'[j] : ''}` };
        out.push({ name: `${p.name}_c${r}_${j}`, mat: mi, cls, V: sm.V, F: sm.F, faceCol: fc, fromSplit: true });
        summary.push(`${CLS_NAMES[cls]}:${cntm}`);
      });
    });
    log(`  '${p.name.slice(0, 40)}' (${mode}): ${colors.length} colour regions -> ${summary.join(' ')}`);
  }
  return out;
}

// --------------------------------------------------------------------------------------
// lamp lenses
// --------------------------------------------------------------------------------------
function dropLampCovers(parts, log, dist = 0.05) {
  const lights = parts.filter((p) => (p.cls === CLS.LIGHT_RED || p.cls === CLS.LIGHT_CLEAR) && p.F.length);
  if (!lights.length) return parts;
  const pts = [];
  for (const p of lights) {
    pts.push(p.V);
    const nf = p.F.length / 3, c = new Float64Array(nf * 3);
    for (let f = 0; f < nf; f++)
      for (let k = 0; k < 3; k++) c[3 * f + k] = (p.V[3 * p.F[3 * f] + k] + p.V[3 * p.F[3 * f + 1] + k] + p.V[3 * p.F[3 * f + 2] + k]) / 3;
    pts.push(c);
  }
  const grid = new PointGrid(concatF64(pts), dist);
  const out = [];
  let dropped = 0;
  for (const p of parts) {
    if (p.cls !== CLS.GLASS || !p.F.length) { out.push(p); continue; }
    const nf = p.F.length / 3, keep = new Uint8Array(nf);
    let nk = 0;
    for (let f = 0; f < nf; f++) {
      const x = (p.V[3 * p.F[3 * f]] + p.V[3 * p.F[3 * f + 1]] + p.V[3 * p.F[3 * f + 2]]) / 3;
      const y = (p.V[3 * p.F[3 * f] + 1] + p.V[3 * p.F[3 * f + 1] + 1] + p.V[3 * p.F[3 * f + 2] + 1]) / 3;
      const z = (p.V[3 * p.F[3 * f] + 2] + p.V[3 * p.F[3 * f + 1] + 2] + p.V[3 * p.F[3 * f + 2] + 2]) / 3;
      grid.nearest(x, y, z, dist);
      keep[f] = grid.lastDist > dist ? 1 : 0;
      nk += keep[f];
    }
    dropped += nf - nk;
    if (nk === nf) { out.push(p); continue; }
    if (nk) {
      const sm = subMesh(p.V, p.F, keep);
      out.push({ ...p, V: sm.V, F: sm.F, faceCol: null });
    }
  }
  if (dropped) log(`  removed ${dropped} lamp-lens glass triangles (lamps visible)`);
  return out;
}

// --------------------------------------------------------------------------------------
// orientation
// --------------------------------------------------------------------------------------
const matMul = (A, B) => A.map((r) => [0, 1, 2].map((j) => r[0] * B[0][j] + r[1] * B[1][j] + r[2] * B[2][j]));
function rotY(deg) {
  const a = deg * Math.PI / 180, c = Math.cos(a), s = Math.sin(a);
  return [[c, 0, s], [0, 1, 0], [-s, 0, c]];
}
function applyR(V, R) {
  const n = V.length / 3, out = new Float64Array(V.length);
  for (let i = 0; i < n; i++) {
    const x = V[3 * i], y = V[3 * i + 1], z = V[3 * i + 2];
    for (let k = 0; k < 3; k++) out[3 * i + k] = R[k][0] * x + R[k][1] * y + R[k][2] * z;
  }
  return out;
}

function lampFrontSign(parts, R) {
  const xr = [], wr = [], xw = [], ww = [];
  let any = false;
  for (const p of parts) {
    const atlas = [CLS.GENERIC, CLS.PLASTIC, CLS.PAINT].includes(p.cls);
    if (![CLS.LIGHT_RED, CLS.LIGHT_CLEAR, CLS.GLASS].includes(p.cls) && !atlas) continue;
    any = true;
    const V = applyR(p.V0, R), F = p.F, nf = F.length / 3;
    const area = faceAreas(V, F, false);
    let maxAbsX = 0;
    for (let i = 0; i < V.length; i += 3) maxAbsX = Math.max(maxAbsX, Math.abs(V[i]));
    const matCol = linToSrgb(p.mat.color);
    for (let f = 0; f < nf; f++) {
      const cx = (V[3 * F[3 * f]] + V[3 * F[3 * f + 1]] + V[3 * F[3 * f + 2]]) / 3;
      const c = p.faceCol ? faceColor(p, f) : matCol;
      const [r, g, b] = c;
      let red = r > 0.08 && r > 1.8 * g && (b > 0.4 * g || g < 0.04);
      let wht = dot3(c, LUM) > 0.5 && ptp3(c) < 0.12;
      if (p.cls === CLS.GLASS || atlas) wht = false;
      if (atlas) red = red && Math.abs(cx) > 0.6 * maxAbsX;
      if (red) { xr.push(cx); wr.push(area[f]); }
      if (wht) { xw.push(cx); ww.push(area[f]); }
    }
  }
  if (!any) return null;
  const sum = (a) => a.reduce((s, v) => s + v, 0);
  const avg = (x, w) => { let s = 0, t = 0; for (let i = 0; i < x.length; i++) { s += x[i] * w[i]; t += w[i]; } return s / t; };
  if (sum(wr) <= 0) return null;
  const mr = avg(xr, wr);
  if (sum(ww) <= 0) {
    let span = 1e-9;
    for (const x of xr) span = Math.max(span, Math.abs(x));
    return Math.abs(mr) < 0.3 * span ? null : (mr < 0 ? 1 : -1);
  }
  const mw = avg(xw, ww);
  if (Math.abs(mr - mw) < 1e-6) return null;
  return mw > mr ? 1 : -1;
}

function normaliseOrientation(parts, cfg) {
  const up = (cfg.up_axis || 'y').toLowerCase();
  let R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  if (up === 'z') R = [[1, 0, 0], [0, 0, 1], [0, -1, 0]];
  else if (up === '-z') R = [[1, 0, 0], [0, 0, -1], [0, 1, 0]];
  else if (up === 'x') R = [[0, 1, 0], [-1, 0, 0], [0, 0, 1]];
  const bounds = (R) => {
    const mn = [Infinity, Infinity, Infinity], mx = [-Infinity, -Infinity, -Infinity];
    for (const p of parts) {
      const V = p.V0, n = V.length / 3;
      for (let i = 0; i < n; i++) {
        const x = V[3 * i], y = V[3 * i + 1], z = V[3 * i + 2];
        for (let k = 0; k < 3; k++) {
          const v = R[k][0] * x + R[k][1] * y + R[k][2] * z;
          if (v < mn[k]) mn[k] = v;
          if (v > mx[k]) mx[k] = v;
        }
      }
    }
    return [mn, mx];
  };
  let [mn, mx] = bounds(R);
  const ext = [mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2]];
  const kind = cfg.kind || 'auto';
  const isCar = kind === 'car' || (kind === 'auto' && parts.some((p) => [CLS.TIRE, CLS.LIGHT_RED, CLS.LIGHT_CLEAR].includes(p.cls)));
  cfg._isCar = isCar;
  if (!isCar) R = matMul(rotY(90), R);
  else if (ext[2] > ext[0]) R = matMul(rotY(-90), R);
  if (cfg.yaw_deg !== undefined && cfg.yaw_deg !== null && cfg.yaw_deg !== '') R = matMul(rotY(Number(cfg.yaw_deg)), R);
  else if (isCar) {
    let side = lampFrontSign(parts, R);
    if (side === null) {
      const meanX = (cls) => {
        let s = 0, n = 0;
        for (const p of parts) if (p.cls === cls) {
          const V = applyR(p.V0, R);
          for (let i = 0; i < V.length; i += 3) { s += V[i]; n++; }
        }
        return n ? s / n : null;
      };
      const fr = meanX(CLS.LIGHT_CLEAR), rr = meanX(CLS.LIGHT_RED);
      if (fr !== null && rr !== null) side = fr > rr ? 1 : -1;
    }
    if (side !== null && side < 0) R = matMul(rotY(180), R);
  }
  if (cfg.flip_front) R = matMul(rotY(180), R);
  [mn, mx] = bounds(R);
  const center = [(mn[0] + mx[0]) / 2, mn[1], (mn[2] + mx[2]) / 2];
  const big = isCar ? mx[0] - mn[0] : Math.max(mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2]);
  const scale = 2.0 / Math.max(big, 1e-9);
  for (const p of parts) {
    const V = applyR(p.V0, R);
    for (let i = 0; i < V.length; i += 3) {
      V[i] = (V[i] - center[0]) * scale; V[i + 1] = (V[i + 1] - center[1]) * scale; V[i + 2] = (V[i + 2] - center[2]) * scale;
    }
    p.V = V;
  }
}

// --------------------------------------------------------------------------------------
// mesh processing
// --------------------------------------------------------------------------------------
function meshoptSimplify(V, F, target, lockBorder = false) {
  const pos = new Float32Array(V);
  const idx = F instanceof Uint32Array ? F : new Uint32Array(F);
  const tic = Math.min(idx.length, Math.max(0, Math.floor(target)) * 3);
  const [out] = MeshoptSimplifier.simplify(idx, pos, 3, tic, 1.0, lockBorder ? ['LockBorder'] : []);
  return out;
}

function decimate(V, F, target) {
  if (F.length / 3 <= target) return { V, F };
  const F2 = meshoptSimplify(V, F, target);
  return compactMesh(V, F2);
}

function creaseNormals(V, F, angleDeg) {
  const nv = V.length / 3, nf = F.length / 3;
  const fn = new Float64Array(nf * 3), fnu = new Float64Array(nf * 3), area = new Float64Array(nf);
  for (let f = 0; f < nf; f++) {
    const a = 3 * F[3 * f], b = 3 * F[3 * f + 1], c = 3 * F[3 * f + 2];
    const ux = V[b] - V[a], uy = V[b + 1] - V[a + 1], uz = V[b + 2] - V[a + 2];
    const vx = V[c] - V[a], vy = V[c + 1] - V[a + 1], vz = V[c + 2] - V[a + 2];
    const nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
    const l = Math.hypot(nx, ny, nz);
    fn[3 * f] = nx; fn[3 * f + 1] = ny; fn[3 * f + 2] = nz;
    area[f] = l;
    const il = 1 / Math.max(l, 1e-20);
    fnu[3 * f] = nx * il; fnu[3 * f + 1] = ny * il; fnu[3 * f + 2] = nz * il;
  }
  const cosT = Math.cos(angleDeg * Math.PI / 180);
  const start = new Int32Array(nv + 1);
  for (let i = 0; i < F.length; i++) start[F[i] + 1]++;
  for (let v = 0; v < nv; v++) start[v + 1] += start[v];
  const fill = start.slice(0, nv), corners = new Int32Array(F.length);
  for (let i = 0; i < F.length; i++) corners[fill[F[i]]++] = i;      // corner index = 3*f + slot
  const cn = new Float64Array(F.length * 3);
  for (let v = 0; v < nv; v++) {
    const s = start[v], e = start[v + 1];
    for (let a = s; a < e; a++) {
      const fa = (corners[a] / 3) | 0;
      let ax = 0, ay = 0, az = 0;
      for (let b = s; b < e; b++) {
        const fb = (corners[b] / 3) | 0;
        const d = fnu[3 * fa] * fnu[3 * fb] + fnu[3 * fa + 1] * fnu[3 * fb + 1] + fnu[3 * fa + 2] * fnu[3 * fb + 2];
        if (d >= cosT) {
          const w = area[fb];
          ax += w * fnu[3 * fb]; ay += w * fnu[3 * fb + 1]; az += w * fnu[3 * fb + 2];
        }
      }
      const l = Math.max(Math.hypot(ax, ay, az), 1e-20);
      const c = corners[a];
      cn[3 * c] = ax / l; cn[3 * c + 1] = ay / l; cn[3 * c + 2] = az / l;
    }
  }
  // dedupe (vertex, quantised normal)
  const map = new Map();
  const key = new Int32Array(4);
  const F2 = new Uint32Array(F.length);
  const Vout = [], Nout = [];
  for (let c = 0; c < F.length; c++) {
    key[0] = F[c];
    key[1] = Math.round(cn[3 * c] * 127); key[2] = Math.round(cn[3 * c + 1] * 127); key[3] = Math.round(cn[3 * c + 2] * 127);
    const k = key[0] * 16777216 + ((key[1] + 128) << 16) + ((key[2] + 128) << 8) + (key[3] + 128);
    let id = map.get(k);
    if (id === undefined) {
      id = Vout.length / 3;
      map.set(k, id);
      const v = F[c];
      Vout.push(V[3 * v], V[3 * v + 1], V[3 * v + 2]);
      Nout.push(cn[3 * c], cn[3 * c + 1], cn[3 * c + 2]);
    }
    F2[c] = id;
  }
  return { V: Float64Array.from(Vout), N: Float64Array.from(Nout), F: F2 };
}

function transferNormals(V, N, grid, hiN, maxAngleDeg = 28) {
  const n = V.length / 3, cosM = Math.cos(maxAngleDeg * Math.PI / 180);
  const out = Float64Array.from(N);
  let ok = 0;
  for (let i = 0; i < n; i++) {
    const j = grid.nearest(V[3 * i], V[3 * i + 1], V[3 * i + 2], 1e-5);
    if (j < 0 || !(grid.lastDist < 1e-5)) continue;
    const d = hiN[3 * j] * N[3 * i] + hiN[3 * j + 1] * N[3 * i + 1] + hiN[3 * j + 2] * N[3 * i + 2];
    if (d > cosM) { out[3 * i] = hiN[3 * j]; out[3 * i + 1] = hiN[3 * j + 1]; out[3 * i + 2] = hiN[3 * j + 2]; ok++; }
  }
  return out;
}

function parseColor(s) {
  s = String(s).trim().replace(/^#/, '');
  return srgbToLin([0, 2, 4].map((i) => parseInt(s.slice(i, i + 2), 16) / 255));
}

function joinParts(parts) {
  let nv = 0, nf = 0;
  for (const p of parts) { nv += p.V.length / 3; nf += p.F.length / 3; }
  const V = new Float64Array(nv * 3), F = new Uint32Array(nf * 3), owner = new Int32Array(nf);
  let vo = 0, fo = 0;
  parts.forEach((p, i) => {
    V.set(p.V, vo * 3);
    const pf = p.F.length / 3;
    for (let k = 0; k < p.F.length; k++) F[fo * 3 + k] = p.F[k] + vo;
    owner.fill(i, fo, fo + pf);
    vo += p.V.length / 3; fo += pf;
  });
  return { V, F, owner };
}

// --------------------------------------------------------------------------------------
// main pipeline
// --------------------------------------------------------------------------------------
/*
 * parts: [{ name, mat: {name, color[3] linear, alpha, metallic, roughness, doubleSided},
 *           V: Float32Array|Float64Array (world space), F: Uint32Array, faceCol: Float32Array|null (sRGB) }]
 * opts:  { name, budget, paint, texColors, keepHidden, views, crease, maxMaterials, noAO, config }
 * hooks: { log(msg), progress(stage, frac) }
 * returns { bytes: Uint8Array, report }
 */
export async function convert(inputParts, opts = {}, hooks = {}) {
  const log = hooks.log || (() => {});
  const progress = hooks.progress || (() => {});
  await MeshoptSimplifier.ready;
  const cfg = { ...(opts.config || {}) };
  const budget = Math.max(200, Math.floor(opts.budget || cfg.budget || 10000));
  const texColors = opts.texColors ?? 12;
  const crease = opts.crease ?? 42;
  const nViews = opts.views ?? 64;
  const maxMaterials = opts.maxMaterials ?? 15;

  let parts = inputParts.filter((p) => p.F && p.F.length >= 3).map((p) => ({
    name: p.name || 'part', mat: { ...p.mat }, V0: Float64Array.from(p.V), F: Uint32Array.from(p.F),
    faceCol: p.faceCol || null,
  }));
  if (!parts.length) throw new Error('No triangle meshes found in the file');
  log(`  ${parts.length} parts, ${parts.reduce((s, p) => s + p.F.length / 3, 0)} triangles`);
  progress('Classifying materials', 0);

  // ---- classify
  const overrides = {};
  for (const [k, v] of Object.entries(cfg.materials || {})) overrides[k.toLowerCase()] = v;
  for (const p of parts) {
    const ov = overrides[p.name.toLowerCase()] ?? overrides[p.mat.name.toLowerCase()];
    if (ov !== undefined && (ov === 'drop' || CLS_NAMES.includes(ov))) p.cls = ov === 'drop' ? CLS_DROP : CLS_NAMES.indexOf(ov);
    else p.cls = classify(p);
  }
  normaliseOrientation(parts, cfg);
  for (const p of parts) delete p.V0;
  parts = parts.filter((p) => p.cls !== CLS_DROP);
  log(`  mode: ${cfg._isCar ? 'car' : 'object'}`);
  parts = applyTextureSplit(parts, cfg._isCar, texColors, log);
  if (cfg._isCar) parts = dropLampCovers(parts, log);
  let ndup = 0;
  for (const p of parts) {
    const w = weld(p.V, p.F, 1e-6);
    const Fd = dedupeFaces(w.F);
    ndup += (w.F.length - Fd.length) / 3;
    fixWinding(Fd, w.V.length / 3);
    p.V = w.V; p.F = Fd; p.faceCol = null;
  }
  parts = parts.filter((p) => p.F.length);
  if (ndup) log(`  removed ${ndup} duplicate (double-sided) triangles`);

  // ---- global arrays
  let { V, F, owner } = joinParts(parts);
  let radius = 0;
  for (let i = 0; i < V.length; i += 3) radius = Math.max(radius, Math.hypot(V[i], V[i + 1], V[i + 2]));
  radius *= 1.02;
  let nf = F.length / 3;

  // ---- hidden surface removal
  let vis;
  if (!opts.keepHidden) {
    progress('Removing hidden geometry', 0);
    log(`  visibility: ${nf} tris, ${nViews} views @ 640px`);
    const r = visibleTriangles(V, F, radius, nViews, 640, -0.35, (f) => progress('Removing hidden geometry', f));
    vis = r.vis;
    const { labels, count } = faceComponents(F, V.length / 3);
    const score = new Float64Array(count);
    for (let f = 0; f < nf; f++) score[labels[f]] += r.back[f] - r.front[f];
    let nflip = 0;
    for (let f = 0; f < nf; f++) if (score[labels[f]] > 0) {
      const t = F[3 * f + 1]; F[3 * f + 1] = F[3 * f + 2]; F[3 * f + 2] = t; nflip++;
    }
    const vcnt = new Float64Array(parts.length), tcnt = new Float64Array(parts.length);
    for (let f = 0; f < nf; f++) {
      const mx = Math.max(r.front[f], r.back[f]), mn = Math.min(r.front[f], r.back[f]);
      if (vis[f]) vcnt[owner[f]]++;
      if (vis[f] && mn >= 0.3 * mx && mx >= 3) tcnt[owner[f]]++;
    }
    parts.forEach((p, i) => { p.twosided = tcnt[i] > 0.25 * Math.max(vcnt[i], 1); });
    let nv = 0;
    for (let f = 0; f < nf; f++) nv += vis[f];
    log(`  visible: ${nv} / ${nf} triangles, winding fixed on ${nflip}`);
  } else {
    vis = new Uint8Array(nf).fill(1);
    for (const p of parts) p.twosided = !!p.mat.doubleSided;
  }

  // high resolution smooth normals (area weighted), transferred to the final mesh
  progress('Simplifying', 0);
  const hiN = new Float64Array(V.length);
  for (let f = 0; f < nf; f++) {
    const a = 3 * F[3 * f], b = 3 * F[3 * f + 1], c = 3 * F[3 * f + 2];
    const ux = V[b] - V[a], uy = V[b + 1] - V[a + 1], uz = V[b + 2] - V[a + 2];
    const vx = V[c] - V[a], vy = V[c + 1] - V[a + 1], vz = V[c + 2] - V[a + 2];
    const nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
    for (const i of [a, b, c]) { hiN[i] += nx; hiN[i + 1] += ny; hiN[i + 2] += nz; }
  }
  for (let i = 0; i < hiN.length; i += 3) {
    const l = Math.max(Math.hypot(hiN[i], hiN[i + 1], hiN[i + 2]), 1e-20);
    hiN[i] /= l; hiN[i + 1] /= l; hiN[i + 2] /= l;
  }
  const hiGrid = new PointGrid(V, 1e-4);

  const areas = faceAreas(V, F);
  // ---- car paint: the biggest visible saturated surface (per material) wins
  const paintMatNames = new Set(parts.filter((p) => p.cls === CLS.PAINT).map((p) => p.mat.name.toLowerCase()));
  if (cfg._isCar && ![...paintMatNames].some((k) => k in overrides)) {
    const visArea = new Float64Array(parts.length);
    for (let f = 0; f < nf; f++) if (vis[f]) visArea[owner[f]] += areas[f];
    const score = new Map();
    parts.forEach((p, i) => {
      if (![CLS.GENERIC, CLS.PLASTIC, CLS.PAINT].includes(p.cls) || p.fromSplit) return;
      const sat = ptp3(linToSrgb(p.mat.color));
      let sc = visArea[i] * (0.25 + 1.5 * sat) * 0.5;
      if (p.cls === CLS.PAINT) sc *= 1.5;
      score.set(p.mat.name, (score.get(p.mat.name) || 0) + sc);
    });
    if (score.size) {
      let mname = null, best = -Infinity;
      for (const [k, v] of score) if (v > best) { best = v; mname = k; }
      for (const p of parts) {
        if (p.mat.name === mname) p.cls = CLS.PAINT;
        else if (p.cls === CLS.PAINT) p.cls = dot3(p.mat.color, LUM) < 0.02 ? CLS.PLASTIC : CLS.GENERIC;
      }
      log(`  paint: material '${mname}'`);
    }
  }

  const totArea = new Float64Array(parts.length), visArea0 = new Float64Array(parts.length);
  for (let f = 0; f < nf; f++) { totArea[owner[f]] += areas[f]; if (vis[f]) visArea0[owner[f]] += areas[f]; }

  // ---- drop hidden triangles, coarse stage-1 decimation
  parts.forEach((p, i) => {
    const m = new Uint8Array(nf);
    for (let f = 0; f < nf; f++) if (owner[f] === i && vis[f]) m[f] = 1;
    const sm = subMesh(V, F, m);
    p.V = sm.V; p.F = sm.F;
  });
  const total = parts.reduce((s, p) => s + p.F.length / 3, 0);
  const keepFrac = Math.min(1, 5 * budget / Math.max(total, 1));
  if (keepFrac < 1) {
    for (const p of parts) {
      if (!p.F.length) continue;
      const w = weld(p.V, p.F, 1e-5);
      const d = decimate(w.V, w.F, Math.max(64, Math.floor(w.F.length / 3 * keepFrac)));
      p.V = d.V; p.F = d.F;
    }
    log(`  stage-1 decimation: ${total} -> ${parts.reduce((s, p) => s + p.F.length / 3, 0)} triangles`);
  }
  progress('Simplifying', 0.4);

  // ---- global error driven decimation (importance scaled positions)
  const pieces = [];
  let voff = 0;
  parts.forEach((p, i) => {
    if (!p.F.length) return;
    const w = weld(p.V, p.F, 1e-5);
    if (!w.F.length) return;
    const frac = visArea0[i] / Math.max(totArea[i], 1e-12);
    const s = CLS_IMPORTANCE[p.cls] * (0.35 + 0.65 * Math.sqrt(frac));
    const n = w.V.length / 3, c = [0, 0, 0];
    for (let k = 0; k < n; k++) { c[0] += w.V[3 * k]; c[1] += w.V[3 * k + 1]; c[2] += w.V[3 * k + 2]; }
    c[0] /= n; c[1] /= n; c[2] /= n;
    const SVp = new Float64Array(w.V.length);
    for (let k = 0; k < n; k++) for (let j = 0; j < 3; j++) SVp[3 * k + j] = (w.V[3 * k + j] - c[j]) * s + c[j];
    pieces.push({ part: i, OV: w.V, SV: SVp, F: w.F, voff });
    voff += n;
  });
  const OV = concatF64(pieces.map((q) => q.OV)), SV = concatF64(pieces.map((q) => q.SV));
  const vowner = new Int32Array(voff);
  let gfLen = 0;
  for (const q of pieces) gfLen += q.F.length;
  const GF = new Uint32Array(gfLen);
  {
    let o = 0;
    pieces.forEach((q, k) => {
      vowner.fill(k, q.voff, q.voff + q.OV.length / 3);
      for (let j = 0; j < q.F.length; j++) GF[o + j] = q.F[j] + q.voff;
      o += q.F.length;
    });
  }
  log(`  global decimation: ${GF.length / 3} -> ${budget} triangles`);
  const DF = GF.length / 3 > budget ? meshoptSimplify(SV, GF, budget) : GF;
  const DV = OV;
  const outParts = [];
  pieces.forEach((q, k) => {
    const m = new Uint8Array(DF.length / 3);
    let any = false;
    for (let f = 0; f < m.length; f++) if (vowner[DF[3 * f]] === k) { m[f] = 1; any = true; }
    if (!any) return;
    const sm = subMesh(DV, DF, m);
    const p = parts[q.part];
    outParts.push({ name: p.name, cls: p.cls, mat: p.mat, V: sm.V, F: sm.F, twosided: !!p.twosided });
  });
  log(`  after decimation: ${outParts.reduce((s, p) => s + p.F.length / 3, 0)} triangles`);
  progress('Simplifying', 1);

  // ---- group by material (class + colour)
  const groups = new Map();
  const FIXED = [CLS.PAINT, CLS.GLASS, CLS.CHROME, CLS.TIRE, CLS.LIGHT_RED, CLS.CARBON];
  for (const p of outParts) {
    const s = linToSrgb(p.mat.color);
    const key = p.cls === CLS.LIGHT_CLEAR ? `${p.cls}|${isAmber(s)}`
      : FIXED.includes(p.cls) ? `${p.cls}` : `${p.cls}|${s.map((v) => Math.round(v * 6)).join(',')}`;
    if (!groups.has(key)) groups.set(key, { cls: p.cls, list: [] });
    groups.get(key).list.push(p);
  }
  const maxm = Math.max(1, Math.min(16, maxMaterials));
  const NEUTRAL = [CLS.PLASTIC, CLS.GENERIC, CLS.INTERIOR, CLS.METAL, CLS.TIRE, CLS.CARBON];
  const gsize = (k) => groups.get(k).list.reduce((s, p) => s + p.F.length / 3, 0);
  const gcol = (k) => {
    const pl = groups.get(k).list, acc = [0, 0, 0];
    let ws = 0;
    for (const p of pl) { const w = p.F.length / 3 + 1e-9; for (let c = 0; c < 3; c++) acc[c] += p.mat.color[c] * w; ws += w; }
    return linToSrgb(acc.map((v) => v / ws));
  };
  const cdist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
  while (groups.size > maxm) {
    const keys = [...groups.keys()];
    let best = null;
    for (let i = 0; i < keys.length; i++)
      for (let j = i + 1; j < keys.length; j++) {
        if (groups.get(keys[i]).cls !== groups.get(keys[j]).cls) continue;
        const d = cdist(gcol(keys[i]), gcol(keys[j]));
        if (!best || d < best[0]) best = [d, keys[i], keys[j]];
      }
    if (!best) {
      const neutral = keys.filter((k) => NEUTRAL.includes(groups.get(k).cls));
      if (neutral.length < 2) {
        const small = keys.reduce((a, b) => (gsize(a) <= gsize(b) ? a : b));
        log(`  warning: ${groups.size} materials, renderer limit is ${maxm}; dropping the smallest`);
        groups.delete(small);
        continue;
      }
      const small = neutral.reduce((a, b) => (gsize(a) <= gsize(b) ? a : b));
      const tgt = neutral.filter((k) => k !== small).reduce((a, b) => (cdist(gcol(a), gcol(small)) <= cdist(gcol(b), gcol(small)) ? a : b));
      best = [0, tgt, small];
    }
    let [, a, b] = best;
    if (gsize(b) > gsize(a)) [a, b] = [b, a];
    groups.get(a).list.push(...groups.get(b).list);
    groups.delete(b);
  }

  let paintLin = null;
  if (opts.paint) paintLin = parseColor(opts.paint);
  else if (cfg.paint) paintLin = parseColor(cfg.paint);

  const allV = [], allN = [], allF = [], mats = [];
  let vo = 0, to = 0;
  for (const { cls, list } of groups.values()) {
    const j = joinParts(list);
    const cr = cls !== CLS.TIRE ? crease : 60;
    const cn = creaseNormals(j.V, j.F, cr);
    const Nn = transferNormals(cn.V, cn.N, hiGrid, hiN);
    const a = list.map((p) => faceAreas(p.V, p.F, false).reduce((s, v) => s + v, 0));
    const aw = a.map((v) => v + 1e-12), asum = aw.reduce((s, v) => s + v, 0);
    let color = [0, 1, 2].map((c) => list.reduce((s, p, i) => s + p.mat.color[c] * aw[i], 0) / asum);
    const rough = list.reduce((s, p, i) => s + (p.mat.roughness ?? 0.5) * aw[i], 0) / asum;
    if (cls === CLS.PAINT && paintLin) color = paintLin;
    allV.push(cn.V); allN.push(Nn);
    const Fo = new Uint32Array(cn.F.length);
    for (let k = 0; k < cn.F.length; k++) Fo[k] = cn.F[k] + vo;
    allF.push(Fo);
    const dsided = list.some((p) => p.twosided) || cls === CLS.GLASS;
    mats.push({ cls, color, rough, triStart: to, triCount: cn.F.length / 3, dsided,
      names: [...new Set(list.map((p) => p.mat.name))].sort() });
    vo += cn.V.length / 3; to += cn.F.length / 3;
  }
  V = concatF64(allV);
  const N = concatF64(allN);
  {
    let n = 0;
    for (const f of allF) n += f.length;
    F = new Uint32Array(n);
    let o = 0;
    for (const f of allF) { F.set(f, o); o += f.length; }
  }
  const nvFinal = V.length / 3;
  if (nvFinal > 65535) throw new Error(`Too many vertices (${nvFinal}); lower the triangle budget`);
  log(`  final: ${nvFinal} vertices, ${F.length / 3} triangles, ${mats.length} materials`);

  // ---- AO
  let ao;
  if (opts.noAO) ao = new Float64Array(nvFinal).fill(1);
  else {
    log('  baking ambient occlusion ...');
    progress('Baking ambient occlusion', 0);
    ao = bakeAO(V, N, F, radius, 128, 512, true, (f) => progress('Baking ambient occlusion', f));
  }

  // ---- matcaps
  log('  rendering matcaps ...');
  mats.forEach((m, i) => {
    progress('Rendering materials', i / mats.length);
    const size = CLS_MATCAP[m.cls] || 64;
    const r = renderMatcap(m.cls, m.color, size, m.rough);
    m.matcapSize = size;
    m.rgb565 = r.rgb565;
  });
  progress('Rendering materials', 1);

  const tm = new Uint8Array(F.length / 3);
  mats.forEach((m, i) => tm.fill(i, m.triStart, m.triStart + m.triCount));
  const name = (opts.name || cfg.name || 'model').trim() || 'model';
  const bytes = writeR3DM({ V, N, F, ao, tm }, mats, radius, name);
  log(`  blob: ${(bytes.length / 1024).toFixed(1)} KiB`);
  const report = {
    name, vertices: nvFinal, triangles: F.length / 3, radius, mode: cfg._isCar ? 'car' : 'object',
    materials: mats.map((m) => ({ cls: CLS_NAMES[m.cls], tris: m.triCount, src: m.names, matcap: m.matcapSize,
      color: linToSrgb(m.color) })),
  };
  return { bytes, report };
}
