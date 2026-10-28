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
const WHEEL_RX = /wheel|(?<![a-z])rims?(?![a-z])|tyre|tire|felg/;
const LIVERY_RX = /paint|(?<![a-z])lack|primary/;            // a textured paint material = livery
let splitCounter = 0;
const TEX_DETAIL_CLS = [CLS.GENERIC, CLS.PLASTIC, CLS.INTERIOR, CLS.PAINT, CLS.LIGHT_RED, CLS.LIGHT_CLEAR];

// linear per-vertex colour from per-face sRGB colours (area weighted)
function vertexColors(p) {
  const nv = p.V.length / 3, nf = p.F.length / 3, acc = new Float64Array(nv * 4);
  const area = faceAreas(p.V, p.F);
  for (let f = 0; f < nf; f++) {
    const w = area[f] + 1e-12;
    const c = [srgbToLin1(p.faceCol[3 * f]), srgbToLin1(p.faceCol[3 * f + 1]), srgbToLin1(p.faceCol[3 * f + 2])];
    for (let k = 0; k < 3; k++) {
      const v = p.F[3 * f + k];
      acc[4 * v] += c[0] * w; acc[4 * v + 1] += c[1] * w; acc[4 * v + 2] += c[2] * w; acc[4 * v + 3] += w;
    }
  }
  const out = new Float32Array(nv * 3);
  for (let v = 0; v < nv; v++) for (let c = 0; c < 3; c++) out[3 * v + c] = acc[4 * v + c] / Math.max(acc[4 * v + 3], 1e-12);
  return out;
}
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
  [CLS.METAL, /(?<![a-z])rims?(?![a-z])|wheel|alloy|felge|disc|disk|brake|caliper|metal|steel|(?<![a-z])alu|exhaust|felg/],
  [CLS.PAINT, /paint|body|carpaint|car_paint|(?<![a-z])lack|karoser|exterior|coat|primary/],
  [CLS.PLASTIC, /plastic|trim|black|grill|grille|bumper_low|matte|kunststoff/],
];

export function classify(part) {
  const name = (part.mat.name + ' ' + part.name).toLowerCase();
  const mi = part.mat;
  // the part's own name (without the material name it often carries) says window: glass,
  // even when the material is a shared "lights and glass" one
  const mn = part.mat.name.toLowerCase();
  const own = mn.length >= 3 ? part.name.toLowerCase().split(mn).join(' ') : part.name.toLowerCase();
  if (/window|windshield|windscreen|glass|scheibe|szyb/.test(own) && !/light|lamp|led|lens|mirror/.test(own)) return CLS.GLASS;
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
function splitByTexture(p, maxColors, lWeight = 1.0, chromaMerge = 0) {
  if (!p.faceCol) return null;
  const F = p.F, V = p.V, nf = F.length / 3;
  const area = faceAreas(V, F);
  const lab = new Float64Array(nf * 3);
  for (let f = 0; f < nf; f++) {
    srgbToOklab(p.faceCol[3 * f], p.faceCol[3 * f + 1], p.faceCol[3 * f + 2], lab, 3 * f);
    lab[3 * f] *= lWeight; lab[3 * f + 1] *= 1.6; lab[3 * f + 2] *= 1.6;
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
  let K = cent.length;
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
  // clusters that differ only in brightness (light and shadow baked into the texture) are one
  // region when the brightness is kept per vertex anyway
  if (chromaMerge > 0 && K > 1) {
    // complete linkage: a cluster joins a group only if it is close to every member
    const cd = (i, j) => Math.hypot(cent[i][1] - cent[j][1], cent[i][2] - cent[j][2]);
    const groups = [], g = new Int32Array(K);
    for (let i = 0; i < K; i++) {
      let gi = groups.findIndex((m) => m.every((j) => cd(i, j) < chromaMerge));
      if (gi < 0) { gi = groups.length; groups.push([]); }
      groups[gi].push(i);
      g[i] = gi;
    }
    if (groups.length < K) {
      for (let f = 0; f < nf; f++) lbl[f] = g[lbl[f]];
      K = groups.length;
    }
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

function isAtlas(p, minFaces = 1500) {
  const nf = p.F.length / 3;
  if (nf < minFaces || !p.faceCol) return false;
  let s = 0, s2 = 0, sat = 0;
  for (let f = 0; f < nf; f++) {
    const c = faceColor(p, f), l = dot3(c, LUM);
    s += l; s2 += l * l;
    if (ptp3(c) > 0.25) sat++;
  }
  const mean = s / nf, std = Math.sqrt(Math.max(s2 / nf - mean * mean, 0));
  return std > 0.12 || sat / nf > 0.02;
}

// textured parts of one material are clustered together (consistent regions across pieces)
function mergeByMaterial(parts) {
  const groups = new Map(), out = [];
  for (const p of parts) {
    if (!p.faceCol) { out.push(p); continue; }
    const k = p.mat.name + '|' + p.cls;
    if (!groups.has(k)) { groups.set(k, []); out.push(k); }
    groups.get(k).push(p);
  }
  return out.map((e) => {
    if (typeof e !== 'string') return e;
    const list = groups.get(e);
    if (list.length === 1) return list[0];
    const j = joinParts(list);
    let n = 0;
    for (const p of list) n += p.faceCol.length;
    const fc = new Float32Array(n);
    let o = 0;
    for (const p of list) { fc.set(p.faceCol, o); o += p.faceCol.length; }
    return { ...list[0], V: j.V, F: j.F, faceCol: fc, texDetail: true };
  });
}

function applyTextureSplit(parts, isCar, maxColors, log) {
  if (maxColors <= 1) return parts;
  if (!isCar) parts = mergeByMaterial(parts);
  // the biggest textured "body" part is the paint even without "paint" in its name
  let biggestPaint = null, bpa = 0;
  for (const p of parts) {
    if (p.cls !== CLS.PAINT || !p.faceCol) continue;
    const a = faceAreas(p.V, p.F).reduce((x, y) => x + y, 0);
    if (a > bpa) { bpa = a; biggestPaint = p; }
  }
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
      else if (p.cls === CLS.PAINT && (p.paintByName || p === biggestPaint))
        mode = isAtlas(p, 300) ? 'paint' : null;                            // livery
      else if (p.cls === CLS.METAL && WHEEL_RX.test((p.mat.name + ' ' + p.name).toLowerCase()))
        mode = isAtlas(p, 300) ? 'wheel' : null;                             // rim + tyre in one texture
      else if ([CLS.GENERIC, CLS.PLASTIC, CLS.PAINT, CLS.INTERIOR].includes(p.cls)) mode = isAtlas(p) ? 'atlas' : null;
    }
    let res = null;
    if (mode) {
      try {
        const k = mode === 'object' ? maxColors : mode === 'light' || mode === 'paint' ? Math.min(maxColors, 6)
          : mode === 'wheel' ? Math.min(maxColors, 5) : Math.min(maxColors, 10);
        // objects / liveries: brightness detail is kept per vertex, so regions follow hue more
        // than shading (baked light and shadow in the texture must not become separate colours)
        res = splitByTexture(p, k, mode === 'object' ? 0.8 : mode === 'paint' ? 0.4 : 1.0, 0);
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
    const splitId = ++splitCounter;
    // livery: the region with the largest area is the body colour
    let mainRegion = 0;
    if (mode === 'paint') {
      const area = faceAreas(p.V, p.F), ra = new Float64Array(colors.length);
      for (let f = 0; f < nf; f++) ra[lbl[f]] += area[f];
      for (let r = 1; r < colors.length; r++) if (ra[r] > ra[mainRegion]) mainRegion = r;
    }
    colors.forEach((c, r) => {
      const fm = new Uint8Array(nf);
      let n = 0, sx = 0, sy = 0;
      for (let f = 0; f < nf; f++) if (lbl[f] === r) { fm[f] = 1; n++; sx += cen[3 * f]; sy += cen[3 * f + 1]; }
      if (!n) return;
      const cx = sx / n, cy = sy / n;
      let pieces;
      if (mode === 'object') {
        const lum = dot3(srgbToLin(c), LUM);
        pieces = [[fm, lum < 0.02 ? CLS.PLASTIC : CLS.GENERIC, c]];
      } else if (mode === 'light') {
        // a window in the lamp material (one "lights and glass" texture): above the window
        // line, dark and neutral -> glass
        const [cls, col] = lightRegionClass(c, cx);
        if (isCar && dot3(c, LUM) < 0.3 && ptp3(c) < 0.15) {
          const hi = new Uint8Array(nf), lo = new Uint8Array(nf);
          for (let f = 0; f < nf; f++) if (fm[f]) { if (cen[3 * f + 1] > 0.6 * carH) hi[f] = 1; else lo[f] = 1; }
          pieces = [[hi, CLS.GLASS, c], [lo, cls, col]];
        } else pieces = [[fm, cls, col]];
      } else if (mode === 'paint') {
        pieces = [[fm, CLS.PAINT, c]];
      } else if (mode === 'wheel') {
        const lum = dot3(c, LUM), sat = ptp3(c);
        pieces = [[fm, lum < 0.22 && sat < 0.12 ? CLS.TIRE : lum > 0.55 && sat < 0.15 ? CLS.CHROME : CLS.METAL, c]];
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
        const piece = { name: `${p.name}_c${r}_${j}`, mat: mi, cls, V: sm.V, F: sm.F, faceCol: fc, fromSplit: true, texDetail: true, splitGroup: splitId };
        if (mode === 'paint') { piece.paintSplit = true; piece.paintSrc = p.mat.name; piece.paintKey = r === mainRegion ? 'main' : `l${r}`; }
        out.push(piece);
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
  // (a) glass lying on top of light geometry; (b) glass at the front / rear end of the car
  // below the window line: head / tail / fog lamp lenses (windows sit higher or further in)
  const lights = parts.filter((p) => (p.cls === CLS.LIGHT_RED || p.cls === CLS.LIGHT_CLEAR) && p.F.length);
  let carH = 0;
  for (const p of parts) for (let i = 1; i < p.V.length; i += 3) carH = Math.max(carH, p.V[i]);
  let grid = null;
  if (lights.length) {
    const pts = [];
    for (const p of lights) {
      pts.push(p.V);
      const nf = p.F.length / 3, c = new Float64Array(nf * 3);
      for (let f = 0; f < nf; f++)
        for (let k = 0; k < 3; k++) c[3 * f + k] = (p.V[3 * p.F[3 * f] + k] + p.V[3 * p.F[3 * f + 1] + k] + p.V[3 * p.F[3 * f + 2] + k]) / 3;
      pts.push(c);
    }
    grid = new PointGrid(concatF64(pts), dist);
  }
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
      let k = 1;
      if (Math.abs(x) > 0.5 && y < 0.6 * carH) k = 0;
      else if (grid) { grid.nearest(x, y, z, dist); if (!(grid.lastDist > dist)) k = 0; }
      keep[f] = k;
      nk += k;
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

// cars whose lamps have no telling names (Material__123): small parts at the very front /
// rear below the roof, bright neutral (or amber) at the front = headlights, red at the
// rear = tail lights. Only used for an end of the car where no lamp was found yet.
function lampsByPosition(parts, log) {
  let carH = 0;
  for (const p of parts) for (let i = 1; i < p.V.length; i += 3) carH = Math.max(carH, p.V[i]);
  const hasLight = (front) => parts.some((p) => {
    if ((p.cls !== CLS.LIGHT_CLEAR && p.cls !== CLS.LIGHT_RED) || p.fromSplit) return false;   // named lamps only
    let sx = 0;
    for (let i = 0; i < p.V.length; i += 3) sx += p.V[i];
    return front ? sx > 0 : sx < 0;
  });
  const doFront = !hasLight(true), doRear = !hasLight(false);
  if (!doFront && !doRear) return parts;
  const out = [];
  let nFront = 0, nRear = 0;
  for (const p of parts) {
    if (![CLS.GENERIC, CLS.PLASTIC].includes(p.cls) || !p.F.length) { out.push(p); continue; }
    let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
    for (let i = 0; i < p.V.length; i += 3) {
      x0 = Math.min(x0, p.V[i]); x1 = Math.max(x1, p.V[i]);
      y0 = Math.min(y0, p.V[i + 1]); y1 = Math.max(y1, p.V[i + 1]);
    }
    const front = doFront && x0 > 0.62, rear = doRear && x1 < -0.62;
    if ((!front && !rear) || y1 - y0 > 0.35 * carH || y0 < 0.2 * carH || y1 > 0.8 * carH) { out.push(p); continue; }
    const nf = p.F.length / 3, lamp = new Uint8Array(nf);
    const matCol = linToSrgb(p.mat.color);
    let n = 0;
    for (let f = 0; f < nf; f++) {
      const c = p.faceCol ? faceColor(p, f) : matCol;
      const ok = front ? ((dot3(c, LUM) > 0.45 && ptp3(c) < 0.15) || isAmber(c)) : (isRed(c) || isReddish(c));
      if (ok) { lamp[f] = 1; n++; }
    }
    if (!n) { out.push(p); continue; }
    // textured lamp: the whole part becomes a lamp, the texture split ('light' mode) then
    // sorts its colour regions (red lens, reverse light, black housing, amber indicator)
    if (p.faceCol && n >= 0.2 * nf) {
      out.push({ ...p, cls: front ? CLS.LIGHT_CLEAR : CLS.LIGHT_RED });
      if (front) nFront += nf; else nRear += nf;
      continue;
    }
    // the whole part when most of it qualifies (no ragged split), else only those faces
    const whole = n >= 0.5 * nf;
    const pieces = whole ? [[null, true]] : [[lamp, true], [lamp.map((v) => 1 - v), false]];
    for (const [m, isLamp] of pieces) {
      const sm = m ? subMesh(p.V, p.F, m) : { V: p.V, F: p.F };
      if (!sm.F.length) continue;
      let fc = null;
      if (p.faceCol && m) {
        fc = new Float32Array(sm.F.length);
        let q = 0;
        for (let f = 0; f < nf; f++) if (m[f]) { fc[q++] = p.faceCol[3 * f]; fc[q++] = p.faceCol[3 * f + 1]; fc[q++] = p.faceCol[3 * f + 2]; }
      } else if (!m) fc = p.faceCol;
      if (!isLamp) { out.push({ ...p, V: sm.V, F: sm.F, faceCol: fc }); continue; }
      const cls = front ? CLS.LIGHT_CLEAR : CLS.LIGHT_RED;
      let col = front ? [1, 1, 1] : matCol;
      if (front && isAmber(matCol)) col = [1.0, 0.55, 0.08];
      if (!front && !isRed(matCol)) col = [0.8, 0.05, 0.04];
      out.push({ ...p, V: sm.V, F: sm.F, faceCol: fc, cls, mat: { ...p.mat, color: srgbToLin(col), name: p.mat.name + (front ? '_head' : '_tail') } });
      if (front) nFront += sm.F.length / 3; else nRear += sm.F.length / 3;
    }
  }
  if (nFront || nRear) log(`  lamps found by position: ${nFront} head light, ${nRear} tail light triangles`);
  return out;
}

// transparent shells lying on an opaque surface (clear coat over the paint, decal films):
// the renderer has no transparency, both layers would fight for the same pixels
function dropOverlayShells(parts, log, tol = 0.012) {
  const isLight = (p) => p.cls === CLS.LIGHT_RED || p.cls === CLS.LIGHT_CLEAR;
  const cand = parts.filter((p) => p.mat.alpha < 0.6 && !isLight(p) && p.cls !== CLS.GLASS && p.F.length);
  if (!cand.length) return parts;
  const pts = [];
  for (const p of parts) {
    if (p.mat.alpha < 0.6 || p.cls === CLS.GLASS || !p.F.length) continue;
    pts.push(p.V);
    const nf = p.F.length / 3, c = new Float64Array(nf * 3);
    for (let f = 0; f < nf; f++)
      for (let k = 0; k < 3; k++) c[3 * f + k] = (p.V[3 * p.F[3 * f] + k] + p.V[3 * p.F[3 * f + 1] + k] + p.V[3 * p.F[3 * f + 2] + k]) / 3;
    pts.push(c);
  }
  if (!pts.length) return parts;
  const grid = new PointGrid(concatF64(pts), tol);
  const drop = new Set();
  for (const p of cand) {
    const nf = p.F.length / 3, area = faceAreas(p.V, p.F);
    let near = 0, tot = 0;
    for (let f = 0; f < nf; f++) {
      const c = [0, 1, 2].map((k) => (p.V[3 * p.F[3 * f] + k] + p.V[3 * p.F[3 * f + 1] + k] + p.V[3 * p.F[3 * f + 2] + k]) / 3);
      grid.nearest(c[0], c[1], c[2], tol);
      if (grid.lastDist <= tol) near += area[f];
      tot += area[f];
    }
    if (tot > 0 && near >= 0.6 * tot) drop.add(p);
  }
  if (!drop.size) return parts;
  log(`  removed transparent overlay layer(s): ${[...new Set([...drop].map((p) => p.mat.name))].join(', ')}`);
  return parts.filter((p) => !drop.has(p));
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

function lampFrontSign(parts, R, x0 = 0) {   // x0: middle of the model along x in the R frame
  const xr = [], wr = [], xw = [], ww = [];
  let any = false;
  for (const p of parts) {
    if (p.cls === CLS_DROP) continue;
    const atlas = [CLS.GENERIC, CLS.PLASTIC, CLS.PAINT].includes(p.cls);
    if (![CLS.LIGHT_RED, CLS.LIGHT_CLEAR, CLS.GLASS].includes(p.cls) && !atlas) continue;
    any = true;
    const V = applyR(p.V0, R), F = p.F, nf = F.length / 3;
    for (let i = 0; i < V.length; i += 3) V[i] -= x0;
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
  const bounds = (R, only) => {
    const mn = [Infinity, Infinity, Infinity], mx = [-Infinity, -Infinity, -Infinity];
    for (const p of only ? [only] : parts) {
      if (!only && p.cls === CLS_DROP) continue;
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
  // ground / shadow planes: flat parts at the very bottom covering most of the footprint
  {
    const H = mx[1] - mn[1], A = (mx[0] - mn[0]) * (mx[2] - mn[2]);
    const ground = [];
    for (const p of parts) {
      if (p.cls === CLS_DROP) continue;
      const [a, b] = bounds(R, p);
      if (b[1] - a[1] <= 0.02 * H && b[1] <= mn[1] + 0.03 * H && (b[0] - a[0]) * (b[2] - a[2]) >= 0.25 * A) ground.push(p);
    }
    if (ground.length && ground.length < parts.filter((p) => p.cls !== CLS_DROP).length) {
      const saved = ground.map((p) => p.cls);
      ground.forEach((p) => { p.cls = CLS_DROP; });
      const [m2, x2] = bounds(R);
      const L2 = Math.max(x2[0] - m2[0], x2[2] - m2[2]);
      if (x2[1] - m2[1] >= 0.05 * L2) {
        cfg._log?.(`  removed ground plane: ${ground.map((p) => p.name).join(', ')}`);
        [mn, mx] = [m2, x2];
      } else ground.forEach((p, i) => { p.cls = saved[i]; });      // the plane is the model
    }
  }
  const ext = [mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2]];
  const kind = cfg.kind || 'auto';
  // car: tyres or lights found by name, or a body paint material on a car shaped model
  const Lh = Math.max(ext[0], ext[2]), Wh = Math.min(ext[0], ext[2]);
  const carShape = Wh > 0 && ext[1] > 0 && Lh / Wh >= 1.5 && Lh / Wh <= 3.4 && Lh / ext[1] >= 1.7 && Lh / ext[1] <= 4.5;
  const live = parts.filter((p) => p.cls !== CLS_DROP);
  // wheels: round-ish parts standing on the ground, two at each end
  let wheels = 0;
  if (carShape) {
    const H = ext[1], alongX = ext[0] >= ext[2], ends = new Set();
    for (const p of live) {
      const [a, b] = bounds(R, p);
      const h = b[1] - a[1], ex = b[0] - a[0], ez = b[2] - a[2];
      const len = alongX ? ex : ez, thick = alongX ? ez : ex;
      if (a[1] > mn[1] + 0.06 * H || h < 0.18 * H || h > 0.6 * H) continue;
      if (len < 0.7 * h || len > 1.4 * h || thick > 0.75 * h) continue;
      const c = alongX ? (a[0] + b[0]) / 2 - (mn[0] + mx[0]) / 2 : (a[2] + b[2]) / 2 - (mn[2] + mx[2]) / 2;
      if (Math.abs(c) < 0.25 * Lh) continue;
      wheels++; ends.add(c > 0);
    }
    if (ends.size < 2) wheels = 0;
  }
  const isCar = kind === 'car' || (kind === 'auto' && (live.some((p) => [CLS.TIRE, CLS.LIGHT_RED, CLS.LIGHT_CLEAR].includes(p.cls))
    || (carShape && (wheels >= 3 || live.some((p) => p.cls === CLS.PAINT)))));
  cfg._isCar = isCar;
  if (!isCar) R = matMul(rotY(90), R);
  else if (ext[2] > ext[0]) R = matMul(rotY(-90), R);
  if (cfg.yaw_deg !== undefined && cfg.yaw_deg !== null && cfg.yaw_deg !== '') R = matMul(rotY(Number(cfg.yaw_deg)), R);
  else if (isCar) {
    const [b0, b1] = bounds(R);
    let side = lampFrontSign(parts, R, (b0[0] + b1[0]) / 2);
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
function meshoptSimplify(V, F, target, lockBorder = false, attr = null, attrWeight = 0, extraFlags = []) {
  const pos = new Float32Array(V);
  const idx = F instanceof Uint32Array ? F : new Uint32Array(F);
  const tic = Math.min(idx.length, Math.max(0, Math.floor(target)) * 3);
  const flags = (lockBorder ? ['LockBorder'] : []).concat(extraFlags);
  // attr: per-vertex region colour; borders between colour regions are kept where possible
  const [out] = attr
    ? MeshoptSimplifier.simplifyWithAttributes(idx, pos, 3, attr, 3, [attrWeight, attrWeight, attrWeight], null, tic, 1.0, flags)
    : MeshoptSimplifier.simplify(idx, pos, 3, tic, 1.0, flags);
  return out;
}

const REGION_ATTR_WEIGHT = 1.0;

function decimate(V, F, target, lockBorder = false) {
  if (F.length / 3 <= target) return { V, F };
  const F2 = meshoptSimplify(V, F, target, lockBorder);
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

// part indices grouped by the surface they were split from (colour regions of one texture)
function splitGroups(parts) {
  const by = new Map(), out = [];
  parts.forEach((p, i) => {
    if (p.splitGroup === undefined) { out.push([i]); return; }
    if (!by.has(p.splitGroup)) { by.set(p.splitGroup, []); out.push(by.get(p.splitGroup)); }
    by.get(p.splitGroup).push(i);
  });
  return out;
}

// several parts -> one welded mesh (shared border vertices) + owner (list index) per face
function mergeParts(list) {
  const j = joinParts(list);
  const w = weld(j.V, j.F, 1e-5);
  // weld drops degenerate faces: rebuild the owner per kept face
  const owner = new Int32Array(w.F.length / 3);
  let o = 0;
  for (let f = 0; f < j.F.length / 3; f++) {
    const a = w.inv[j.F[3 * f]], b = w.inv[j.F[3 * f + 1]], c = w.inv[j.F[3 * f + 2]];
    if (a === b || b === c || a === c) continue;
    owner[o++] = j.owner[f];
  }
  const nf = owner.length, cen = new Float64Array(nf * 3);
  for (let f = 0; f < nf; f++)
    for (let k = 0; k < 3; k++) cen[3 * f + k] = (w.V[3 * w.F[3 * f] + k] + w.V[3 * w.F[3 * f + 1] + k] + w.V[3 * w.F[3 * f + 2] + k]) / 3;
  // region colour per vertex (OKLab of the owner with most incident faces)
  let attr = null;
  if (list.length > 1) {
    const nv = w.V.length / 3, K = list.length, cnt = new Uint16Array(nv * Math.min(K, 64)), best = new Int32Array(nv);
    const KK = Math.min(K, 64);
    for (let f = 0; f < nf; f++) {
      const o2 = Math.min(owner[f], KK - 1);
      for (let k = 0; k < 3; k++) { const v = w.F[3 * f + k]; if (++cnt[v * KK + o2] > cnt[v * KK + best[v]]) best[v] = o2; }
    }
    const labs = list.map((p) => { const c = linToSrgb(p.mat.color), o3 = [0, 0, 0]; srgbToOklab(c[0], c[1], c[2], o3, 0); return o3; });
    attr = new Float32Array(nv * 3);
    for (let v = 0; v < nv; v++) { const l = labs[best[v]]; attr[3 * v] = l[0]; attr[3 * v + 1] = l[1] * 1.6; attr[3 * v + 2] = l[2] * 1.6; }
  }
  return { V: w.V, F: w.F, owner, cen, attr };
}

// owner of each simplified triangle = owner of the nearest original face (by centroid)
function faceOwners(m, D) {
  const nf = D.length / 3, out = new Int32Array(nf);
  if (!m.grid) {
    let ext = 0;
    for (let i = 0; i < m.V.length; i++) ext = Math.max(ext, Math.abs(m.V[i]));
    m.cell = Math.max(ext / 200, 1e-4);
    m.grid = new PointGrid(m.cen, m.cell);
  }
  for (let f = 0; f < nf; f++) {
    let x = 0, y = 0, z = 0;
    for (let k = 0; k < 3; k++) { const v = 3 * D[3 * f + k]; x += m.V[v]; y += m.V[v + 1]; z += m.V[v + 2]; }
    x /= 3; y /= 3; z /= 3;
    let r = m.cell, q = -1;
    for (let it = 0; it < 6 && q < 0; it++, r *= 3) q = m.grid.nearest(x, y, z, r);
    out[f] = q >= 0 ? m.owner[q] : m.owner[0];
  }
  return out;
}

function outwardNormals(V, N, yMid) {
  for (let i = 0; i < V.length; i += 3) {
    const d = V[i] * N[i] + (V[i + 1] - yMid) * N[i + 1] + V[i + 2] * N[i + 2];
    if (d < 0) { N[i] = -N[i]; N[i + 1] = -N[i + 1]; N[i + 2] = -N[i + 2]; }
  }
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
    faceCol: p.faceCol || null, texDetail: !!p.faceCol,
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
    else { p.cls = classify(p); p.paintByName = p.cls === CLS.PAINT && LIVERY_RX.test((p.mat.name + ' ' + p.name).toLowerCase()); }
  }
  cfg._log = log;
  normaliseOrientation(parts, cfg);
  delete cfg._log;
  for (const p of parts) delete p.V0;
  parts = parts.filter((p) => p.cls !== CLS_DROP);
  log(`  mode: ${cfg._isCar ? 'car' : 'object'}`);
  parts = dropOverlayShells(parts, log);
  if (cfg._isCar) parts = lampsByPosition(parts, log);
  parts = applyTextureSplit(parts, cfg._isCar, texColors, log);
  if (cfg._isCar) parts = dropLampCovers(parts, log);
  let ndup = 0;
  for (const p of parts) {
    const vc = p.faceCol && p.texDetail ? vertexColors(p) : null;
    const w = weld(p.V, p.F, 1e-6);
    const Fd = dedupeFaces(w.F);
    ndup += (w.F.length - Fd.length) / 3;
    fixWinding(Fd, w.V.length / 3);
    p.vcol = null;
    if (vc) {                                  // average the colours of welded vertices
      const n2 = w.V.length / 3, acc = new Float64Array(n2 * 4);
      for (let i = 0; i < w.inv.length; i++) {
        const r = w.inv[i];
        acc[4 * r] += vc[3 * i]; acc[4 * r + 1] += vc[3 * i + 1]; acc[4 * r + 2] += vc[3 * i + 2]; acc[4 * r + 3]++;
      }
      p.vcol = new Float32Array(n2 * 3);
      for (let r = 0; r < n2; r++) for (let c = 0; c < 3; c++) p.vcol[3 * r + c] = acc[4 * r + c] / Math.max(acc[4 * r + 3], 1);
    }
    p.V = w.V; p.F = Fd; p.faceCol = null;
  }
  parts = parts.filter((p) => p.F.length);
  if (ndup) log(`  removed ${ndup} duplicate (double-sided) triangles`);

  // ---- global arrays
  let { V, F, owner } = joinParts(parts);
  // linear texture colour per vertex (-1 = no texture), for the texture detail below
  const hiCol = new Float32Array(V.length).fill(-1);
  {
    let o = 0;
    for (const p of parts) { if (p.vcol) hiCol.set(p.vcol, o); p.hiRange = [o / 3, p.V.length / 3]; o += p.V.length; p.vcol = null; }
  }
  const hiV = V;
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
    // a paint material with a livery competes with its colour regions (paintSrc)
    const visHigh = new Float64Array(parts.length);
    let yTop = 0;
    for (let i = 1; i < V.length; i += 3) yTop = Math.max(yTop, V[i]);
    for (let f = 0; f < nf; f++) {
      if (!vis[f]) continue;
      const cy = (V[3 * F[3 * f] + 1] + V[3 * F[3 * f + 1] + 1] + V[3 * F[3 * f + 2] + 1]) / 3;
      if (cy > 0.3 * yTop) visHigh[owner[f]] += areas[f];
    }
    const score = new Map();
    parts.forEach((p, i) => {
      if (![CLS.GENERIC, CLS.PLASTIC, CLS.PAINT].includes(p.cls) || (p.fromSplit && !p.paintSplit)) return;
      const sat = ptp3(linToSrgb(p.mat.color));
      // only surface above the sills counts (the underbody is big but never the paint)
      let sc = visHigh[i] * (0.25 + 1.5 * sat) * 0.5;
      if (p.cls === CLS.PAINT) sc *= 1.5;
      if (LIVERY_RX.test((p.mat.name + ' ' + p.name).toLowerCase())) sc *= 2.5;      // "paint", "primary"
      const k = p.paintSrc || p.mat.name;
      score.set(k, (score.get(k) || 0) + sc);
    });
    if (score.size) {
      let mname = null, best = -Infinity;
      for (const [k, v] of score) if (v > best) { best = v; mname = k; }
      for (const p of parts) {
        if ((p.paintSrc || p.mat.name) === mname) p.cls = CLS.PAINT;
        else if (p.cls === CLS.PAINT) {
          p.cls = dot3(p.mat.color, LUM) < 0.02 ? CLS.PLASTIC : CLS.GENERIC;
          p.paintKey = undefined;
        }
      }
      log(`  paint: material '${mname}'${parts.some((p) => p.paintSrc === mname) ? ' (with livery)' : ''}`);
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
    // colour regions of one surface are simplified together (shared borders, no cracks)
    for (const list of splitGroups(parts)) {
      const live = list.filter((i) => parts[i].F.length);
      if (!live.length) continue;
      const m = mergeParts(live.map((i) => parts[i]));
      if (!m.F.length) continue;
      const D = meshoptSimplify(m.V, m.F, Math.max(64, Math.floor(m.F.length / 3 * keepFrac)), false, m.attr, REGION_ATTR_WEIGHT);
      const own = faceOwners(m, D);
      live.forEach((i, k) => {
        const mask = new Uint8Array(D.length / 3);
        for (let f = 0; f < mask.length; f++) mask[f] = own[f] === k ? 1 : 0;
        const sm = subMesh(m.V, D, mask);
        parts[i].V = sm.V; parts[i].F = sm.F;
      });
    }
    log(`  stage-1 decimation: ${total} -> ${parts.reduce((s, p) => s + p.F.length / 3, 0)} triangles`);
  }
  progress('Simplifying', 0.4);

  // ---- global error driven decimation (importance scaled positions)
  // colour regions of one surface form one piece (shared borders); importance = area weighted
  const pieces = [];
  let voff = 0;
  for (const list of splitGroups(parts)) {
    const live = list.filter((i) => parts[i].F.length);
    if (!live.length) continue;
    const m = mergeParts(live.map((i) => parts[i]));
    if (!m.F.length) continue;
    let sw = 0, aw = 0;
    for (const i of live) {
      const frac = visArea0[i] / Math.max(totArea[i], 1e-12);
      const a = Math.max(totArea[i], 1e-12);
      sw += CLS_IMPORTANCE[parts[i].cls] * (0.35 + 0.65 * Math.sqrt(frac)) * a; aw += a;
    }
    const sc = sw / aw;
    const n = m.V.length / 3, c = [0, 0, 0];
    for (let k = 0; k < n; k++) { c[0] += m.V[3 * k]; c[1] += m.V[3 * k + 1]; c[2] += m.V[3 * k + 2]; }
    c[0] /= n; c[1] /= n; c[2] /= n;
    const SVp = new Float64Array(m.V.length);
    for (let k = 0; k < n; k++) for (let j = 0; j < 3; j++) SVp[3 * k + j] = (m.V[3 * k + j] - c[j]) * sc + c[j];
    pieces.push({ live, m, OV: m.V, SV: SVp, F: m.F, voff });
    voff += n;
  }
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
  let GA = null;
  if (pieces.some((q) => q.m.attr)) {
    GA = new Float32Array(voff * 3);
    for (const q of pieces) {
      if (q.m.attr) GA.set(q.m.attr, q.voff * 3);
      else {
        const c = linToSrgb(parts[q.live[0]].mat.color), l = [0, 0, 0];
        srgbToOklab(c[0], c[1], c[2], l, 0);
        for (let v = 0; v < q.OV.length / 3; v++) { GA[3 * (q.voff + v)] = l[0]; GA[3 * (q.voff + v) + 1] = l[1] * 1.6; GA[3 * (q.voff + v) + 2] = l[2] * 1.6; }
      }
    }
  }
  let DF = GF.length / 3 > budget ? meshoptSimplify(SV, GF, budget, false, GA, REGION_ATTR_WEIGHT) : GF;
  if (DF.length / 3 > budget * 1.1) {
    // some topologies (thin closed shells, tyre treads) block the normal edge collapses:
    // the permissive mode allows them
    const before = DF.length / 3;
    DF = meshoptSimplify(SV, GF, budget, false, GA, REGION_ATTR_WEIGHT, ['Permissive']);
    log(`  simplifier stopped at ${before} triangles, permissive mode: ${DF.length / 3}`);
  }
  const DV = OV;
  const outParts = [];
  pieces.forEach((q, k) => {
    // this piece's triangles, back in the piece's own vertex numbering
    const tris = [];
    for (let f = 0; f < DF.length / 3; f++) if (vowner[DF[3 * f]] === k) tris.push(f);
    if (!tris.length) return;
    const Floc = new Uint32Array(tris.length * 3);
    tris.forEach((f, t) => { for (let j = 0; j < 3; j++) Floc[3 * t + j] = DF[3 * f + j] - q.voff; });
    const own = q.live.length > 1 ? faceOwners(q.m, Floc) : new Int32Array(tris.length);
    q.live.forEach((i, kk) => {
      const mask = new Uint8Array(tris.length);
      let any = false;
      for (let t = 0; t < tris.length; t++) if (own[t] === kk) { mask[t] = 1; any = true; }
      if (!any) return;
      const sm = subMesh(q.OV, Floc, mask);
      const p = parts[i];
      outParts.push({ name: p.name, cls: p.cls, mat: p.mat, V: sm.V, F: sm.F, twosided: !!p.twosided, texDetail: !!p.texDetail, hiRange: p.hiRange, paintKey: p.paintKey });
    });
  });
  void DV;
  log(`  after decimation: ${outParts.reduce((s, p) => s + p.F.length / 3, 0)} triangles`);
  progress('Simplifying', 1);

  // ---- group by material (class + colour)
  const groups = new Map();
  const FIXED = [CLS.PAINT, CLS.GLASS, CLS.CHROME, CLS.TIRE, CLS.LIGHT_RED, CLS.CARBON];
  for (const p of outParts) {
    const s = linToSrgb(p.mat.color);
    const key = p.mat.unlit ? `u${p.cls}|${s.map((v) => Math.round(v * 6)).join(',')}`
      : p.cls === CLS.PAINT ? `${p.cls}|${p.paintKey && p.paintKey !== 'main' ? s.map((v) => Math.round(v * 6)).join(',') : 'main'}`
      : p.cls === CLS.LIGHT_CLEAR ? `${p.cls}|${isAmber(s)}`
      : FIXED.includes(p.cls) ? `${p.cls}` : `${p.cls}|${s.map((v) => Math.round(v * 6)).join(',')}`;
    if (!groups.has(key)) groups.set(key, { cls: p.cls, list: [], unlit: !!p.mat.unlit, main: key === `${CLS.PAINT}|main` });
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

  // windows are thin sheets, often double sided or modelled as two layers: their normals can
  // end up pointing into the car (dark AO, wrong reflection). Glass lies on the outside, so
  // its normals are turned to point away from the middle of the model.
  let yMid = 0;
  {
    let y0 = Infinity, y1 = -Infinity;
    for (const p of outParts) for (let i = 1; i < p.V.length; i += 3) { y0 = Math.min(y0, p.V[i]); y1 = Math.max(y1, p.V[i]); }
    yMid = (y0 + y1) / 2;
  }
  const allV = [], allN = [], allF = [], mats = [], details = [];
  let vo = 0, to = 0;
  for (const { cls, list, unlit, main } of groups.values()) {
    const j = joinParts(list);
    const cr = cls !== CLS.TIRE ? crease : 60;
    const cn = creaseNormals(j.V, j.F, cr);
    const Nn = transferNormals(cn.V, cn.N, hiGrid, hiN);
    if (cls === CLS.GLASS) outwardNormals(cn.V, Nn, yMid);
    const a = list.map((p) => faceAreas(p.V, p.F, false).reduce((s, v) => s + v, 0));
    const aw = a.map((v) => v + 1e-12), asum = aw.reduce((s, v) => s + v, 0);
    let color = [0, 1, 2].map((c) => list.reduce((s, p, i) => s + p.mat.color[c] * aw[i], 0) / asum);
    const rough = list.reduce((s, p, i) => s + (p.mat.roughness ?? 0.5) * aw[i], 0) / asum;
    if (cls === CLS.PAINT && paintLin && main) color = paintLin;
    allV.push(cn.V); allN.push(Nn);
    const Fo = new Uint32Array(cn.F.length);
    for (let k = 0; k < cn.F.length; k++) Fo[k] = cn.F[k] + vo;
    allF.push(Fo);
    const dsided = list.some((p) => p.twosided) || cls === CLS.GLASS;
    // texture detail: brightness changes inside a colour region go into the per-vertex light
    // factor (the region itself has one colour), so a textured surface keeps its shading
    let detail = null;
    if ((TEX_DETAIL_CLS.includes(cls) || unlit) && list.some((p) => p.texDetail) && !(cls === CLS.PAINT && paintLin && main)) {
      // look the colour up among the source vertices of this material only (a vertex on a
      // region border exists once per region, each with its own colour)
      const idx = [];
      for (const p of list) if (p.hiRange) for (let k = 0; k < p.hiRange[1]; k++) idx.push(p.hiRange[0] + k);
      const sub = new Float64Array(idx.length * 3);
      idx.forEach((k, i) => { sub[3 * i] = hiV[3 * k]; sub[3 * i + 1] = hiV[3 * k + 1]; sub[3 * i + 2] = hiV[3 * k + 2]; });
      const g2 = new PointGrid(sub, 1e-4);
      const nvg = cn.V.length / 3, lum = new Float64Array(nvg).fill(-1);
      for (let i = 0; i < nvg; i++) {
        const q = g2.nearest(cn.V[3 * i], cn.V[3 * i + 1], cn.V[3 * i + 2], 1e-5);
        const k = q >= 0 ? idx[q] : -1;
        if (k >= 0 && hiCol[3 * k] >= 0) lum[i] = hiCol[3 * k] * LUM[0] + hiCol[3 * k + 1] * LUM[1] + hiCol[3 * k + 2] * LUM[2];
      }
      const valid = [...lum].filter((v) => v >= 0).sort((x, y) => x - y);
      if (valid.length > 8) {
        const ref = valid[Math.floor(valid.length * 0.8)], mean = valid.reduce((s2, v) => s2 + v, 0) / valid.length;
        if (ref > 1e-4 && mean > 1e-4) {
          detail = new Float64Array(nvg);
          for (let i = 0; i < nvg; i++) detail[i] = lum[i] < 0 ? 1 : Math.min(1, Math.max(0.3, lum[i] / ref));
          // the region colour becomes the bright end; the factor darkens towards the texture
          const sc = Math.min(ref / mean, 1 / Math.max(color[0], color[1], color[2], 1e-6));
          color = color.map((c2) => c2 * sc);
        }
      }
    }
    details.push({ n: cn.V.length / 3, detail, unlit });
    mats.push({ cls, color, rough, unlit, triStart: to, triCount: cn.F.length / 3, dsided,
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
  {
    // unlit materials (KHR_materials_unlit, light baked into the texture): no AO on top
    let o = 0;
    details.forEach(({ n, detail, unlit }) => {
      if (unlit) for (let i = 0; i < n; i++) ao[o + i] = 1;
      if (detail) for (let i = 0; i < n; i++) ao[o + i] *= detail[i];
      o += n;
    });
  }

  // ---- matcaps
  log('  rendering matcaps ...');
  mats.forEach((m, i) => {
    progress('Rendering materials', i / mats.length);
    const size = CLS_MATCAP[m.cls] || 64;
    const r = renderMatcap(m.cls, m.color, size, m.rough, { unlit: m.unlit, object: !cfg._isCar });
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
