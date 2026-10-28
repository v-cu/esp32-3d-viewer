// util.js - small numeric helpers shared by the converter (no DOM, runs in a worker)

export const LUM = [0.2126, 0.7152, 0.0722];

export function srgbToLin1(c) {
  return c <= 0.04045 ? c / 12.92 : Math.pow((c + 0.055) / 1.055, 2.4);
}
export function linToSrgb1(c) {
  c = c < 0 ? 0 : c > 1 ? 1 : c;
  return c <= 0.0031308 ? c * 12.92 : 1.055 * Math.pow(c, 1 / 2.4) - 0.055;
}
export const srgbToLin = (c) => c.map(srgbToLin1);
export const linToSrgb = (c) => c.map(linToSrgb1);
export const dot3 = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const ptp3 = (c) => Math.max(c[0], c[1], c[2]) - Math.min(c[0], c[1], c[2]);

export function srgbToOklab(r, g, b, out, o) {
  const lr = srgbToLin1(r), lg = srgbToLin1(g), lb = srgbToLin1(b);
  const l = Math.cbrt(Math.max(0.4122214708 * lr + 0.5363325363 * lg + 0.0514459929 * lb, 0));
  const m = Math.cbrt(Math.max(0.2119034982 * lr + 0.6806995451 * lg + 0.1073969566 * lb, 0));
  const s = Math.cbrt(Math.max(0.0883024619 * lr + 0.2817188376 * lg + 0.6299787005 * lb, 0));
  out[o] = 0.2104542553 * l + 0.7936177850 * m - 0.0040720468 * s;
  out[o + 1] = 1.9779984951 * l - 2.4285922050 * m + 0.4505937099 * s;
  out[o + 2] = 0.0259040371 * l + 0.7827717662 * m - 0.8086757660 * s;
}

// deterministic RNG (mulberry32)
export function rng(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6D2B79F5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

// --------------------------------------------------------------------------------------
// Open addressing hash for keys made of K int32 values -> dense ids (first occurrence order)
// --------------------------------------------------------------------------------------
export class IntKeyMap {
  constructor(K, expected) {
    this.K = K;
    let cap = 1024;
    while (cap < expected * 2) cap <<= 1;
    this.cap = cap;
    this.table = new Int32Array(cap).fill(-1);
    this.keys = new Int32Array(Math.max(16, expected) * K);
    this.size = 0;
  }
  _hash(key, o) {
    let h = 0x811C9DC5 | 0;
    for (let k = 0; k < this.K; k++) {
      h = Math.imul(h ^ key[o + k], 0x01000193);
      h ^= h >>> 13;
    }
    return (Math.imul(h, 0x5bd1e995) ^ (h >>> 15)) >>> 0;
  }
  _grow() {
    const K = this.K;
    if ((this.size + 1) * K > this.keys.length) {
      const nk = new Int32Array(this.keys.length * 2);
      nk.set(this.keys);
      this.keys = nk;
    }
    if (this.size * 2 >= this.cap) {
      this.cap <<= 1;
      this.table = new Int32Array(this.cap).fill(-1);
      const mask = this.cap - 1;
      for (let id = 0; id < this.size; id++) {
        let h = this._hash(this.keys, id * K) & mask;
        while (this.table[h] !== -1) h = (h + 1) & mask;
        this.table[h] = id;
      }
    }
  }
  // returns id (existing or new); key read from arr[o..o+K)
  getOrAdd(arr, o) {
    const K = this.K, mask = this.cap - 1, keys = this.keys;
    let h = this._hash(arr, o) & mask;
    for (;;) {
      const id = this.table[h];
      if (id === -1) break;
      let eq = true;
      for (let k = 0; k < K; k++) if (keys[id * K + k] !== arr[o + k]) { eq = false; break; }
      if (eq) return id;
      h = (h + 1) & mask;
    }
    this._grow();
    const id = this.size++;
    for (let k = 0; k < K; k++) this.keys[id * K + k] = arr[o + k];
    const m2 = this.cap - 1;
    let h2 = this._hash(arr, o) & m2;
    while (this.table[h2] !== -1) h2 = (h2 + 1) & m2;
    this.table[h2] = id;
    return id;
  }
  get(arr, o) {
    const K = this.K, mask = this.cap - 1, keys = this.keys;
    let h = this._hash(arr, o) & mask;
    for (;;) {
      const id = this.table[h];
      if (id === -1) return -1;
      let eq = true;
      for (let k = 0; k < K; k++) if (keys[id * K + k] !== arr[o + k]) { eq = false; break; }
      if (eq) return id;
      h = (h + 1) & mask;
    }
  }
}

// --------------------------------------------------------------------------------------
// Uniform grid for nearest neighbour queries within a radius
// --------------------------------------------------------------------------------------
export class PointGrid {
  constructor(P, cell) {        // P: Float64Array/Float32Array n*3
    const n = P.length / 3;
    this.P = P; this.cell = cell; this.inv = 1 / cell;
    const map = new IntKeyMap(3, n);
    const cid = new Int32Array(n);
    const key = new Int32Array(3);
    for (let i = 0; i < n; i++) {
      key[0] = Math.floor(P[3 * i] * this.inv);
      key[1] = Math.floor(P[3 * i + 1] * this.inv);
      key[2] = Math.floor(P[3 * i + 2] * this.inv);
      cid[i] = map.getOrAdd(key, 0);
    }
    const nc = map.size;
    const start = new Int32Array(nc + 1);
    for (let i = 0; i < n; i++) start[cid[i] + 1]++;
    for (let c = 0; c < nc; c++) start[c + 1] += start[c];
    const fill = start.slice(0, nc);
    const items = new Int32Array(n);
    for (let i = 0; i < n; i++) items[fill[cid[i]]++] = i;
    this.map = map; this.start = start; this.items = items;
    this.key = new Int32Array(3);
  }
  // calls cb(index) for every point within r of (x, y, z)
  within(x, y, z, r, cb) {
    const inv = this.inv, P = this.P, key = this.key, r2 = r * r;
    const k = Math.max(1, Math.ceil(r * inv));
    const cx = Math.floor(x * inv), cy = Math.floor(y * inv), cz = Math.floor(z * inv);
    for (let dx = -k; dx <= k; dx++)
      for (let dy = -k; dy <= k; dy++)
        for (let dz = -k; dz <= k; dz++) {
          key[0] = cx + dx; key[1] = cy + dy; key[2] = cz + dz;
          const c = this.map.get(key, 0);
          if (c < 0) continue;
          for (let q = this.start[c]; q < this.start[c + 1]; q++) {
            const i = this.items[q];
            const ex = P[3 * i] - x, ey = P[3 * i + 1] - y, ez = P[3 * i + 2] - z;
            if (ex * ex + ey * ey + ez * ez <= r2) cb(i);
          }
        }
  }

  // nearest point index within maxDist (or -1); returns distance in this.lastDist
  nearest(x, y, z, maxDist) {
    const inv = this.inv, P = this.P, key = this.key;
    const r = Math.max(1, Math.ceil(maxDist * inv));
    const cx = Math.floor(x * inv), cy = Math.floor(y * inv), cz = Math.floor(z * inv);
    let best = -1, bd = maxDist * maxDist;
    for (let dx = -r; dx <= r; dx++)
      for (let dy = -r; dy <= r; dy++)
        for (let dz = -r; dz <= r; dz++) {
          key[0] = cx + dx; key[1] = cy + dy; key[2] = cz + dz;
          const c = this.map.get(key, 0);
          if (c < 0) continue;
          for (let k = this.start[c]; k < this.start[c + 1]; k++) {
            const i = this.items[k];
            const ex = P[3 * i] - x, ey = P[3 * i + 1] - y, ez = P[3 * i + 2] - z;
            const d = ex * ex + ey * ey + ez * ez;
            if (d <= bd) { bd = d; best = i; }
          }
        }
    this.lastDist = best >= 0 ? Math.sqrt(bd) : Infinity;
    return best;
  }
}

// --------------------------------------------------------------------------------------
// Topology
// --------------------------------------------------------------------------------------
// vertex -> incident faces (CSR)
export function vertexFaces(F, nv) {
  const nf = F.length / 3;
  const start = new Int32Array(nv + 1);
  for (let i = 0; i < F.length; i++) start[F[i] + 1]++;
  for (let v = 0; v < nv; v++) start[v + 1] += start[v];
  const fill = start.slice(0, nv);
  const items = new Int32Array(F.length);
  for (let f = 0; f < nf; f++)
    for (let k = 0; k < 3; k++) items[fill[F[3 * f + k]]++] = f;
  return { start, items };
}

// pairs of faces sharing an edge (by vertex index): Int32Array [f0, g0, f1, g1, ...]
export function faceAdjacency(F, nv) {
  const nf = F.length / 3;
  const { start, items } = vertexFaces(F, nv);
  let out = new Int32Array(nf * 4);
  let n = 0;
  for (let f = 0; f < nf; f++) {
    for (let k = 0; k < 3; k++) {
      const a = F[3 * f + k], b = F[3 * f + (k + 1) % 3];
      // faces around a that also contain b, with index > f (each pair once)
      for (let s = start[a]; s < start[a + 1]; s++) {
        const g = items[s];
        if (g <= f) continue;
        const g0 = F[3 * g], g1 = F[3 * g + 1], g2 = F[3 * g + 2];
        if (g0 === b || g1 === b || g2 === b) {
          if (n + 2 > out.length) { const o2 = new Int32Array(out.length * 2); o2.set(out); out = o2; }
          out[n++] = f; out[n++] = g;
        }
      }
    }
  }
  return out.subarray(0, n);
}

export class UnionFind {
  constructor(n) { this.p = new Int32Array(n); for (let i = 0; i < n; i++) this.p[i] = i; }
  find(x) {
    const p = this.p;
    while (p[x] !== x) { p[x] = p[p[x]]; x = p[x]; }
    return x;
  }
  union(a, b) { a = this.find(a); b = this.find(b); if (a !== b) this.p[a] = b; }
  // dense component labels
  labels() {
    const n = this.p.length, lab = new Int32Array(n).fill(-1), out = new Int32Array(n);
    let c = 0;
    for (let i = 0; i < n; i++) {
      const r = this.find(i);
      if (lab[r] < 0) lab[r] = c++;
      out[i] = lab[r];
    }
    this.count = c;
    return out;
  }
}

export function faceAreas(V, F, half = true) {
  const nf = F.length / 3, A = new Float64Array(nf);
  for (let f = 0; f < nf; f++) {
    const a = 3 * F[3 * f], b = 3 * F[3 * f + 1], c = 3 * F[3 * f + 2];
    const ux = V[b] - V[a], uy = V[b + 1] - V[a + 1], uz = V[b + 2] - V[a + 2];
    const vx = V[c] - V[a], vy = V[c + 1] - V[a + 1], vz = V[c + 2] - V[a + 2];
    const nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
    A[f] = Math.sqrt(nx * nx + ny * ny + nz * nz) * (half ? 0.5 : 1);
  }
  return A;
}

// keep only the faces in mask, compact the vertices
export function subMesh(V, F, mask) {
  const nv = V.length / 3, nf = F.length / 3;
  const remap = new Int32Array(nv).fill(-1);
  let cnt = 0, fc = 0;
  for (let f = 0; f < nf; f++) if (mask[f]) fc++;
  const F2 = new Uint32Array(fc * 3);
  let j = 0;
  for (let f = 0; f < nf; f++) {
    if (!mask[f]) continue;
    for (let k = 0; k < 3; k++) {
      const v = F[3 * f + k];
      if (remap[v] < 0) remap[v] = cnt++;
      F2[j++] = remap[v];
    }
  }
  const V2 = new Float64Array(cnt * 3);
  for (let v = 0; v < nv; v++) {
    const r = remap[v];
    if (r >= 0) { V2[3 * r] = V[3 * v]; V2[3 * r + 1] = V[3 * v + 1]; V2[3 * r + 2] = V[3 * v + 2]; }
  }
  return { V: V2, F: F2, remap };
}

// compact unused vertices (faces index V)
export function compactMesh(V, F) {
  const all = new Uint8Array(F.length / 3).fill(1);
  return subMesh(V, F, all);
}

export function weld(V, F, tol) {
  const nv = V.length / 3, nf = F.length / 3;
  const map = new IntKeyMap(3, nv);
  const key = new Int32Array(3);
  const inv = new Int32Array(nv);
  const inv_t = 1 / tol;
  for (let i = 0; i < nv; i++) {
    key[0] = Math.round(V[3 * i] * inv_t);
    key[1] = Math.round(V[3 * i + 1] * inv_t);
    key[2] = Math.round(V[3 * i + 2] * inv_t);
    inv[i] = map.getOrAdd(key, 0);
  }
  const n2 = map.size;
  const V2 = new Float64Array(n2 * 3);
  const seen = new Uint8Array(n2);
  for (let i = 0; i < nv; i++) {
    const r = inv[i];
    if (seen[r]) continue;
    seen[r] = 1;
    V2[3 * r] = V[3 * i]; V2[3 * r + 1] = V[3 * i + 1]; V2[3 * r + 2] = V[3 * i + 2];
  }
  const F2 = new Uint32Array(nf * 3);
  let j = 0;
  for (let f = 0; f < nf; f++) {
    const a = inv[F[3 * f]], b = inv[F[3 * f + 1]], c = inv[F[3 * f + 2]];
    if (a === b || b === c || a === c) continue;
    F2[j++] = a; F2[j++] = b; F2[j++] = c;
  }
  return { V: V2, F: F2.slice(0, j), inv };
}

// remove triangles that use the same 3 vertices (double-sided duplicates)
export function dedupeFaces(F) {
  const nf = F.length / 3;
  const map = new IntKeyMap(3, nf);
  const key = new Int32Array(3);
  const out = new Uint32Array(F.length);
  let j = 0;
  for (let f = 0; f < nf; f++) {
    let a = F[3 * f], b = F[3 * f + 1], c = F[3 * f + 2], t;
    if (a > b) { t = a; a = b; b = t; }
    if (b > c) { t = b; b = c; c = t; }
    if (a > b) { t = a; a = b; b = t; }
    key[0] = a; key[1] = b; key[2] = c;
    const before = map.size;
    map.getOrAdd(key, 0);
    if (map.size === before) continue;
    out[j++] = F[3 * f]; out[j++] = F[3 * f + 1]; out[j++] = F[3 * f + 2];
  }
  return out.slice(0, j);
}

// make winding consistent inside every edge-connected patch (like trimesh.repair.fix_winding)
export function fixWinding(F, nv) {
  const nf = F.length / 3;
  const adj = faceAdjacency(F, nv);
  // CSR face -> neighbours
  const start = new Int32Array(nf + 1);
  for (let i = 0; i < adj.length; i++) start[adj[i] + 1]++;
  for (let f = 0; f < nf; f++) start[f + 1] += start[f];
  const fill = start.slice(0, nf), nb = new Int32Array(adj.length);
  for (let i = 0; i < adj.length; i += 2) {
    nb[fill[adj[i]]++] = adj[i + 1];
    nb[fill[adj[i + 1]]++] = adj[i];
  }
  const done = new Uint8Array(nf), queue = new Int32Array(nf);
  // same direction edge a->b in both faces => inconsistent
  const hasDirEdge = (g, a, b) => {
    const g0 = F[3 * g], g1 = F[3 * g + 1], g2 = F[3 * g + 2];
    return (g0 === a && g1 === b) || (g1 === a && g2 === b) || (g2 === a && g0 === b);
  };
  let flipped = 0;
  for (let s = 0; s < nf; s++) {
    if (done[s]) continue;
    done[s] = 1;
    let qh = 0, qt = 0;
    queue[qt++] = s;
    while (qh < qt) {
      const f = queue[qh++];
      for (let k = start[f]; k < start[f + 1]; k++) {
        const g = nb[k];
        if (done[g]) continue;
        // shared edge of f (directed as in f)
        let inconsistent = false, shared = 0;
        for (let e = 0; e < 3; e++) {
          const a = F[3 * f + e], b = F[3 * f + (e + 1) % 3];
          const g0 = F[3 * g], g1 = F[3 * g + 1], g2 = F[3 * g + 2];
          const hasA = g0 === a || g1 === a || g2 === a, hasB = g0 === b || g1 === b || g2 === b;
          if (hasA && hasB) { shared++; if (hasDirEdge(g, a, b)) inconsistent = true; }
        }
        if (shared !== 1) continue;            // degenerate / duplicate: skip
        if (inconsistent) {
          const t = F[3 * g + 1]; F[3 * g + 1] = F[3 * g + 2]; F[3 * g + 2] = t;
          flipped++;
        }
        done[g] = 1;
        queue[qt++] = g;
      }
    }
  }
  return flipped;
}

// connected components of faces over shared edges -> labels
export function faceComponents(F, nv) {
  const nf = F.length / 3;
  const uf = new UnionFind(nf);
  const adj = faceAdjacency(F, nv);
  for (let i = 0; i < adj.length; i += 2) uf.union(adj[i], adj[i + 1]);
  const lab = uf.labels();
  return { labels: lab, count: uf.count };
}

export function concatF64(list) {
  let n = 0;
  for (const a of list) n += a.length;
  const out = new Float64Array(n);
  let o = 0;
  for (const a of list) { out.set(a, o); o += a.length; }
  return out;
}
