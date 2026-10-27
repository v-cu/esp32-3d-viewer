// raster.js - orthographic ID/depth rasterizer: hidden surface removal and AO baking
// (port of Raster / visible_triangles / bake_ao in tools/model_convert.py)

export function fibSphere(n, minElev = -1) {
  const out = [];
  for (let k = 0; k < n; k++) {
    const i = k + 0.5;
    const phi = Math.acos(1 - 2 * i / n);
    const th = Math.PI * (1 + Math.sqrt(5)) * i;
    const d = [Math.cos(th) * Math.sin(phi), Math.cos(phi), Math.sin(th) * Math.sin(phi)];
    if (d[1] >= minElev) out.push(d);
  }
  return out;
}

function basisFromDir(d) {
  const l = Math.hypot(d[0], d[1], d[2]);
  d = [d[0] / l, d[1] / l, d[2] / l];
  const up = Math.abs(d[1]) < 0.95 ? [0, 1, 0] : [1, 0, 0];
  let x = [up[1] * d[2] - up[2] * d[1], up[2] * d[0] - up[0] * d[2], up[0] * d[1] - up[1] * d[0]];
  const xl = Math.hypot(x[0], x[1], x[2]);
  x = [x[0] / xl, x[1] / xl, x[2] / xl];
  const y = [d[1] * x[2] - d[2] * x[1], d[2] * x[0] - d[0] * x[2], d[0] * x[1] - d[1] * x[0]];
  return [x, y, d];
}

export class Raster {
  constructor(res, radius) {
    this.res = res;
    this.radius = radius;
    this.pix = 2 * radius / res;
    this.zb = new Float32Array(res * res);
    this.ib = new Int32Array(res * res);
  }

  toScreen(P, d, sx, sy, sz) {
    const [x, y, z] = basisFromDir(d);
    const n = P.length / 3, ip = 1 / this.pix, h = this.res / 2;
    for (let i = 0; i < n; i++) {
      const px = P[3 * i], py = P[3 * i + 1], pz = P[3 * i + 2];
      sx[i] = (px * x[0] + py * x[1] + pz * x[2]) * ip + h;
      sy[i] = (px * y[0] + py * y[1] + pz * y[2]) * ip + h;
      sz[i] = -(px * z[0] + py * z[1] + pz * z[2]);
    }
  }

  // fills this.zb / this.ib for direction d
  render(V, F, d) {
    const res = this.res, nv = V.length / 3, nf = F.length / 3;
    if (!this.sx || this.sx.length < nv) {
      this.sx = new Float64Array(nv); this.sy = new Float64Array(nv); this.sz = new Float64Array(nv);
    }
    const sx = this.sx, sy = this.sy, sz = this.sz;
    this.toScreen(V, d, sx, sy, sz);
    const zb = this.zb, ib = this.ib;
    zb.fill(Infinity);
    ib.fill(-1);
    const R1 = res - 1;
    for (let t = 0; t < nf; t++) {
      const i0 = F[3 * t], i1 = F[3 * t + 1], i2 = F[3 * t + 2];
      const x0 = sx[i0], x1 = sx[i1], x2 = sx[i2];
      const y0 = sy[i0], y1 = sy[i1], y2 = sy[i2];
      const area = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0);
      if (!(Math.abs(area) > 1e-12)) continue;
      let bx0 = Math.floor(Math.min(x0, x1, x2) - 0.5), by0 = Math.floor(Math.min(y0, y1, y2) - 0.5);
      let bx1 = Math.ceil(Math.max(x0, x1, x2) - 0.5), by1 = Math.ceil(Math.max(y0, y1, y2) - 0.5);
      bx0 = bx0 < 0 ? 0 : bx0 > R1 ? R1 : bx0; by0 = by0 < 0 ? 0 : by0 > R1 ? R1 : by0;
      bx1 = bx1 < 0 ? 0 : bx1 > R1 ? R1 : bx1; by1 = by1 < 0 ? 0 : by1 > R1 ? R1 : by1;
      const ia = 1 / area;
      const z0 = sz[i0], z1 = sz[i1], z2 = sz[i2];
      // w0 = ((x1-cx)(y2-cy) - (x2-cx)(y1-cy)) / a, linear in cx, cy
      const a0x = (y1 - y2) * ia, a0y = (x2 - x1) * ia, a0c = (x1 * y2 - x2 * y1) * ia;
      const a1x = (y2 - y0) * ia, a1y = (x0 - x2) * ia, a1c = (x2 * y0 - x0 * y2) * ia;
      for (let py = by0; py <= by1; py++) {
        const cy = py + 0.5;
        const r0 = a0y * cy + a0c, r1 = a1y * cy + a1c;
        const row = py * res;
        for (let px = bx0; px <= bx1; px++) {
          const cx = px + 0.5;
          const w0 = a0x * cx + r0;
          if (w0 < -1e-9) continue;
          const w1 = a1x * cx + r1;
          if (w1 < -1e-9) continue;
          const w2 = 1 - w0 - w1;
          if (w2 < -1e-9) continue;
          const z = w0 * z0 + w1 * z1 + w2 * z2;
          const idx = row + px;
          if (z <= zb[idx]) { zb[idx] = z; ib[idx] = t; }
        }
      }
    }
  }

  // depth buffer value under point P (screen truncation like the numpy version) + its depth
  depthAt(P, d, outZb, outD) {
    const [x, y, z] = basisFromDir(d);
    const n = P.length / 3, ip = 1 / this.pix, h = this.res / 2, R1 = this.res - 1;
    for (let i = 0; i < n; i++) {
      const px = P[3 * i], py = P[3 * i + 1], pz = P[3 * i + 2];
      let ix = Math.trunc((px * x[0] + py * x[1] + pz * x[2]) * ip + h);
      let iy = Math.trunc((px * y[0] + py * y[1] + pz * y[2]) * ip + h);
      ix = ix < 0 ? 0 : ix > R1 ? R1 : ix;
      iy = iy < 0 ? 0 : iy > R1 ? R1 : iy;
      outZb[i] = this.zb[iy * this.res + ix];
      outD[i] = -(px * z[0] + py * z[1] + pz * z[2]);
    }
  }
}

// Exact visibility from many directions. Returns {vis, front, back}.
export function visibleTriangles(V, F, radius, nViews = 64, res = 640, minElev = -0.35, progress) {
  const ras = new Raster(res, radius);
  const nf = F.length / 3;
  const vis = new Uint8Array(nf), front = new Int32Array(nf), back = new Int32Array(nf);
  const fn = new Float64Array(nf * 3), C = new Float64Array(nf * 3);
  for (let f = 0; f < nf; f++) {
    const a = 3 * F[3 * f], b = 3 * F[3 * f + 1], c = 3 * F[3 * f + 2];
    const ux = V[b] - V[a], uy = V[b + 1] - V[a + 1], uz = V[b + 2] - V[a + 2];
    const vx = V[c] - V[a], vy = V[c + 1] - V[a + 1], vz = V[c + 2] - V[a + 2];
    fn[3 * f] = uy * vz - uz * vy; fn[3 * f + 1] = uz * vx - ux * vz; fn[3 * f + 2] = ux * vy - uy * vx;
    C[3 * f] = (V[a] + V[b] + V[c]) / 3; C[3 * f + 1] = (V[a + 1] + V[b + 1] + V[c + 1]) / 3;
    C[3 * f + 2] = (V[a + 2] + V[b + 2] + V[c + 2]) / 3;
  }
  const seen = new Uint8Array(nf), zc = new Float32Array(nf), dc = new Float64Array(nf);
  const dirs = fibSphere(nViews, minElev);
  dirs.forEach((d, k) => {
    ras.render(V, F, d);
    seen.fill(0);
    const ib = ras.ib;
    for (let i = 0; i < ib.length; i++) if (ib[i] >= 0) seen[ib[i]] = 1;
    ras.depthAt(C, d, zc, dc);
    const tol = ras.pix * 0.5;
    for (let f = 0; f < nf; f++) {
      if (!seen[f] && dc[f] <= zc[f] + tol) seen[f] = 1;
      if (!seen[f]) continue;
      vis[f] = 1;
      const facing = fn[3 * f] * d[0] + fn[3 * f + 1] * d[1] + fn[3 * f + 2] * d[2];
      if (facing > 0) front[f]++;
      else if (facing < 0) back[f]++;
    }
    if (progress) progress((k + 1) / dirs.length);
  });
  return { vis, front, back };
}

export function bakeAO(V, N, F, radius, nDirs = 128, res = 512, ground = true, progress) {
  const ras = new Raster(res, radius);
  const nv = V.length / 3;
  const Pv = new Float64Array(nv * 3);
  const off = ras.pix * 1.5;
  for (let i = 0; i < nv * 3; i++) Pv[i] = V[i] + N[i] * off;
  const acc = new Float64Array(nv), wsum = new Float64Array(nv);
  const zv = new Float32Array(nv), dv = new Float64Array(nv);
  const dirs = fibSphere(nDirs);
  dirs.forEach((d, k) => {
    if (ground && d[1] < 0.02) {
      for (let i = 0; i < nv; i++) wsum[i] += Math.max(N[3 * i] * d[0] + N[3 * i + 1] * d[1] + N[3 * i + 2] * d[2], 0);
      return;
    }
    ras.render(V, F, d);
    ras.depthAt(Pv, d, zv, dv);
    const tol = ras.pix;
    for (let i = 0; i < nv; i++) {
      const w = Math.max(N[3 * i] * d[0] + N[3 * i + 1] * d[1] + N[3 * i + 2] * d[2], 0);
      if (dv[i] <= zv[i] + tol) acc[i] += w;
      wsum[i] += w;
    }
    if (progress) progress((k + 1) / dirs.length);
  });
  const ao = new Float64Array(nv);
  for (let i = 0; i < nv; i++) {
    let a = acc[i] / Math.max(wsum[i], 1e-9);
    a = Math.min(Math.max(a * 1.35, 0), 1) ** 0.7;
    ao[i] = 0.22 + 0.78 * a;
  }
  return ao;
}
