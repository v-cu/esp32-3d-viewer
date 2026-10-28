// matcap.js - procedural studio matcaps (port of env_* / render_matcap in model_convert.py)
import { linToSrgb1 } from './util.js';

export const CLS = {
  PAINT: 0, GLASS: 1, CHROME: 2, METAL: 3, TIRE: 4, PLASTIC: 5, LIGHT_RED: 6,
  LIGHT_CLEAR: 7, INTERIOR: 8, CARBON: 9, GENERIC: 10,
};
export const CLS_NAMES = ['paint', 'glass', 'chrome', 'metal', 'tire', 'plastic', 'light_red',
  'light_clear', 'interior', 'carbon', 'generic'];

// view-space studio environment; writes radiance of direction (x,y,z) into o[0..2]
function envRadiance(x, y, z, o) {
  // same terms as env_radiance() in model_convert.py; exp() of large negative arguments
  // (< e^-40) is skipped, the result is identical to float precision
  let r, g, b;
  if (y < 0) { r = g = b = 0.004; }
  else { r = g = b = 0.006 + 0.02 * (y > 1 ? 1 : y); }
  let t = (y - 0.03) / 0.05, t2 = t * t;
  if (t2 < 40) { const hz = Math.exp(-t2) * 0.12; r += hz * 0.85; g += hz * 0.9; b += hz; }
  if (z > -0.5) {                                  // long overhead softbox
    t = (y - 0.92) / 0.10; t2 = t * t;
    const ty = t2 * t2 * t2;
    if (ty < 40) {
      t = x / 0.75; t2 = t * t;
      const tx = t2 * t2 * t2;
      if (tx < 40) { const sb = Math.exp(-ty - tx) * 3.2; r += sb; g += sb; b += sb; }
    }
  }
  if (z > -0.3) {                                  // two strip lights left / right
    t = (y - 0.30) / 0.045; t2 = t * t;
    const ty = t2 * t2;
    if (ty < 40) {
      let s = 0;
      t = (-x - 0.80) / 0.25; t2 = t * t; let tx = t2 * t2;
      if (tx < 40) s += Math.exp(-ty - tx);
      t = (x - 0.80) / 0.25; t2 = t * t; tx = t2 * t2;
      if (tx < 40) s += Math.exp(-ty - tx);
      s *= 3.0; r += s; g += s; b += s;
    }
  }
  const kx = x - 0.42, ky = y - 0.52, kz = z - 0.74;
  const kd = (kx * kx + ky * ky + kz * kz) / 0.010;
  if (kd < 40) { const key = Math.exp(-kd) * 8.0; r += key; g += key; b += key; }
  const rx = x + 0.35, ry = y - 0.25, rz = z + 0.90;
  const rd = (rx * rx + ry * ry + rz * rz) / 0.06;
  if (rd < 40) { const rim = Math.exp(-rd) * 1.2; r += rim * 0.8; g += rim * 0.9; b += rim; }
  o[0] = r; o[1] = g; o[2] = b;
}

// grid of normals / reflection vectors for a supersampled matcap of side S
function grid(S) {
  const n = new Float64Array(S * S * 3), refl = new Float64Array(S * S * 3), ndv = new Float64Array(S * S);
  for (let j = 0; j < S; j++) {
    const Y = -((j + 0.5) / S * 2 - 1);
    for (let i = 0; i < S; i++) {
      const X = (i + 0.5) / S * 2 - 1;
      const r2 = X * X + Y * Y;
      let Xc = X, Yc = Y;
      if (r2 >= 1) { const s = Math.sqrt(Math.max(r2, 1e-12)); Xc = X / s * 0.999; Yc = Y / s * 0.999; }
      const Zc = Math.sqrt(Math.min(Math.max(1 - Xc * Xc - Yc * Yc, 0), 1));
      const k = j * S + i;
      n[3 * k] = Xc; n[3 * k + 1] = Yc; n[3 * k + 2] = Zc;
      const d = Math.min(Math.max(Zc, 0), 1);
      ndv[k] = d;
      let rx = 2 * d * Xc, ry = 2 * d * Yc, rz = 2 * d * Zc - 1;
      const l = Math.hypot(rx, ry, rz);
      refl[3 * k] = rx / l; refl[3 * k + 1] = ry / l; refl[3 * k + 2] = rz / l;
    }
  }
  return { n, refl, ndv };
}

const cache = new Map();
function cached(key, fn) {
  if (!cache.has(key)) cache.set(key, fn());
  return cache.get(key);
}

function envBlurred(dirs, rough, nsamp = 192) {
  const cnt = dirs.length / 3, out = new Float64Array(cnt * 3), o = [0, 0, 0];
  if (rough <= 0.01) {
    for (let k = 0; k < cnt; k++) {
      envRadiance(dirs[3 * k], dirs[3 * k + 1], dirs[3 * k + 2], o);
      out[3 * k] = o[0]; out[3 * k + 1] = o[1]; out[3 * k + 2] = o[2];
    }
    return out;
  }
  const sigma = rough * 0.9;
  const kx = new Float64Array(nsamp), ky = new Float64Array(nsamp), kw = new Float64Array(nsamp);
  let wsum = 0;
  for (let s = 0; s < nsamp; s++) {
    const i = s + 0.5;
    const r = 2.2 * sigma * Math.sqrt(i / nsamp);
    const th = i * Math.PI * (3 - Math.sqrt(5));
    kx[s] = r * Math.cos(th); ky[s] = r * Math.sin(th);
    kw[s] = Math.exp(-((r / sigma) ** 2));
    wsum += kw[s];
  }
  for (let k = 0; k < cnt; k++) {
    const dx = dirs[3 * k], dy = dirs[3 * k + 1], dz = dirs[3 * k + 2];
    const up = Math.abs(dy) < 0.99 ? [0, 1, 0] : [1, 0, 0];
    let tx = up[1] * dz - up[2] * dy, ty = up[2] * dx - up[0] * dz, tz = up[0] * dy - up[1] * dx;
    const tl = Math.sqrt(tx * tx + ty * ty + tz * tz); tx /= tl; ty /= tl; tz /= tl;
    const bx = dy * tz - dz * ty, by = dz * tx - dx * tz, bz = dx * ty - dy * tx;
    let ar = 0, ag = 0, ab = 0;
    for (let s = 0; s < nsamp; s++) {
      let ex = tx * kx[s] + bx * ky[s] + dx, ey = ty * kx[s] + by * ky[s] + dy, ez = tz * kx[s] + bz * ky[s] + dz;
      const l = Math.sqrt(ex * ex + ey * ey + ez * ez);
      envRadiance(ex / l, ey / l, ez / l, o);
      const w = kw[s];
      ar += o[0] * w; ag += o[1] * w; ab += o[2] * w;
    }
    out[3 * k] = ar / wsum; out[3 * k + 1] = ag / wsum; out[3 * k + 2] = ab / wsum;
  }
  return out;
}

function envIrradiance(normals, nsamp = 600) {
  const dirs = new Float64Array(nsamp * 3), L = new Float64Array(nsamp * 3), o = [0, 0, 0];
  for (let s = 0; s < nsamp; s++) {
    const i = s + 0.5;
    const phi = Math.acos(1 - 2 * i / nsamp);
    const th = Math.PI * (1 + Math.sqrt(5)) * i;
    const x = Math.cos(th) * Math.sin(phi), y = Math.sin(th) * Math.sin(phi), z = Math.cos(phi);
    dirs[3 * s] = x; dirs[3 * s + 1] = y; dirs[3 * s + 2] = z;
    envRadiance(x, y, z, o);
    L[3 * s] = o[0]; L[3 * s + 1] = o[1]; L[3 * s + 2] = o[2];
  }
  const cnt = normals.length / 3, out = new Float64Array(cnt * 3);
  const k = 4.0 / nsamp;
  for (let p = 0; p < cnt; p++) {
    const nx = normals[3 * p], ny = normals[3 * p + 1], nz = normals[3 * p + 2];
    let r = 0, g = 0, b = 0;
    for (let s = 0; s < nsamp; s++) {
      const c = nx * dirs[3 * s] + ny * dirs[3 * s + 1] + nz * dirs[3 * s + 2];
      if (c > 0) { r += c * L[3 * s]; g += c * L[3 * s + 1]; b += c * L[3 * s + 2]; }
    }
    out[3 * p] = r * k; out[3 * p + 1] = g * k; out[3 * p + 2] = b * k;
  }
  return out;
}

function tonemap1(c) {
  const a = 2.51, b = 0.03, cc = 2.43, d = 0.59, e = 0.14;
  c = (c * (a * c + b)) / (c * (cc * c + d) + e);
  return linToSrgb1(c < 0 ? 0 : c > 1 ? 1 : c);
}

const BAYER = [0, 8, 2, 10, 12, 4, 14, 6, 3, 11, 1, 9, 15, 7, 13, 5].map((v) => v / 16 - 0.5);

// returns { rgb565: Uint16Array(size*size), srgb: Float32Array(size*size*3) }
export function renderMatcap(cls, color, size, roughHint = 0.5, opts = {}) {
  const S = size * 2;
  const G = cached('g' + S, () => grid(S));
  const irr = cached('i' + S, () => envIrradiance(G.n));
  const blur = (r) => cached('b' + S + ':' + r, () => envBlurred(G.refl, r));
  const N = S * S, out = new Float64Array(N * 3);
  const c = color;
  const fres = (f0, k) => f0 + (1 - f0) * (1 - G.ndv[k]) ** 5;
  const clampc = (lo, hi) => c.map((v) => Math.min(Math.max(v, lo), hi));
  let E1, E2;
  if (opts.unlit) {
    // unlit material: the texture already holds the light; only a hint of form
    const rgb565 = new Uint16Array(size * size), srgb = new Float32Array(size * size * 3);
    for (let j = 0; j < size; j++)
      for (let i = 0; i < size; i++) {
        const k = (2 * j) * S + 2 * i, sh = 0.86 + 0.14 * G.ndv[k];
        const dith = BAYER[(j & 3) * 4 + (i & 3)] * 0.9;
        const px = c.map((v) => linToSrgb1(v * sh));
        srgb.set(px, 3 * (j * size + i));
        const r = Math.min(Math.max(Math.floor(px[0] * 31 + 0.5 + dith), 0), 31);
        const g = Math.min(Math.max(Math.floor(px[1] * 63 + 0.5 + dith), 0), 63);
        const b = Math.min(Math.max(Math.floor(px[2] * 31 + 0.5 + dith), 0), 31);
        rgb565[j * size + i] = (r << 11) | (g << 5) | b;
      }
    return { rgb565, srgb };
  }
  if (opts.object && (cls === CLS.GENERIC || cls === CLS.PLASTIC)) {
    // figures and objects: brighter, softer light than car trim so the colours read true
    const rough = Math.min(Math.max(roughHint, 0.25), 0.8);
    E1 = blur(Math.round(rough * 1000) / 1000);
    const base = cls === CLS.PLASTIC ? clampc(0.0, 0.35) : c;
    for (let k = 0; k < N; k++) {
      const f = fres(0.04, k);
      for (let ch = 0; ch < 3; ch++)
        out[3 * k + ch] = base[ch] * (irr[3 * k + ch] * 1.25 + 0.10) + f * E1[3 * k + ch] * (1 - rough * 0.7) * 0.7;
    }
    cls = -1;                                         // skip the switch below
  }
  switch (cls) {
    case -1: break;
    case CLS.PAINT: {
      E1 = blur(0.30); E2 = blur(0.035);
      for (let k = 0; k < N; k++) {
        const f = fres(0.04, k);
        for (let ch = 0; ch < 3; ch++)
          out[3 * k + ch] = c[ch] * irr[3 * k + ch] * 0.9 + c[ch] * E1[3 * k + ch] * 0.45 + f * E2[3 * k + ch] * 0.9;
      }
      break;
    }
    case CLS.GLASS: {
      E1 = blur(0.035);                               // a touch softer: no stair steps on flat glass
      const tint = [0.010, 0.012, 0.016];
      for (let k = 0; k < N; k++) {
        const f = fres(0.04, k);
        for (let ch = 0; ch < 3; ch++) out[3 * k + ch] = tint[ch] + f * E1[3 * k + ch];
      }
      break;
    }
    case CLS.CHROME: {
      E1 = blur(0.03);
      for (let k = 0; k < N * 3; k++) out[k] = E1[k] * 0.95 + 0.03;
      break;
    }
    case CLS.METAL: {
      E1 = blur(0.22);
      const base = clampc(0.35, 0.9);
      for (let k = 0; k < N; k++)
        for (let ch = 0; ch < 3; ch++) out[3 * k + ch] = base[ch] * E1[3 * k + ch] * 0.85 + base[ch] * irr[3 * k + ch] * 0.08;
      break;
    }
    case CLS.TIRE: {
      E1 = blur(0.6);
      const t = [0.018, 0.018, 0.02];
      for (let k = 0; k < N; k++) {
        const f = fres(0.04, k);
        for (let ch = 0; ch < 3; ch++) out[3 * k + ch] = t[ch] * irr[3 * k + ch] * 3 + f * E1[3 * k + ch] * 0.25;
      }
      break;
    }
    case CLS.PLASTIC: {
      E1 = blur(0.35);
      const base = clampc(0.01, 0.2);
      for (let k = 0; k < N; k++) {
        const f = fres(0.04, k);
        for (let ch = 0; ch < 3; ch++) out[3 * k + ch] = base[ch] * irr[3 * k + ch] * 0.6 + f * E1[3 * k + ch] * 0.5;
      }
      break;
    }
    case CLS.LIGHT_RED: {
      E1 = blur(0.03);
      const t = [0.45, 0.01, 0.01];
      for (let k = 0; k < N; k++) {
        const f = fres(0.06, k);
        for (let ch = 0; ch < 3; ch++) out[3 * k + ch] = t[ch] * (0.35 + irr[3 * k + ch] * 0.3) + f * E1[3 * k + ch] * 1.3;
      }
      break;
    }
    case CLS.LIGHT_CLEAR: {
      E1 = blur(0.06);
      const mx = Math.max(c[0], c[1], c[2]), mn = Math.min(c[0], c[1], c[2]);
      const tint = (mx - mn) > 0.2 * mx ? c.map((v) => v / Math.max(mx, 1e-6)) : [1, 1, 1];
      for (let k = 0; k < N; k++)
        for (let ch = 0; ch < 3; ch++) out[3 * k + ch] = E1[3 * k + ch] * 0.85 * tint[ch] + [0.08, 0.085, 0.09][ch] * tint[ch];
      break;
    }
    case CLS.INTERIOR: {
      const base = clampc(0.01, 0.3);
      for (let k = 0; k < N; k++)
        for (let ch = 0; ch < 3; ch++) out[3 * k + ch] = base[ch] * irr[3 * k + ch] * 0.45;
      break;
    }
    case CLS.CARBON: {
      E1 = blur(0.05);
      const t = [0.02, 0.02, 0.022];
      for (let k = 0; k < N; k++) {
        const f = fres(0.05, k);
        for (let ch = 0; ch < 3; ch++) out[3 * k + ch] = t[ch] * irr[3 * k + ch] * 2 + f * E1[3 * k + ch] * 0.9;
      }
      break;
    }
    default: {
      const rough = Math.min(Math.max(roughHint, 0.05), 0.8);
      E1 = blur(Math.round(rough * 1000) / 1000);
      for (let k = 0; k < N; k++) {
        const f = fres(0.04, k);
        for (let ch = 0; ch < 3; ch++) out[3 * k + ch] = c[ch] * irr[3 * k + ch] * 0.6 + f * E1[3 * k + ch] * (1 - rough * 0.7);
      }
    }
  }
  // 2x2 box filter, tonemap, dither to RGB565
  const rgb565 = new Uint16Array(size * size), srgb = new Float32Array(size * size * 3);
  for (let j = 0; j < size; j++)
    for (let i = 0; i < size; i++) {
      const dith = BAYER[(j & 3) * 4 + (i & 3)] * 0.9;
      const px = [0, 0, 0];
      for (let ch = 0; ch < 3; ch++) {
        const a = out[3 * ((2 * j) * S + 2 * i) + ch], b = out[3 * ((2 * j) * S + 2 * i + 1) + ch];
        const cc = out[3 * ((2 * j + 1) * S + 2 * i) + ch], d = out[3 * ((2 * j + 1) * S + 2 * i + 1) + ch];
        px[ch] = tonemap1((a + b + cc + d) / 4);
        srgb[3 * (j * size + i) + ch] = px[ch];
      }
      const r = Math.min(Math.max(Math.floor(px[0] * 31 + 0.5 + dith), 0), 31);
      const g = Math.min(Math.max(Math.floor(px[1] * 63 + 0.5 + dith), 0), 63);
      const b = Math.min(Math.max(Math.floor(px[2] * 31 + 0.5 + dith), 0), 31);
      rgb565[j * size + i] = (r << 11) | (g << 5) | b;
    }
  return { rgb565, srgb };
}
