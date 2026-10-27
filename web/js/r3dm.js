// r3dm.js - R3DM model (one model) and R3DP pack (several models) binary formats.
// Layout mirrored from firmware/components/r3d/include/r3d_model.h and tools/pack_models.py.

export const R3DM_VERSION = 2;
export const HDR_SIZE = 128;
export const MAT_SIZE = 12;
export const PACK_MAX_MODELS = 32;
export const MODEL_PARTITION_OFFSET = 0x310000;
export const MODEL_PARTITION_SIZE = 0xCF0000;
const PACK_ENTRY = 32;              // u32 offset, u32 size, char name[24]

let CRC_TABLE = null;
export function crc32(bytes, start = 0, end = bytes.length) {
  if (!CRC_TABLE) {
    CRC_TABLE = new Uint32Array(256);
    for (let n = 0; n < 256; n++) {
      let c = n;
      for (let k = 0; k < 8; k++) c = c & 1 ? 0xEDB88320 ^ (c >>> 1) : c >>> 1;
      CRC_TABLE[n] = c >>> 0;
    }
  }
  let c = 0xFFFFFFFF;
  for (let i = start; i < end; i++) c = CRC_TABLE[(c ^ bytes[i]) & 0xFF] ^ (c >>> 8);
  return (c ^ 0xFFFFFFFF) >>> 0;
}

const al16 = (x) => (x + 15) & ~15;
const enc = new TextEncoder();
const dec = new TextDecoder();

function cstr(bytes, o, n) {
  let e = o;
  while (e < o + n && bytes[e] !== 0) e++;
  return dec.decode(bytes.subarray(o, e));
}

// UTF-8 bytes of s, truncated to at most maxBytes without cutting a character
export function utf8Truncate(s, maxBytes) {
  let b = enc.encode(s);
  if (b.length <= maxBytes) return b;
  let n = maxBytes;
  while (n > 0 && (b[n] & 0xC0) === 0x80) n--;
  return b.subarray(0, n);
}

/*
 * Write an R3DM blob.
 * lod: { V: Float64Array(n*3), N: Float64Array(n*3), F: Uint32Array(m*3), ao: Float64Array(n), tm: Uint8Array(m) }
 * mats: [{ cls, dsided, matcapSize, rgb565: Uint16Array }]
 */
export function writeR3DM(lod, mats, radius, name) {
  const lods = [lod];
  const nvAll = lod.V.length / 3;
  const mx = [0, 0, 0];
  let ymax = -Infinity;
  for (let i = 0; i < nvAll; i++) {
    for (let k = 0; k < 3; k++) mx[k] = Math.max(mx[k], Math.abs(lod.V[3 * i + k]));
    ymax = Math.max(ymax, lod.V[3 * i + 1]);
  }
  const scale = mx.map((m) => Math.fround((m + 1e-9) / 32767));
  const scale64 = mx.map((m) => (m + 1e-9) / 32767);

  let off = al16(HDR_SIZE + MAT_SIZE * mats.length);
  const sections = [], lodEntries = [];
  for (const L of lods) {
    const nv = L.V.length / 3, nf = L.F.length / 3;
    if (nv > 65535) throw new Error(`Too many vertices (${nv}); lower the triangle budget`);
    const vb = new Uint8Array(nv * 10), dv = new DataView(vb.buffer);
    for (let i = 0; i < nv; i++) {
      for (let k = 0; k < 3; k++) {
        let q = Math.round(L.V[3 * i + k] / scale64[k]);
        q = Math.max(-32768, Math.min(32767, q));
        dv.setInt16(i * 10 + 2 * k, q, true);
      }
      for (let k = 0; k < 3; k++) {
        let q = Math.round(L.N[3 * i + k] * 127);
        dv.setInt8(i * 10 + 6 + k, Math.max(-127, Math.min(127, q)));
      }
      dv.setUint8(i * 10 + 9, Math.max(0, Math.min(255, Math.round(L.ao[i] * 255))));
    }
    const ib = new Uint8Array(nf * 6), di = new DataView(ib.buffer);
    for (let i = 0; i < nf * 3; i++) di.setUint16(2 * i, L.F[i], true);
    const tb = new Uint8Array(L.tm);
    const ov = off; off = al16(off + vb.length);
    const oi = off; off = al16(off + ib.length);
    const ot = off; off = al16(off + tb.length);
    sections.push([ov, vb], [oi, ib], [ot, tb]);
    lodEntries.push([ov, oi, ot, nv, 0, nf]);
  }
  while (lodEntries.length < 2) lodEntries.push(lodEntries[0]);
  const mcOffsets = [];
  for (const m of mats) {
    const b = new Uint8Array(m.rgb565.length * 2), d = new DataView(b.buffer);
    for (let i = 0; i < m.rgb565.length; i++) d.setUint16(2 * i, m.rgb565[i], true);
    mcOffsets.push(off);
    sections.push([off, b]);
    off = al16(off + b.length);
  }
  const total = off;
  const body = new Uint8Array(total);
  const dvb = new DataView(body.buffer);
  mats.forEach((m, i) => {
    const o = HDR_SIZE + MAT_SIZE * i;
    dvb.setUint8(o, m.cls);
    dvb.setUint8(o + 1, m.dsided ? 1 : 0);
    dvb.setUint8(o + 2, Math.round(Math.log2(m.matcapSize)));
    dvb.setUint8(o + 3, 0);
    dvb.setUint32(o + 4, mcOffsets[i], true);
    dvb.setUint16(o + 8, m.matcapSize, true);
    dvb.setUint16(o + 10, 0, true);
  });
  for (const [o, b] of sections) body.set(b, o);
  const crc = crc32(body, HDR_SIZE, total);
  body.set(enc.encode('R3DM'), 0);
  dvb.setUint16(4, R3DM_VERSION, true);
  dvb.setUint16(6, HDR_SIZE, true);
  dvb.setUint32(8, total, true);
  dvb.setUint32(12, crc, true);
  dvb.setUint16(16, mats.length, true);
  dvb.setUint16(18, lods.length, true);
  dvb.setFloat32(20, scale[0], true); dvb.setFloat32(24, scale[1], true); dvb.setFloat32(28, scale[2], true);
  dvb.setFloat32(32, radius, true);
  dvb.setFloat32(36, ymax, true);
  dvb.setUint32(40, HDR_SIZE, true);
  for (let l = 0; l < 2; l++) {
    const o = 44 + 20 * l, e = lodEntries[l];
    dvb.setUint32(o, e[0], true); dvb.setUint32(o + 4, e[1], true); dvb.setUint32(o + 8, e[2], true);
    dvb.setUint16(o + 12, e[3], true); dvb.setUint16(o + 14, 0, true); dvb.setUint32(o + 16, e[5], true);
  }
  body.set(utf8Truncate(name || 'model', 31), 84);
  return body;
}

/* Parse an R3DM blob (for the preview). Throws on errors. */
export function parseR3DM(bytes, checkCrc = true) {
  if (bytes.length < HDR_SIZE || cstr(bytes, 0, 4) !== 'R3DM') throw new Error('not an R3DM model');
  const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const version = dv.getUint16(4, true);
  if (version !== R3DM_VERSION) throw new Error(`unsupported R3DM version ${version}`);
  const total = dv.getUint32(8, true);
  if (total > bytes.length) throw new Error('model file is truncated');
  if (checkCrc && crc32(bytes, HDR_SIZE, total) !== dv.getUint32(12, true)) throw new Error('model CRC mismatch');
  const nm = dv.getUint16(16, true);
  const scale = [dv.getFloat32(20, true), dv.getFloat32(24, true), dv.getFloat32(28, true)];
  const radius = dv.getFloat32(32, true), ymax = dv.getFloat32(36, true);
  const offMat = dv.getUint32(40, true);
  const o = 44;
  const lod = {
    offV: dv.getUint32(o, true), offI: dv.getUint32(o + 4, true), offT: dv.getUint32(o + 8, true),
    nv: dv.getUint16(o + 12, true), nf: dv.getUint32(o + 16, true),
  };
  const name = cstr(bytes, 84, 32);
  const mats = [];
  for (let i = 0; i < nm; i++) {
    const m = offMat + MAT_SIZE * i;
    const size = dv.getUint16(m + 8, true), mo = dv.getUint32(m + 4, true);
    const rgb565 = new Uint16Array(size * size);
    for (let k = 0; k < size * size; k++) rgb565[k] = dv.getUint16(mo + 2 * k, true);
    mats.push({ cls: dv.getUint8(m), dsided: !!(dv.getUint8(m + 1) & 1), size, rgb565 });
  }
  const pos = new Float32Array(lod.nv * 3), nrm = new Float32Array(lod.nv * 3), ao = new Float32Array(lod.nv);
  for (let i = 0; i < lod.nv; i++) {
    const b = lod.offV + 10 * i;
    for (let k = 0; k < 3; k++) {
      pos[3 * i + k] = dv.getInt16(b + 2 * k, true) * scale[k];
      nrm[3 * i + k] = dv.getInt8(b + 6 + k) / 127;
    }
    ao[i] = dv.getUint8(b + 9) / 255;
  }
  const idx = new Uint32Array(lod.nf * 3);
  for (let i = 0; i < lod.nf * 3; i++) idx[i] = dv.getUint16(lod.offI + 2 * i, true);
  const trimat = bytes.slice(lod.offT, lod.offT + lod.nf);
  return { name, version, total, radius, ymax, scale, mats, pos, nrm, ao, idx, trimat,
    numVertices: lod.nv, numTriangles: lod.nf };
}

/* quick header info without decoding everything */
export function peekR3DM(bytes) {
  if (bytes.length < HDR_SIZE || cstr(bytes, 0, 4) !== 'R3DM') return null;
  const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  return {
    name: cstr(bytes, 84, 32), total: dv.getUint32(8, true),
    numVertices: dv.getUint16(56, true), numTriangles: dv.getUint32(60, true),
    numMaterials: dv.getUint16(16, true),
  };
}

/* models: [{ bytes: Uint8Array (R3DM), name }] -> R3DP pack */
export function buildPack(models) {
  if (models.length === 0) throw new Error('no models');
  if (models.length > PACK_MAX_MODELS) throw new Error(`at most ${PACK_MAX_MODELS} models`);
  const al4k = (x) => (x + 4095) & ~4095;
  const hdrSize = 8 + PACK_ENTRY * models.length;
  let total = al4k(hdrSize);
  for (const m of models) total += al4k(m.bytes.length);
  const out = new Uint8Array(total).fill(0xFF);
  const dv = new DataView(out.buffer);
  out.set(enc.encode('R3DP'), 0);
  dv.setUint16(4, 1, true);
  dv.setUint16(6, models.length, true);
  let off = al4k(hdrSize);
  models.forEach((m, i) => {
    const e = 8 + PACK_ENTRY * i;
    dv.setUint32(e, off, true);
    dv.setUint32(e + 4, m.bytes.length, true);
    const nm = new Uint8Array(24);                  // zero padded like struct.pack("24s")
    nm.set(utf8Truncate(m.name || peekR3DM(m.bytes)?.name || 'model', 23));
    out.set(nm, e + 8);
    out.set(m.bytes, off);
    off += al4k(m.bytes.length);
  });
  return out;
}

/* R3DP pack -> [{ name, bytes }] ; a single R3DM file -> one entry */
export function splitPack(bytes) {
  const magic = cstr(bytes, 0, 4);
  if (magic === 'R3DM') return [{ name: peekR3DM(bytes).name, bytes }];
  if (magic !== 'R3DP') throw new Error('not an R3DM model or R3DP pack');
  const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const n = dv.getUint16(6, true), out = [];
  for (let i = 0; i < n; i++) {
    const e = 8 + PACK_ENTRY * i;
    const off = dv.getUint32(e, true), size = dv.getUint32(e + 4, true);
    if (off + size > bytes.length) throw new Error('pack is truncated');
    out.push({ name: cstr(bytes, e + 8, 24), bytes: bytes.slice(off, off + size) });
  }
  return out;
}
