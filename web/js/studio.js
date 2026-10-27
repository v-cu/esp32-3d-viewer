// studio.js - "Model studio": drop models, convert them in a worker, order them into a pack,
// preview, download, write the model partition.
import { ModelViewer } from './viewer.js';
import { loadModel, isModelFile } from './loader.js';
import { peekR3DM, buildPack, splitPack, utf8Truncate, MODEL_PARTITION_OFFSET, MODEL_PARTITION_SIZE,
  PACK_MAX_MODELS } from './r3dm.js';
import { Flasher, webSerialSupported } from './flasher.js';
import { setProgress, setStatus } from './installer.js';

const $ = (id) => document.getElementById(id);
const CLS_LABEL = {
  paint: 'paint', glass: 'glass', chrome: 'chrome', metal: 'metal', tire: 'tyre', plastic: 'plastic',
  light_red: 'tail light', light_clear: 'light', interior: 'interior', carbon: 'carbon', generic: 'surface',
};
const fmtKB = (n) => (n >= 1024 * 1024 ? `${(n / 1048576).toFixed(1)} MB` : `${Math.max(1, Math.round(n / 1024))} KB`);
const fmtInt = (n) => n.toLocaleString('en-US').replace(/,/g, ' ');
const PACK_SIZE = (items) => {
  const al = (x) => (x + 4095) & ~4095;
  return al(8 + 32 * items.length) + items.reduce((s, it) => s + al(it.bytes ? it.bytes.length : 0), 0);
};
const DEFAULT_OPTS = { budget: 10000, yaw: '', up: 'y', kind: 'auto', texColors: 12, paint: null, keepHidden: false };

// ---------------------------------------------------------------------------------------
// worker queue (one conversion at a time keeps memory use sane)
// ---------------------------------------------------------------------------------------
let worker = null, jobId = 0;
const queue = [];
let running = false;

function getWorker() {
  if (!worker) worker = new Worker(new URL('./convert/worker.js', import.meta.url), { type: 'module' });
  return worker;
}

function runJob(parts, opts, onMsg) {
  return new Promise((resolve, reject) => {
    queue.push({ parts, opts, onMsg, resolve, reject });
    pump();
  });
}

function pump() {
  if (running || !queue.length) return;
  running = true;
  const job = queue.shift();
  const w = getWorker(), id = ++jobId;
  const done = () => { w.removeEventListener('message', handler); w.removeEventListener('error', onErr); running = false; pump(); };
  const handler = (e) => {
    const m = e.data;
    if (m.id !== id) return;
    if (m.type === 'done') { done(); job.resolve(m); }
    else if (m.type === 'error') { done(); job.reject(new Error(m.message)); }
    else job.onMsg(m);
  };
  const onErr = (e) => {
    done();
    worker.terminate(); worker = null;               // a crashed worker is replaced
    job.reject(new Error(e.message || 'The converter stopped (out of memory?)'));
  };
  w.addEventListener('message', handler);
  w.addEventListener('error', onErr);
  w.postMessage({ id, parts: job.parts, opts: job.opts });
}

// ---------------------------------------------------------------------------------------
// persistence: converted models survive a page reload (IndexedDB, per browser)
// ---------------------------------------------------------------------------------------
const DB = 'esp32-3d-viewer', STORE = 'models';
function db() {
  return new Promise((res, rej) => {
    const r = indexedDB.open(DB, 1);
    r.onupgradeneeded = () => r.result.createObjectStore(STORE);
    r.onsuccess = () => res(r.result);
    r.onerror = () => rej(r.error);
  });
}
async function saveList(items) {
  try {
    const d = await db();
    const data = items.filter((it) => it.bytes).map((it) => ({ name: it.name, bytes: it.bytes, report: it.report || null }));
    await new Promise((res, rej) => {
      const tx = d.transaction(STORE, 'readwrite');
      tx.objectStore(STORE).put(data, 'list');
      tx.oncomplete = res; tx.onerror = () => rej(tx.error);
    });
  } catch { /* storage not available: fine */ }
}
async function loadList() {
  try {
    const d = await db();
    return await new Promise((res) => {
      const r = d.transaction(STORE).objectStore(STORE).get('list');
      r.onsuccess = () => res(r.result || []);
      r.onerror = () => res([]);
    });
  } catch { return []; }
}

// ---------------------------------------------------------------------------------------
export function initStudio(manifest, site) {
  const items = [];
  let selected = null, viewer = null, uid = 0, flashing = false;
  const list = $('models'), statusEl = $('models-status'), prog = $('models-progress');
  const dlog = $('d-log');

  const ensureViewer = () => {
    if (!viewer) {
      try { viewer = new ModelViewer($('studio-screen'), { autoRotate: true }); }
      catch { $('studio-empty').textContent = 'WebGL is not available, no preview.'; }
    }
    return viewer;
  };

  const persist = () => saveList(items);

  // ---- list rendering
  function render() {
    list.textContent = '';
    items.forEach((it, i) => {
      const li = document.createElement('li');
      li.setAttribute('aria-selected', String(it === selected));
      li.tabIndex = 0;
      const main = document.createElement('div');
      main.className = 'm-main';
      const nm = document.createElement('div');
      nm.className = 'm-name';
      nm.textContent = it.name;
      const meta = document.createElement('div');
      meta.className = 'm-meta' + (it.status === 'error' ? ' bad' : '');
      meta.textContent = metaText(it);
      main.append(nm, meta);
      const tools = document.createElement('div');
      tools.className = 'm-tools';
      const tb = (label, title, fn, dis) => {
        const b = document.createElement('button');
        b.className = 'icon-btn'; b.type = 'button'; b.textContent = label; b.title = title; b.setAttribute('aria-label', title);
        b.disabled = !!dis;
        b.addEventListener('click', (e) => { e.stopPropagation(); fn(); });
        tools.appendChild(b);
      };
      tb('↑', 'Move up', () => move(i, -1), i === 0);
      tb('↓', 'Move down', () => move(i, 1), i === items.length - 1);
      tb('✕', 'Remove', () => remove(it));
      li.append(main, tools);
      if (it.status === 'converting' || it.status === 'loading') {
        const bar = document.createElement('div');
        bar.className = 'm-bar';
        const fill = document.createElement('div');
        fill.style.width = `${Math.round((it.progress || 0) * 100)}%`;
        bar.appendChild(fill);
        li.appendChild(bar);
        it._fill = fill; it._meta = meta;
      }
      li.addEventListener('click', () => select(it));
      li.addEventListener('keydown', (e) => {
        if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); select(it); }
        if (e.altKey && e.key === 'ArrowUp') { e.preventDefault(); move(i, -1); }
        if (e.altKey && e.key === 'ArrowDown') { e.preventDefault(); move(i, 1); }
      });
      list.appendChild(li);
    });
    const ready = items.filter((it) => it.bytes);
    $('pack-count').textContent = items.length ? `${items.length} model${items.length > 1 ? 's' : ''} in the pack` : 'No models yet';
    const size = ready.length ? PACK_SIZE(ready) : 0;
    const full = size > MODEL_PARTITION_SIZE || ready.length > PACK_MAX_MODELS;
    $('meter-fill').style.width = `${Math.min(100, size / MODEL_PARTITION_SIZE * 100)}%`;
    $('meter').classList.toggle('full', full);
    $('meter-text').textContent = `${fmtKB(size)} of ${fmtKB(MODEL_PARTITION_SIZE)}`;
    const busy = items.some((it) => it.status === 'converting' || it.status === 'loading');
    $('btn-download-pack').disabled = !ready.length || full || busy;
    $('btn-flash-models').disabled = !ready.length || full || busy || !webSerialSupported() || flashing;
    if (full) setStatus(statusEl, ready.length > PACK_MAX_MODELS ? `At most ${PACK_MAX_MODELS} models fit in a pack.`
      : 'The models do not fit in the partition. Remove one or convert with fewer triangles.', 'bad');
  }

  function metaText(it) {
    if (it.status === 'loading') return 'Reading the file…';
    if (it.status === 'converting') return `${it.stage || 'Converting'}…`;
    if (it.status === 'error') return it.error;
    const r = it.info;
    return r ? `${fmtInt(r.numTriangles)} triangles, ${fmtKB(it.bytes.length)}` : fmtKB(it.bytes.length);
  }

  function updateProgress(it) {
    if (it._fill) it._fill.style.width = `${Math.round((it.progress || 0) * 100)}%`;
    if (it._meta) it._meta.textContent = metaText(it);
  }

  function move(i, d) {
    const j = i + d;
    if (j < 0 || j >= items.length) return;
    [items[i], items[j]] = [items[j], items[i]];
    render(); persist();
    list.children[j]?.focus();
  }

  function remove(it) {
    const i = items.indexOf(it);
    if (i < 0) return;
    items.splice(i, 1);
    if (selected === it) select(items[Math.min(i, items.length - 1)] || null);
    render(); persist();
  }

  // ---- selection / detail panel
  function select(it) {
    selected = it;
    render();
    const detail = $('detail');
    if (!it) {
      detail.hidden = true;
      viewer?.clear();
      $('studio-empty').hidden = false;
      return;
    }
    detail.hidden = false;
    $('d-name').value = it.name;
    dlog.textContent = (it.log || []).join('\n');
    $('d-settings').hidden = !it.parts;
    fillSettings(it.opts || DEFAULT_OPTS);
    $('d-download').disabled = !it.bytes;
    if (it.bytes) {
      const v = ensureViewer();
      $('studio-empty').hidden = true;
      try { if (v) v.show(it.bytes); } catch (e) { $('d-stats').textContent = e.message; }
      showReport(it);
    } else {
      viewer?.clear();
      $('studio-empty').hidden = false;
      $('studio-empty').textContent = it.status === 'error' ? it.error : 'Converting…';
      $('d-stats').textContent = it.status === 'error' ? '' : 'Converting…';
      $('d-mats').textContent = '';
    }
  }

  function showReport(it) {
    const r = it.info;
    const parts = [`${fmtInt(r.numTriangles)} triangles`, `${fmtInt(r.numVertices)} vertices`, `${r.numMaterials} materials`, fmtKB(it.bytes.length)];
    if (it.report?.mode) parts.push(it.report.mode === 'car' ? 'converted as a car' : 'converted as an object');
    $('d-stats').textContent = parts.join(', ');
    const ul = $('d-mats');
    ul.textContent = '';
    for (const m of it.report?.materials || []) {
      const li = document.createElement('li');
      const sw = document.createElement('i');
      sw.style.background = `rgb(${m.color.map((c) => Math.round(c * 255)).join(',')})`;
      li.append(sw, CLS_LABEL[m.cls] || m.cls);
      li.title = `${fmtInt(m.tris)} triangles from: ${(m.src || []).join(', ')}`;
      ul.appendChild(li);
    }
  }

  // ---- settings form
  const S = {
    budget: $('s-budget'), yaw: $('s-yaw'), up: $('s-up'), kind: $('s-kind'), tex: $('s-tex'),
    paintOn: $('s-paint-on'), paint: $('s-paint'), hidden: $('s-hidden'),
  };
  S.budget.addEventListener('input', () => { $('s-budget-out').textContent = fmtInt(+S.budget.value); });
  S.tex.addEventListener('input', () => { $('s-tex-out').textContent = S.tex.value === '0' ? 'off' : S.tex.value; });
  function fillSettings(o) {
    S.budget.value = o.budget; $('s-budget-out').textContent = fmtInt(o.budget);
    S.yaw.value = o.yaw ?? ''; S.up.value = o.up; S.kind.value = o.kind;
    S.tex.value = o.texColors; $('s-tex-out').textContent = o.texColors ? String(o.texColors) : 'off';
    S.paintOn.checked = !!o.paint; if (o.paint) S.paint.value = o.paint;
    S.hidden.checked = !!o.keepHidden;
  }
  function readSettings() {
    return {
      budget: +S.budget.value, yaw: S.yaw.value, up: S.up.value, kind: S.kind.value, texColors: +S.tex.value,
      paint: S.paintOn.checked ? S.paint.value : null, keepHidden: S.hidden.checked,
    };
  }
  S.paint.addEventListener('input', () => { S.paintOn.checked = true; });

  $('d-reconvert').addEventListener('click', () => {
    if (!selected || !selected.parts) return;
    selected.opts = readSettings();
    convertItem(selected);
  });

  $('d-name').addEventListener('change', () => {
    if (!selected) return;
    const v = $('d-name').value.trim() || 'model';
    selected.name = new TextDecoder().decode(utf8Truncate(v, 23));
    $('d-name').value = selected.name;
    if (selected.bytes) renameBlob(selected.bytes, selected.name);
    render(); persist();
  });

  // the name in the R3DM header is outside the CRC range: patch it in place
  function renameBlob(bytes, name) {
    bytes.fill(0, 84, 116);
    bytes.set(utf8Truncate(name, 31), 84);
  }

  $('d-download').addEventListener('click', () => {
    if (selected?.bytes) download(selected.bytes, `${safeFile(selected.name)}.bin`);
  });

  // ---- conversion
  async function convertItem(it) {
    it.status = 'converting'; it.progress = 0; it.stage = 'Starting'; it.log = [];
    it.bytes = null;
    if (it === selected) select(it); else render();
    const o = it.opts;
    const config = { up_axis: o.up, kind: o.kind };
    if (o.yaw === 'flip') config.flip_front = true;
    else if (o.yaw !== '' && o.yaw !== undefined) config.yaw_deg = Number(o.yaw);
    const stages = ['Classifying materials', 'Removing hidden geometry', 'Simplifying', 'Baking ambient occlusion', 'Rendering materials'];
    const weight = [0.04, 0.46, 0.2, 0.12, 0.18];
    try {
      const res = await runJob(it.parts, {
        name: it.name, budget: o.budget, texColors: o.texColors, paint: o.paint, keepHidden: o.keepHidden, config,
      }, (m) => {
        if (m.type === 'log') { it.log.push(m.text); if (it === selected) { dlog.textContent += (dlog.textContent ? '\n' : '') + m.text; } }
        if (m.type === 'progress') {
          const k = Math.max(0, stages.indexOf(m.stage));
          it.stage = m.stage;
          it.progress = weight.slice(0, k).reduce((s, v) => s + v, 0) + weight[k] * m.frac;
          updateProgress(it);
        }
      });
      it.bytes = res.bytes;
      it.report = res.report;
      it.info = peekR3DM(it.bytes);
      it.status = 'ready';
      persist();
    } catch (e) {
      it.status = 'error';
      it.error = e.message || String(e);
      it.log.push('ERROR: ' + it.error);
    }
    if (it === selected) select(it); else render();
  }

  // ---- adding files
  async function addFiles(fileList) {
    const files = [...fileList];
    if (!files.length) return;
    setStatus(statusEl, '');
    const models = files.filter((f) => isModelFile(f.name));
    const bins = files.filter((f) => /\.bin$/i.test(f.name));
    const used = new Set();
    // converted models / packs (.bin that are not glTF buffers)
    for (const f of bins) {
      const bytes = new Uint8Array(await f.arrayBuffer());
      const magic = String.fromCharCode(...bytes.subarray(0, 4));
      if (magic !== 'R3DM' && magic !== 'R3DP') continue;           // glTF buffer, used below
      used.add(f);
      try {
        for (const e of splitPack(bytes)) addReady(e.name, e.bytes);
      } catch (e) { setStatus(statusEl, `${f.name}: ${e.message}`, 'bad'); }
    }
    const others = files.filter((f) => !used.has(f));
    if (!models.length) {
      if (!used.size) setStatus(statusEl, 'Choose a .glb, .gltf or .obj file (for .gltf / .obj together with its other files).', 'bad');
      render();
      return;
    }
    for (const mf of models) {
      // a .gltf / .obj takes the non-model files dropped with it; a .glb is self-contained
      const group = /\.glb$/i.test(mf.name) ? [mf] : [mf, ...others.filter((f) => !isModelFile(f.name))];
      const it = { id: ++uid, name: mf.name.replace(/\.[^.]+$/, '').slice(0, 23), status: 'loading', progress: 0, opts: { ...DEFAULT_OPTS }, log: [] };
      it.name = new TextDecoder().decode(utf8Truncate(it.name, 23));
      items.push(it);
      if (!selected || selected.status !== 'converting') selected = it;
      render();
      if (it === selected) select(it);
      try {
        const m = await loadModel(group);
        it.parts = m.parts;
        it.log.push(`Loaded ${m.file}: ${m.parts.length} parts, ${fmtInt(m.triangles)} triangles`);
        if (m.missing.length) it.log.push(`Missing files (not dropped): ${m.missing.join(', ')}`);
        convertItem(it);
      } catch (e) {
        it.status = 'error';
        it.error = e.message || String(e);
        if (it === selected) select(it); else render();
      }
    }
  }

  function addReady(name, bytes, report = null) {
    const info = peekR3DM(bytes);
    if (!info) return null;
    const it = { id: ++uid, name: name || info.name, status: 'ready', bytes, info, report, log: [] };
    items.push(it);
    if (!selected) selected = it;
    return it;
  }

  // ---- drag & drop
  const drop = $('drop');
  ['dragenter', 'dragover'].forEach((t) => drop.addEventListener(t, (e) => { e.preventDefault(); drop.classList.add('over'); }));
  ['dragleave', 'drop'].forEach((t) => drop.addEventListener(t, () => drop.classList.remove('over')));
  drop.addEventListener('drop', (e) => { e.preventDefault(); addFiles(e.dataTransfer.files); });
  $('file-input').addEventListener('change', (e) => { addFiles(e.target.files); e.target.value = ''; });
  // dropping anywhere on the studio works too
  const studio = $('studio');
  studio.addEventListener('dragover', (e) => { if ([...e.dataTransfer.types].includes('Files')) e.preventDefault(); });
  studio.addEventListener('drop', (e) => { if (e.target.closest('#drop')) return; e.preventDefault(); addFiles(e.dataTransfer.files); });

  // ---- demo models
  const demos = (manifest && manifest.models) || [];
  if (!demos.length) $('btn-demo').hidden = true;
  $('btn-demo').addEventListener('click', async () => {
    try {
      for (const d of demos) {
        if (items.some((it) => it.demo === d.file)) continue;
        const it = addReady(d.name, (await site.demoBytes(d)).slice());
        if (it) it.demo = d.file;
      }
      select(selected || items[0]);
      persist();
    } catch (e) { setStatus(statusEl, e.message, 'bad'); }
  });

  // ---- pack: download / flash
  const readyItems = () => items.filter((it) => it.bytes);
  const pack = () => buildPack(readyItems().map((it) => ({ name: it.name, bytes: it.bytes })));

  $('btn-download-pack').addEventListener('click', () => download(pack(), 'model.bin'));

  $('btn-flash-models').addEventListener('click', async () => {
    flashing = true; render();
    const flasher = new Flasher((s) => console.debug('[flasher]', s));
    try {
      const data = pack();
      setStatus(statusEl, 'Choose the board in the browser dialog…');
      await flasher.connect();
      setProgress(prog, 0, 'Starting…');
      setStatus(statusEl, `Writing ${readyItems().length} models (${fmtKB(data.length)})…`);
      await flasher.write([{ name: 'Models', address: MODEL_PARTITION_OFFSET, data }], (f) => setProgress(prog, f, `${Math.round(f * 100)} %`));
      setProgress(prog, 1, 'Done');
      await flasher.resetAndClose();
      setStatus(statusEl, 'Written. The board restarts with the new models; triple tap to switch between them.', 'ok');
    } catch (e) {
      await flasher.disconnect();
      if (e && e.name === 'NotFoundError') setStatus(statusEl, 'No board was chosen.');
      else setStatus(statusEl, e.message || String(e), 'bad');
    } finally {
      flashing = false; render();
    }
  });

  // ---- restore the previous session
  loadList().then((saved) => {
    if (items.length) return;
    for (const s of saved) {
      if (!(s.bytes instanceof Uint8Array)) continue;
      addReady(s.name, s.bytes, s.report);
    }
    if (items.length) select(items[0]);
    else render();
  });
  render();
}

function safeFile(name) { return (name || 'model').replace(/[^\w\- .]+/g, '_').trim() || 'model'; }

function download(bytes, name) {
  const url = URL.createObjectURL(new Blob([bytes], { type: 'application/octet-stream' }));
  const a = document.createElement('a');
  a.href = url; a.download = name;
  document.body.appendChild(a); a.click(); a.remove();
  setTimeout(() => URL.revokeObjectURL(url), 10000);
}
