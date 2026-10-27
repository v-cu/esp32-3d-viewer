// installer.js - "Install the firmware" section
import { Flasher, webSerialSupported } from './flasher.js';

const $ = (id) => document.getElementById(id);

export function setProgress(el, frac, text) {
  el.hidden = false;
  el.querySelector('.fill').style.width = `${Math.round(Math.max(0, Math.min(1, frac)) * 100)}%`;
  el.querySelector('.label').textContent = text;
}

export function setStatus(el, text, kind) {
  el.textContent = text;
  el.className = 'status' + (kind ? ' ' + kind : '');
}

export function makeLogger(pre) {
  return (s, inline) => {
    pre.textContent += inline ? s : (pre.textContent ? '\n' : '') + s;
    pre.scrollTop = pre.scrollHeight;
  };
}

async function fetchBin(path) {
  const r = await fetch(path, { cache: 'no-cache' });
  if (!r.ok) throw new Error(`Download of ${path} failed (${r.status})`);
  return new Uint8Array(await r.arrayBuffer());
}

export function initInstaller(manifest) {
  const btn = $('btn-install'), info = $('fw-info'), status = $('install-status'), prog = $('install-progress');
  const log = makeLogger($('install-log'));

  if (!manifest || !manifest.parts) {
    info.textContent = 'No firmware build is published on this site yet. Build it with ESP-IDF (see the README) or check back later.';
    return;
  }
  const date = manifest.date ? new Date(manifest.date).toLocaleDateString(undefined, { year: 'numeric', month: 'long', day: 'numeric' }) : '';
  info.textContent = `Firmware ${manifest.version || ''}${date ? ', built ' + date : ''}. ` +
    'Writes the bootloader, the partition table and the application; the demo models are optional.';
  if (!webSerialSupported()) { btn.textContent = 'Needs Chrome or Edge'; return; }
  btn.disabled = false;

  btn.addEventListener('click', async () => {
    btn.disabled = true;
    const flasher = new Flasher(log);
    try {
      setStatus(status, 'Downloading the firmware…');
      const files = [];
      for (const p of manifest.parts) files.push({ name: p.name, address: p.offset, data: await fetchBin(p.file) });
      if ($('opt-models').checked && manifest.pack) {
        files.push({ name: 'Demo models', address: manifest.pack.offset, data: await fetchBin(manifest.pack.file) });
      }
      setStatus(status, 'Choose the board in the browser dialog…');
      const { chip, flashSize } = await flasher.connect();
      log(`Connected: ${chip}, flash ${flashSize || 'unknown'}`);
      if (flashSize && flashSize !== '16MB') {
        throw new Error(`This board has ${flashSize} of flash; the AMOLED-1.8 V2 has 16 MB. Is it the right board?`);
      }
      const erase = $('opt-erase').checked;
      if (erase) setStatus(status, 'Erasing the flash (about 20 s)…');
      setProgress(prog, 0, erase ? 'Erasing…' : 'Starting…');
      await flasher.write(files, (f, i) => {
        setProgress(prog, f, `${files[i].name}: ${Math.round(f * 100)} %`);
        setStatus(status, `Writing ${files[i].name.toLowerCase()}…`);
      }, erase);
      setProgress(prog, 1, 'Done');
      await flasher.resetAndClose();
      setStatus(status, 'Installed. The board restarts and shows the first model.', 'ok');
    } catch (e) {
      await flasher.disconnect();
      if (e && e.name === 'NotFoundError') setStatus(status, 'No board was chosen.', '');
      else { setStatus(status, e.message || String(e), 'bad'); log('ERROR: ' + (e.message || e)); }
    } finally {
      btn.disabled = false;
    }
  });
}
