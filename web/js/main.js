// main.js - page bootstrap: hero preview, firmware manifest, installer and model studio
import { ModelViewer } from './viewer.js';
import { webSerialSupported } from './flasher.js';
import { initInstaller } from './installer.js';
import { initStudio } from './studio.js';

const $ = (id) => document.getElementById(id);

export const site = {
  manifest: null,
  demoCache: new Map(),
  async demoBytes(entry) {
    if (!this.demoCache.has(entry.file)) {
      const r = await fetch(entry.file);
      if (!r.ok) throw new Error(`Could not load ${entry.file} (${r.status})`);
      this.demoCache.set(entry.file, new Uint8Array(await r.arrayBuffer()));
    }
    return this.demoCache.get(entry.file);
  },
};

async function loadManifest() {
  try {
    const r = await fetch('firmware/manifest.json', { cache: 'no-cache' });
    if (!r.ok) throw new Error(String(r.status));
    return await r.json();
  } catch {
    return null;
  }
}

function webglOk() {
  try { return !!document.createElement('canvas').getContext('webgl2'); } catch { return false; }
}

async function initHero(manifest) {
  const screen = $('hero-screen'), chips = $('hero-chips');
  const demos = (manifest && manifest.models) || [];
  if (!demos.length || !webglOk()) {
    screen.innerHTML = '<p class="empty">Preview not available</p>';
    return;
  }
  const viewer = new ModelViewer(screen, { autoRotate: true });
  const show = async (i) => {
    try {
      viewer.show(await site.demoBytes(demos[i]));
      [...chips.children].forEach((c, k) => c.setAttribute('aria-pressed', String(k === i)));
    } catch (e) {
      $('hero-caption').textContent = e.message;
    }
  };
  demos.forEach((d, i) => {
    const b = document.createElement('button');
    b.className = 'chip';
    b.type = 'button';
    b.textContent = d.name;
    b.addEventListener('click', () => show(i));
    chips.appendChild(b);
  });
  show(0);
}

async function main() {
  if (!webSerialSupported()) $('browser-notice').hidden = false;
  const manifest = await loadManifest();
  site.manifest = manifest;
  initHero(manifest);
  initInstaller(manifest);
  initStudio(manifest, site);
}

main();
