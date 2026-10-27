// flasher.js - thin wrapper around esptool-js (Web Serial)
import { ESPLoader, Transport } from '../vendor/esptool-js/bundle.js';
import { md5hex } from './md5.js';

export const webSerialSupported = () => 'serial' in navigator;

// Espressif native USB, WCH (CH34x), Silicon Labs (CP210x), FTDI
const FILTERS = [0x303a, 0x1a86, 0x10c4, 0x0403].map((usbVendorId) => ({ usbVendorId }));

export class Flasher {
  constructor(log) {
    this.log = log || (() => {});
    this.loader = null;
    this.transport = null;
    this.chip = '';
  }

  get connected() { return !!this.loader; }

  async connect() {
    if (!webSerialSupported()) throw new Error('This browser has no Web Serial. Use Chrome or Edge on a computer.');
    const port = await navigator.serial.requestPort({ filters: FILTERS });
    this.transport = new Transport(port, false);
    const term = {
      clean: () => {},
      writeLine: (s) => this.log(s),
      write: (s) => this.log(s, true),
    };
    this.loader = new ESPLoader({ transport: this.transport, baudrate: 921600, romBaudrate: 115200, terminal: term });
    try {
      this.chip = await this.loader.main();
    } catch (e) {
      await this.disconnect();
      throw new Error('The board did not answer. Hold BOOT, press and release RESET (or plug the cable in ' +
        'while holding BOOT), then try again. (' + (e.message || e) + ')');
    }
    if (!/ESP32-S3/i.test(this.chip)) {
      const c = this.chip;
      await this.disconnect();
      throw new Error(`This is a ${c}. The viewer needs the ESP32-S3 on the Waveshare AMOLED-1.8 board.`);
    }
    try { this.flashSize = await this.loader.detectFlashSize(); } catch { this.flashSize = undefined; }
    return { chip: this.chip, flashSize: this.flashSize };
  }

  /* files: [{ data: Uint8Array, address: number, name }]; onProgress(fraction, fileIndex) */
  async write(files, onProgress, eraseAll = false) {
    if (!this.loader) throw new Error('Not connected');
    const total = files.reduce((s, f) => s + f.data.length, 0);
    const before = files.map((_, i) => files.slice(0, i).reduce((s, f) => s + f.data.length, 0));
    await this.loader.writeFlash({
      fileArray: files.map((f) => ({ data: f.data, address: f.address })),
      flashMode: 'keep', flashFreq: 'keep', flashSize: 'keep',
      eraseAll, compress: true,
      reportProgress: (i, written, size) => {
        if (onProgress) onProgress((before[i] + files[i].data.length * (size ? written / size : 1)) / total, i);
      },
      calculateMD5Hash: (img) => md5hex(img),
    });
  }

  async resetAndClose() {
    if (this.loader) {
      try { await this.loader.after('hard_reset'); } catch (e) { this.log('reset: ' + (e.message || e)); }
    }
    await this.disconnect();
  }

  async disconnect() {
    try { if (this.transport) await this.transport.disconnect(); } catch { /* already closed */ }
    this.transport = null;
    this.loader = null;
  }
}
