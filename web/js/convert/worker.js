// worker.js - runs the converter off the main thread.
// in:  { id, parts, opts }   out: { id, type: 'log'|'progress'|'done'|'error', ... }
import { convert } from './core.js';

self.onmessage = async (e) => {
  const { id, parts, opts } = e.data;
  const post = (msg, transfer) => self.postMessage({ id, ...msg }, transfer || []);
  try {
    const r = await convert(parts, opts, {
      log: (text) => post({ type: 'log', text }),
      progress: (stage, frac) => post({ type: 'progress', stage, frac }),
    });
    post({ type: 'done', bytes: r.bytes, report: r.report }, [r.bytes.buffer]);
  } catch (err) {
    post({ type: 'error', message: err && err.message ? err.message : String(err) });
  }
};
