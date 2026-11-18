// Serves firmware-format rows/frames (from equiv_test) through the real dashboard page as a
// manual recording, downloads the zip and writes it out for the replay check.
const { chromium } = require('playwright');
const fs = require('fs');
const D = process.argv[2];
const GRASP = process.argv[3] === 'grasp';   // grasp: the page records automatically from START   // work dir: page.html + chain/ from run.sh
const rowsAll = fs.readFileSync(D + '/chain/rec.csv', 'utf8').split('\n').filter(l => /^\d/.test(l));
const tofAll = fs.readFileSync(D + '/chain/rec_tof.csv', 'utf8').split('\n').filter(l => /^\d/.test(l));
const HDR = fs.readFileSync(D + '/chain/rec.csv', 'utf8').split('\n')[1];
(async () => {
  const b = await chromium.launch(); const ctx = await b.newContext({ acceptDownloads: true }); const p = await ctx.newPage();
  const errs = []; p.on('pageerror', e => errs.push(e.message));
  let go = false, t0 = 0;
  const avail = () => go ? Math.min(rowsAll.length, Math.floor((Date.now() - t0) / 2)) : 0;   // 500 rows/s
  await p.route('http://dev.local/', r => r.fulfill({ body: fs.readFileSync(D + '/page.html', 'utf8'), contentType: 'text/html' }));
  await p.route('**/api/status', r => r.fulfill({ json: { ok: true, state: 'IDLE', blockers: [], home: {}, object: {}, diag: { fw: 'chain-test' } } }));
  const cfg = { ok: true, tofMask0: 'ffffffffffffffff', tofMask1: 'ffffffffffffffff', tofMask2: 'ffffffffffffffff',
    mediumStandoffMm: 10, mediumForceN: 2, mediumMaxCurrent: 260, mediumGraspSpeed: 5, fsrFilter: 2, fsrFilterWindow: 5, fsrFilterAlpha: 0.3,
    contactForceN: 0.25, maxForceN: 8, slipFsrDropFrac: 0.3, slipFsrWindowMs: 300, slipFsrMinForceN: 0.15, slipFsrConfirmMs: 30,
    slipTofNearMm: 350, slipTofZoneMm: 12, slipTofZoneFrac: 0.1, slipTofZoneCount: 4, slipTofConfirmMs: 150, slipTofDispMm: 0,
    slipMode: 2, slipFuseWindowMs: 500, slipSettleMs: 500, slipHoldoffMs: 1500, slipTighten: false };
  await p.route('**/api/config', r => r.fulfill({ json: cfg }));
  await p.route('**/api/log/tof*', r => {
    const u = new URL(r.request().url()); let from = +u.searchParams.get('from');
    const head = tofAll.filter(l => +l.split(',')[2] <= avail()).length;
    if (from > head) from = head;
    const end = Math.min(head, from + 30);
    r.fulfill({ body: tofAll.slice(from, end).join('\n') + (end > from ? '\n' : ''), headers: { 'X-Log-Next': String(end), 'X-Log-Oldest': '0', 'X-Log-Boot': 'b1' } });
  });
  await p.route(u => new URL(u).pathname === '/api/log', r => {
    const u = new URL(r.request().url()); const from = +u.searchParams.get('from');
    const end = Math.min(avail(), from + 600);
    const lines = (from === 0 ? [HDR] : []).concat(rowsAll.slice(from, end));
    r.fulfill({ body: lines.join('\n') + '\n', headers: { 'X-Log-Next': String(end), 'X-Log-Oldest': '0', 'X-Log-Boot': 'b1' } });
  });
  await p.route(u => /\/api\/(fsm|servo|object|fsr|sweep)/.test(u.toString()), r => r.fulfill({ json: { ok: true } }));
  await p.goto('http://dev.local/'); await p.waitForTimeout(800);
  if (!GRASP) await p.click('#recMan');
  go = true; t0 = Date.now();
  await p.waitForTimeout(rowsAll.length * 2 + 2500);
  if (!GRASP) await p.click('#recMan');
  await p.waitForTimeout(500);
  const [dl] = await Promise.all([p.waitForEvent('download'), p.click('#recAll')]);
  fs.copyFileSync(await dl.path(), D + '/chain/out.zip');
  console.log('errors:', JSON.stringify(errs));
  await b.close();
})();
