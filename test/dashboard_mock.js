'use strict';
// Local UI test fixture only. Never connects to hardware or reads credentials.
const http = require('node:http');
const fs = require('node:fs');
const state = { mode: 'idle', reason: 'mock_only', controller: false,
  commissioned: false, lidarHealthy: true, drive: 0, turn: 0, coverage: 360,
  scanAgeMs: 25, frontMm: 2000, rearMm: 2500, packets: 1234, crcErrors: 0,
  heap: 140000, uptime: 60, version: '0.2.0 MOCK', manualLimit: 100, lidarFrontDeg: 180 };
let scanSequence = 0;
state.settings={manualStopMm:200,manualRotateMm:200,autoStopMm:500,autoRotateMm:550,
 autoSlowMm:1200,autoDrivePercent:30,autoTurnPercent:30,autoSlowPercent:20,rampMs:1000,
 reverseMs:800,turnMs:1600,resumeMarginMm:50,robotWidthMm:250,autoLimitS:300};
function scan() {
  return {fresh: true, rpm: 600, ageMs: 25, sequence: ++scanSequence,
    distance: Array.from({length: 360}, (_, a) => 2000 + Math.round(400 * Math.sin(a / 20)))};
}
let heartbeat = 0, motion = 0;
const commands = [];
const server = http.createServer((req, res) => {
  res.setHeader('Cache-Control', 'no-store');
  if (req.url === '/') {
    res.setHeader('Content-Type', 'text/html; charset=utf-8');
    return res.end(fs.readFileSync('src/web/index.html'));
  }
  res.setHeader('Content-Type', 'application/json');
  if (req.url === '/api/status') return res.end(JSON.stringify(state));
  if (req.url === '/api/telemetry') return res.end(JSON.stringify({...state, scan: scan()}));
  if (req.url === '/api/scan') return res.end(JSON.stringify(scan()));
  if (req.url === '/test/commands') return res.end(JSON.stringify(commands));
  if (req.url !== '/api/control' || req.method !== 'POST') {res.statusCode = 404; return res.end('{}');}
  let body = '';
  req.on('data', data => {body += data; if (body.length > 1024) req.destroy();});
  req.on('end', () => {
    try {
      const b = JSON.parse(body);
      commands.push(b);
      if (commands.length > 100) commands.shift();
      if (b.command === 'claim') {
        if (state.controller) {res.statusCode = 409; return res.end('{}');}
        state.controller = true; heartbeat = Date.now();
        return res.end(JSON.stringify({ok: true, token: 123}));
      }
      if (b.command === 'estop') {state.mode = 'estop'; state.drive = state.turn = 0;}
      else if (b.token !== 123 || !state.controller) {res.statusCode = 409; return res.end('{}');}
      else {
        heartbeat = Date.now();
        if (b.command === 'mode' && !['estop','fault'].includes(state.mode)) state.mode = b.mode;
        if (b.command === 'move' && state.mode === 'manual') {
          state.drive = b.drive; state.turn = b.turn; motion = Date.now();
        }
        if (b.command === 'reset' || b.command === 'stop' || b.command === 'release') {
          if (b.command === 'reset' || !['estop','fault'].includes(state.mode)) state.mode = 'idle';
          state.drive = state.turn = 0;
        }
        if (b.command === 'release') state.controller = false;
        if (b.command === 'commission') state.commissioned = true;
        if (b.command === 'settings') {
          if(state.mode!=='idle'){res.statusCode=409;return res.end('{}');}
          for(const name of Object.keys(state.settings))state.settings[name]=b[name];
          state.commissioned=false;state.reason='settings_updated';
        }
      }
      res.end(JSON.stringify({ok: true}));
    } catch {res.statusCode = 400; res.end('{}');}
  });
});
setInterval(() => {
  if (Date.now() - motion > 350) state.drive = state.turn = 0;
  if (state.controller && Date.now() - heartbeat > 5000) {
    state.controller = false;
    state.drive = state.turn = 0;
    if (!['estop','fault'].includes(state.mode)) state.mode = 'idle';
  }
}, 20);
server.listen(8099, '127.0.0.1', () => console.log('MOCK ONLY dashboard: http://127.0.0.1:8099'));