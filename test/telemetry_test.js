'use strict';
// Deterministic browser logic test. No network or physical motor commands.
const fs = require('node:fs');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const html = fs.readFileSync('src/web/index.html', 'utf8');
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1]
  .replace('window.addEventListener(\'resize\',draw);draw();poll();pump();', '');
const nodes = new Map(), timers = [], requests = [];
let failure = false;
let failOnce=false,reject=false;
let mapStatus=200,holdTelemetry=false,releaseTelemetry;
const scan = {fresh: true, rpm: 600, sequence: 41, distance: Array(360).fill(2000)};
const status = {mode: 'manual', lidarHealthy: true, scan, uptime: 12,
  scanAgeMs: 20, coverage: 360, drive: 25, turn: 0, heap: 190000,
  packets: 100, crcErrors: 0, manualLimit: 100, reason: 'manual'};
status.settings={manualStopMm:200,manualRotateMm:200,autoStopMm:500,autoRotateMm:550,
  autoSlowMm:1200,autoDrivePercent:30,autoTurnPercent:30,autoSlowPercent:20,rampMs:1000,
  reverseMs:800,turnMs:1600,resumeMarginMm:50,robotWidthMm:250,autoLimitS:300,
  turnRampMs:400,steerMinPercent:20,wallDistanceMm:350,driveMinPercent:85,turnMinPercent:80,
  driveKickMs:1000,turnKickMs:300};
function node(id) {
  if (!nodes.has(id)) nodes.set(id, {value: '30', style: {}, children: [],
    classList: {add() {}, remove() {}, toggle() {}}, addEventListener() {},
    prepend() {}, before() {}, append() {}, reportValidity: () => true, textContent: '',
    getBoundingClientRect: () => ({width:0,height:0})});
  return nodes.get(id);
}
const context = vm.createContext({
  document: {hidden: false, getElementById: node, querySelector: node, querySelectorAll: () => [],
    createElement: () => node('created'), addEventListener() {}},
  window: {addEventListener() {}}, Date, Math, Set, Promise, confirm: () => true,
  AbortController, performance: {now: () => 100},
  setTimeout: (fn, ms) => {timers.push({fn, ms}); return timers.length;},
  clearTimeout() {},
  fetch: async (path, options) => {
    requests.push({path, options});
    if(failOnce){failOnce=false;throw new Error('temporary transport failure');}
    if(reject)return {ok:false,status:409};
    if (failure) throw new Error('test timeout');
    if(holdTelemetry&&path.startsWith('/api/telemetry'))await new Promise(resolve=>{releaseTelemetry=resolve;});
    if(path.startsWith('/api/map')){
      const b=new ArrayBuffer(56),v=new DataView(b);
      [...'SMAP'].forEach((c,i)=>v.setUint8(i,c.charCodeAt(0)));v.setUint8(4,1);v.setUint16(6,8,true);v.setUint16(8,50,true);
      return {ok:mapStatus===200,status:mapStatus,arrayBuffer:async()=>b};
    }
    return {ok: true, json: async () => path === '/api/telemetry'||path==='/api/status' ? status : {ok: true}};
  },
});
vm.runInContext(script, context);
vm.runInContext('draw=()=>{}; token=123; online=true; state={mode:"manual"}; axes={drive:1,turn:0};', context);
(async () => {
  await vm.runInContext('poll()', context);
  assert.equal(requests.length, 1);
  assert.equal(requests[0].path, '/api/telemetry');
  assert.equal(vm.runInContext('scanData.sequence', context), 41);
  assert.equal(vm.runInContext('axes.drive', context), 1, 'Telemetry must not cancel valid movement');
  assert.equal(node('setting-rampMs').value,1000);
  assert.equal(node('zone-label').textContent,'200 mm Stoppzone ab Körper');
  assert.equal(node('save-settings').disabled,true,'Settings prohibited during manual movement');
  assert.ok(timers.some(t => t.ms === 200), 'Target telemetry period is 200 ms');
  await vm.runInContext('pump()', context);
  const move = JSON.parse(requests.at(-1).options.body);
  assert.equal(move.command, 'move');
  assert.equal(move.drive, 30);
  assert.ok(timers.some(t => t.ms === 100), 'Control remains on its separate 100 ms cadence');
  scan.sequence++;
  await vm.runInContext('poll()', context);
  assert.equal(vm.runInContext('scanData.sequence', context), 42, 'Radar updates while moving');
  status.mode='idle';
  await vm.runInContext('poll()',context);
  assert.equal(node('save-settings').disabled,false);
  await vm.runInContext('saveSettings.onclick()',context);
  const settings=JSON.parse(requests.at(-1).options.body);
  assert.equal(settings.command,'settings');
  assert.equal(settings.manualStopMm,200);
  assert.equal(settings.rampMs,1000);
  assert.equal(settings.robotWidthMm,250);
  assert.equal(settings.autoLimitS,300);
  assert.equal(settings.turnRampMs,400);
  assert.equal(settings.wallDistanceMm,350);
  assert.equal(settings.driveMinPercent,85);
  assert.equal(settings.turnKickMs,300);
  assert.ok(requests.at(-1).options.body.length<=768);
  failure = true;
  vm.runInContext('state={mode:"auto"};',context);
  const pollBefore=requests.length;
  await vm.runInContext('poll()', context);
  assert.equal(requests.length,pollBefore+1,'Telemetry failure must not send STOP (would abort auto)');
  assert.equal(vm.runInContext('axes.drive', context), 0, 'Failure stops local motion');
  assert.equal(vm.runInContext('online', context), false);
  assert.equal(vm.runInContext('scanData', context), null, 'Do not display an old scan as live');
  await vm.runInContext('queue', context);
  failure=false;
  vm.runInContext('token=123;online=true;state={mode:"auto"};',context);
  failOnce=true;
  const before=requests.length;
  await vm.runInContext('pump()',context);
  assert.equal(vm.runInContext('token',context),123,'Single transport miss must not abandon ownership');
  const retryCommands=requests.slice(before).map(r=>JSON.parse(r.options.body));
  assert.deepEqual(retryCommands.map(b=>b.command),['heartbeat'],'Never retry immediately');
  await vm.runInContext('pump()',context);
  const next=JSON.parse(requests.at(-1).options.body);
  assert.equal(next.command,'heartbeat');
  assert.ok(next.sequence>retryCommands[0].sequence,'Fresh sequence after a lost packet');
  assert.ok(timers.some(t=>t.ms===250),'Idle heartbeat cadence 250 ms');
  reject=true;
  const rejectedBefore=requests.length;
  await vm.runInContext('pump()',context);
  assert.equal(requests.length,rejectedBefore+1,'Explicit rejection must not be retried');
  assert.equal(vm.runInContext('token',context),0);
  reject=false;
  vm.runInContext('token=123;',context);status.controller=false;status.reason='controller_timeout';
  await vm.runInContext('poll()',context);
  assert.equal(vm.runInContext('token',context),0,'Confirmed server lease loss clears local ownership');

  node('tab-fahren').hidden=true;
  await vm.runInContext('poll()',context);
  assert.equal(requests.at(-1).path,'/api/status','Hidden radar uses lightweight status');
  assert.ok(timers.some(t=>t.ms===500),'Hidden radar polls at 2 Hz');
  node('tab-fahren').hidden=false;
  holdTelemetry=true;
  const pending=vm.runInContext('poll()',context), flightCount=requests.length;
  await vm.runInContext('poll()',context);
  assert.equal(requests.length,flightCount,'Slow telemetry must not overlap');
  holdTelemetry=false;releaseTelemetry();await pending;
  failure=true;
  await vm.runInContext('poll()',context);
  assert.ok(timers.some(t=>t.ms===1000),'Transport failure backs off telemetry');
  failure=false;

  const mapScript=[...html.matchAll(/<script>([\s\S]*?)<\/script>/g)][1][1]
    .replace("window.addEventListener('resize',drawMap);drawMap();mapLoop();",'');
  vm.runInContext(mapScript,context);
  vm.runInContext('drawMap=()=>{};buildImage=()=>({});online=true;',context);
  node('tab-karte').hidden=true;
  let beforeMap=requests.length;
  await vm.runInContext('mapLoop()',context);
  assert.equal(requests.length,beforeMap,'Hidden map must not fetch any map data');
  node('tab-karte').hidden=false;
  await vm.runInContext('mapLoop()',context);
  assert.equal(requests.at(-1).path,'/api/map?rle=1','Visible map requests bounded compression');
  mapStatus=503;
  await vm.runInContext('mapLoop()',context);
  assert.equal(vm.runInContext('mapRetryMs',context),6000,'Low-memory response backs off large transfers');
  assert.equal(vm.runInContext('mapBusy',context),false);
  mapStatus=200;
  await vm.runInContext('mapLoop()',context);
  assert.equal(vm.runInContext('mapRetryMs',context),3000);
  vm.runInContext('online=false;',context);beforeMap=requests.length;
  await vm.runInContext('mapLoop()',context);
  assert.equal(requests.length,beforeMap,'Offline map must not create additional load');
  console.log('Telemetry tests passed: visible radar 5 Hz, hidden status 2 Hz, no hidden map traffic, backoff, single-flight, unchanged control cadence.');
})().catch(error => {console.error(error); process.exitCode = 1;});
