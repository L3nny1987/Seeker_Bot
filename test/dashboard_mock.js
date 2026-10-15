'use strict';
// Local UI test fixture only. Never connects to hardware or reads credentials.
const http = require('node:http');
const fs = require('node:fs');
const state = { mode: 'idle', reason: 'mock_only', controller: false,
  commissioned: false, lidarHealthy: true, drive: 0, turn: 0, coverage: 360,
  scanAgeMs: 25, frontMm: 2000, rearMm: 2500, packets: 1234, crcErrors: 0,
  heap: 140000, uptime: 60, version: '0.4.1 MOCK ONLY', manualLimit: 100, lidarFrontDeg: 180 };
let scanSequence = 0;
state.settings={manualStopMm:200,manualRotateMm:200,autoStopMm:500,autoRotateMm:550,
 autoSlowMm:1200,autoDrivePercent:90,autoTurnPercent:100,autoSlowPercent:85,rampMs:1000,
 reverseMs:800,turnMs:1600,resumeMarginMm:50,robotWidthMm:250,autoLimitS:300,
 turnRampMs:400,steerMinPercent:20,wallDistanceMm:350,driveMinPercent:85,turnMinPercent:80,
 driveKickMs:1000,turnKickMs:300};
Object.assign(state,{strategy:'explore',navDetail:'mock',goalSet:false,goalX:0,goalY:0,
 poseValid:true,slamLost:false,poseX:600,poseY:-300,poseDeg:30,slamQuality:0.82,
 imuPresent:true,imuReady:true,imuCalibrating:false,imuRateDps:42,imuTiltDeg:2,imuHz:200,imuErrors:0,
 slamImu:true,headingSigmaDeg:0.8,gyroBiasDps:0.01,wifiRssi:-44,wifiDisconnects:1,wifiLastReason:200,
 wifiConnectedS:312,wifiLastOutageMs:2400,httpClients:2,
 slamScans:500,slamIntegrated:120,slamRejected:3,slamUpdateMs:18,mapAreaM2:9.5,pathPoints:3});
/* Synthetic 3 x 4 m room with the same layout as /api/map in web_server.c. */
function map() {
  const n = 192, res = 50, path = [[0, 0], [30, -10], [60, -30]];
  const b = Buffer.alloc(40 + n * n / 4 + path.length * 4);
  b.write('SMAP', 0); b[4] = 1; b[5] = 1;
  b.writeUInt16LE(n, 6); b.writeUInt16LE(res, 8); b.writeUInt16LE(path.length, 10);
  b.writeFloatLE(state.poseX, 12); b.writeFloatLE(state.poseY, 16);
  b.writeFloatLE(state.poseDeg * Math.PI / 180, 20); b[32] = 82;
  for (let gy = 0; gy < n; gy++) for (let gx = 0; gx < n; gx++) {
    const x = (gx - n / 2) * res, y = (gy - n / 2) * res;
    if (x < -1000 || x > 3000 || y < -1500 || y > 1500) continue;
    const wall = x < -950 || x > 2950 || y < -1450 || y > 1450 || (x > 1200 && x < 1300 && y > 0);
    const i = gy * n + gx;
    b[40 + (i >> 2)] |= (wall ? 3 : 1) << ((i & 3) * 2);
  }
  path.forEach(([x, y], i) => { b.writeInt16LE(x, 40 + n * n / 4 + i * 4); b.writeInt16LE(y, 42 + n * n / 4 + i * 4); });
  return b;
}
function scan() {
  return {fresh: true, rpm: 600, ageMs: 25, sequence: ++scanSequence,
    distance: Array.from({length: 360}, (_, a) => 2000 + Math.round(400 * Math.sin(a / 20)))};
}
Object.assign(state,{mapGeneration:9001,routeRevision:0,routeActive:false,routeConfigured:false,
 routeCompleted:0,routeLearnedLaps:0,routeLaps:0,routeTarget:0,routePoints:0,routeConfidence:0,routePower:85,
 slamWeak:false,slamDegenerate:false,poseAgeMs:20,bodyFrontMm:100,bodyRearMm:200,bodyHalfWidthMm:70});
let mockRoute={configured:false,active:false,generation:9001,points:[],scores:[]};
let routeTick=0;
let heartbeat = 0, motion = 0;
const commands = [];
const server = http.createServer((req, res) => {
  res.setHeader('Cache-Control', 'no-store');
  if (req.url === '/') {
    res.setHeader('Content-Type', 'text/html; charset=utf-8');
    return res.end(fs.readFileSync('src/web/index.html'));
  }
  if (req.url.startsWith('/api/map')) { res.setHeader('Content-Type', 'application/octet-stream'); return res.end(map()); }
  res.setHeader('Content-Type', 'application/json');
  if (req.url === '/api/status') return res.end(JSON.stringify(state));
  if (req.url === '/api/telemetry') return res.end(JSON.stringify({...state, scan: scan()}));
  if (req.url === '/api/scan') return res.end(JSON.stringify(scan()));
  if (req.url === '/test/commands') return res.end(JSON.stringify(commands));
  if (req.url === '/api/route' && req.method === 'GET') return res.end(JSON.stringify(mockRoute));
  if (req.url === '/api/route' && req.method === 'POST') {
    const chunks=[];let size=0;
    req.on('data',data=>{size+=data.length;if(size>288)req.destroy();else chunks.push(data);});
    req.on('end',()=>{
      const b=Buffer.concat(chunks),count=b.length>=32?b.readUInt16LE(6):0;
      if(b.length!==32+count*4||count<3||count>64||b.toString('ascii',0,4)!=='SRTE'||b[4]!==1||b[5]>1){res.statusCode=400;return res.end(JSON.stringify({ok:false,error:'route_invalid_packet'}));}
      if(!state.controller||b.readUInt32LE(8)!==123||state.mode!=='idle'){res.statusCode=409;return res.end(JSON.stringify({ok:false,error:'route_busy'}));}
      if(b.readUInt32LE(16)!==state.mapGeneration){res.statusCode=409;return res.end(JSON.stringify({ok:false,error:'route_wrong_map'}));}
      const points=Array.from({length:count},(_,i)=>[b.readInt16LE(32+i*4),b.readInt16LE(34+i*4)]);
      mockRoute={configured:true,active:true,generation:state.mapGeneration,points,scores:Array(count).fill(0),laps:b.readUInt32LE(20),completed:0,learnedLaps:0,target:0,approaching:true,adaptive:!!b[5]};
      Object.assign(state,{mode:'auto',strategy:'route',reason:'route_approach',routeRevision:state.routeRevision+1,routeActive:true,routeConfigured:true,routeApproaching:true,routeLaps:mockRoute.laps,routeCompleted:0,routeLearnedLaps:0,routePoints:count,routeTarget:0,routeConfidence:0,routeAdaptive:mockRoute.adaptive});
      commands.push({command:'route',points,laps:mockRoute.laps,adaptive:mockRoute.adaptive});routeTick=Date.now();heartbeat=Date.now();res.end(JSON.stringify({ok:true}));
    });return;
  }
  if (req.url !== '/api/control'  || req.method !== 'POST') {res.statusCode = 404; return res.end('{}');}
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
      if (b.command === 'estop') {state.mode = 'estop'; state.drive = state.turn = 0;state.routeActive=false;mockRoute.active=false;}
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
        if (b.command === 'map_reset') {state.mapGeneration++;state.routeConfigured=false;mockRoute={configured:false,active:false,generation:state.mapGeneration,points:[],scores:[]};}
        if(['stop','release','reset','mode'].includes(b.command)){state.routeActive=false;mockRoute.active=false;}

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
  if(state.routeActive&&Date.now()-routeTick>700){
    routeTick=Date.now();const point=mockRoute.points[mockRoute.target];state.poseX=point[0];state.poseY=point[1];
    if(mockRoute.approaching){mockRoute.approaching=false;state.routeApproaching=false;}
    else {mockRoute.scores[mockRoute.target]=Math.min(100,mockRoute.scores[mockRoute.target]+20);
      if(mockRoute.target===0){mockRoute.completed++;mockRoute.learnedLaps++;state.routeCompleted=mockRoute.completed;state.routeLearnedLaps=mockRoute.learnedLaps;
        if(mockRoute.laps&&mockRoute.completed>=mockRoute.laps){state.mode='idle';state.routeActive=false;mockRoute.active=false;state.reason='route_complete';}}
    }
    mockRoute.target=(mockRoute.target+1)%mockRoute.points.length;state.routeTarget=mockRoute.target;
    state.routeConfidence=Math.round(mockRoute.scores.reduce((a,b)=>a+b,0)/mockRoute.scores.length);
    state.routePower=mockRoute.adaptive?85+Math.floor(state.routeConfidence/20):90;
  }
  if (Date.now() - motion > 350) state.drive = state.turn = 0;
  if (state.controller && Date.now() - heartbeat > 5000) {
    state.controller = false;
    state.drive = state.turn = 0;
    if (!['estop','fault'].includes(state.mode)) state.mode = 'idle';state.routeActive=false;mockRoute.active=false;
  }
}, 20);
server.listen(8099, '127.0.0.1', () => console.log('MOCK ONLY dashboard: http://127.0.0.1:8099'));