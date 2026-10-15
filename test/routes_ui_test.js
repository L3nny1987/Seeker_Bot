'use strict';
const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const html=fs.readFileSync('src/web/index.html','utf8');
const script=[...html.matchAll(/<script>([\s\S]*?)<\/script>/g)][1][1];
const nodes=new Map(),storage=new Map(),requests=[];
function node(id){if(!nodes.has(id))nodes.set(id,{value:'',checked:false,disabled:false,hidden:true,style:{},children:[],textContent:'',classList:{toggle(){}},append(child){this.children.push(child);},getBoundingClientRect(){return {width:0,height:0,left:0,top:0};}});return nodes.get(id);}
const context=vm.createContext({$:node,reasonText:{},document:{hidden:true,createElement:()=>node('new-'+Math.random())},window:{addEventListener(){}},setTimeout(){return 1;},clearTimeout(){},
  localStorage:{getItem:k=>storage.get(k)||null,setItem:(k,v)=>storage.set(k,v)},DataView,Uint8Array,ArrayBuffer,Math,Number,String,Date,Promise,AbortController,
  state:{mode:'idle',mapGeneration:9,commissioned:true,poseValid:true,lidarHealthy:true,settings:{robotWidthMm:200}},online:true,token:7,sequence:7,queue:Promise.resolve(),
  zero(){},log(){},confirm(){return true;},request:async path=>{requests.push(path);return {configured:false,points:[],scores:[]};},
  fetch:async(path,options)=>{requests.push({path,options});return {ok:true,json:async()=>({ok:true})};},devicePixelRatio:1});
vm.runInContext(script,context);
const square=[{x:-1000,y:-1000},{x:1000,y:-1000},{x:1000,y:1000},{x:-1000,y:1000}];
const prepared=context.prepareRoute(square);
assert.ok(prepared.length>=8&&prepared.length<=64);
for(let i=0;i<prepared.length;i++){const distance=context.routeDistance(prepared[i],prepared[(i+1)%prepared.length]);assert.ok(distance>=250&&distance<=700.5);}
assert.throws(()=>context.prepareRoute([{x:0,y:0},{x:100,y:0},{x:200,y:0}]));
assert.throws(()=>context.prepareRoute([{x:NaN,y:0},...square]));
assert.throws(()=>context.prepareRoute([{x:6300,y:0},...square]));
const packet=context.routePacket(square,0,true,9,7,8);
assert.equal(Buffer.from(packet).toString('ascii',0,4),'SRTE');
assert.equal(new DataView(packet).getUint32(20,true),0);
assert.deepEqual(Buffer.from(packet),fs.readFileSync('.pio/host-tests/route_packet_fixture.bin'));
assert.throws(()=>context.routePacket(square,-1,true,9,7,8));
assert.throws(()=>context.routePacket(square,1.5,true,9,7,8));
assert.throws(()=>context.routePacket(square,1,true,0,7,8));
const n=128,cells=new Uint8Array(n*n/4).fill(0x55),map={n,res:50,cells,path:[],poseValid:true,x:0,y:0};
const setCell=(gx,gy,cls)=>{const i=gy*n+gx;cells[i>>2]=(cells[i>>2]&~(3<<((i&3)*2)))|(cls<<((i&3)*2));};
assert.equal(context.routeIssue(square,map,100),null);
/* (64,44) lies on the centre line y=-1000 of the bottom edge; cell (64,45) spans
   y -950..-900 (50 mm gap), (64,47) spans -850..-800 (150 mm gap). */
setCell(64,44,0);assert.equal(context.routeIssue(square,map,100).code,'route_unexplored','unexplored centre line must reject route');cells.fill(0x55);
setCell(64,44,3);assert.equal(context.routeIssue(square,map,100).code,'route_obstacle','obstacle between vertices must reject route');cells.fill(0x55);
setCell(64,45,0);assert.equal(context.routeIssue(square,map,100),null,'unknown cells beside the centre line are acceptable');cells.fill(0x55);
setCell(64,45,2);const issue=context.routeIssue(square,map,100);assert.equal(issue.code,'route_obstacle');assert.equal(issue.edge,0);
assert.equal(Math.round(issue.distance),50);assert.deepEqual([issue.x,issue.y],[25,-925]);
assert.equal(context.routeIssue(square,map,40),null,'a narrower corridor accepts the same cell');cells.fill(0x55);
setCell(64,47,3);assert.equal(context.routeIssue(square,map,100),null,'obstacles outside the driving corridor are acceptable');
assert.equal(context.routeIssue(square,map,160).code,'route_obstacle');cells.fill(0x55);
const fixture=JSON.parse(fs.readFileSync('.pio/host-tests/route_corridor_fixture.json','utf8'));
const fixtureMap={n:fixture.n,res:fixture.res,cells:Uint8Array.from(fixture.cells)};
fixture.routes.forEach((r,i)=>{const issue=context.routeIssue(r.points.map(([x,y])=>({x,y})),fixtureMap,fixture.radius);assert.equal(issue?issue.code:'ok',r.code,'route '+i+' differs from firmware');});
assert.equal(context.routeRadius(),100,'default: half of the 200 mm driving corridor');
context.state={...context.state,routeCorridorMm:150};assert.equal(context.routeRadius(),150);
context.state={...context.state,routeCorridorMm:100};
context.testMap=map;context.testPoints=prepared;
vm.runInContext('mapData=testMap;routeFrame=9;routeDraft=testPoints;routeClosed=true;',context);
node('route-laps').value='3';node('route-adaptive').checked=true;context.routeControls();assert.equal(node('route-start').disabled,false);
assert.match(node('route-ready').textContent,/Bereit/);
context.state={...context.state,commissioned:false};context.routeControls();
assert.ok(node('route-start').disabled);assert.match(node('route-ready').textContent,/Inbetriebnahme/);
context.state={...context.state,commissioned:true};
setCell(64,44,3);context.routeControls();
assert.ok(node('route-start').disabled,'a route through a mapped obstacle cannot start');assert.match(node('route-ready').textContent,/rot markierten Abschnitt/);
cells.fill(0x55);context.routeControls();assert.equal(node('route-start').disabled,false);
/* Point editor: insert on an edge, drag, delete, and validate like the firmware. */
const before=vm.runInContext('routeDraft.length',context);
node('route-adjust').onclick();assert.equal(vm.runInContext('routeAdjusting',context),true);
assert.ok(node('route-start').disabled&&node('route-save').disabled);assert.match(node('route-ready').textContent,/Fertig/);
assert.equal(context.routeInsertPoint(0),1);assert.equal(vm.runInContext('routeDraft.length',context),before+1);
const inserted=vm.runInContext('routeDraft[1]',context),first=vm.runInContext('routeDraft[0]',context),second=vm.runInContext('routeDraft[2]',context);
assert.deepEqual([inserted.x,inserted.y],[Math.round((first.x+second.x)/2),Math.round((first.y+second.y)/2)]);
vm.runInContext('routeSelected=1;routeDraft[1]={x:routeDraft[0].x+100,y:routeDraft[0].y}',context);
assert.throws(()=>context.finishRouteEdit(vm.runInContext('routeDraft',context)),/mindestens 25 cm/);
node('route-adjust').onclick();assert.equal(vm.runInContext('routeAdjusting',context),true,'invalid edits keep the editor open');
context.routeControls();assert.equal(node('route-point-delete').disabled,false);node('route-point-delete').onclick();
assert.equal(vm.runInContext('routeDraft.length',context),before);
node('route-adjust').onclick();assert.equal(vm.runInContext('routeAdjusting',context),false);assert.equal(node('route-start').disabled,false);
assert.throws(()=>context.finishRouteEdit([{x:0,y:0},{x:400,y:0},{x:0,y:400}]),/zu klein/);
const split=context.finishRouteEdit([{x:-1500,y:-1000},{x:1500,y:-1000},{x:0,y:1000}]);
assert.ok(split.length>3&&split.every((p,i)=>context.routeDistance(p,split[(i+1)%split.length])<=2000),'edges above 2 m are split');
const triangle=[{x:0,y:0},{x:1000,y:0},{x:0,y:1000}];vm.runInContext('routeDraft=testTriangle',Object.assign(context,{testTriangle:triangle}));
assert.equal(context.routeDeletePoint(1),false,'a loop keeps at least three points');
/* Hit testing on screen coordinates: points first, then the "+" handle at mid-edge. */
vm.runInContext('mapView={s:2,ox:0,oy:0,c0:0,r0:0}',context);
const at=p=>context.routeScreen(p),handle=at({x:500,y:0});
const hit=context.routeHit(at(triangle[1]).x+3,at(triangle[1]).y);assert.equal(hit.type,'point');assert.equal(hit.index,1);assert.equal(hit.d,3);
assert.equal(context.routeHit(handle.x,handle.y).type,'edge');assert.equal(context.routeHit(handle.x,handle.y).index,0);
assert.equal(context.routeHit(at(triangle[0]).x+60,at(triangle[0]).y+60),null);
vm.runInContext('routeDraft=testPoints;mapView=null;',context);
context.routeTelemetry({mapGeneration:9,routeRevision:0,routeActive:true,routeConfigured:true,routePoints:8,routeTarget:2,reason:'route_follow'});
context.routeTelemetry({mapGeneration:9,routeRevision:0,routeActive:false,routeConfigured:true,reason:'route_deviation'});
assert.equal(node('route-state').textContent,'ROUTE ABGEBROCHEN');assert.match(node('route-info').textContent,/neben der Kante/);
node('route-name').value='<unsafe name>';node('route-save').onclick();
assert.ok(storage.get('seeker-routes-v1').includes('<unsafe name>'));
assert.ok(node('route-select').children.some(n=>n.textContent==='<unsafe name>'),'names must be plain text');
node('route-infinite').checked=true;node('route-infinite').onchange();assert.ok(node('route-laps').disabled);
(async()=>{
 await context.startRoute();const upload=requests.find(r=>r.path==='/api/route'&&r.options);
 assert.ok(upload);assert.equal(upload.options.headers['Content-Type'],'application/octet-stream');assert.equal(upload.options.headers['X-Seeker-Control'],'1');
 assert.equal(new DataView(upload.options.body).getUint32(12,true),8);assert.equal(new DataView(upload.options.body).getUint32(20,true),0);
 context.state={...context.state,mode:'auto',routeActive:true};context.routeControls();assert.ok(node('route-start').disabled&&node('route-draw').disabled&&!node('route-stop').disabled);
 context.routeTelemetry({mapGeneration:10,routeRevision:1,routeConfigured:false});
 assert.equal(vm.runInContext('routeDraft.length',context),0);assert.equal(vm.runInContext('routeLibrary.length',context),0);assert.equal(vm.runInContext('mapData',context),null);
 assert.ok(node('route-start').disabled,'map reset must invalidate stale drawings');
 console.log('Route UI: drawing geometry, exact corridor check, point editor, start blockers, abort reasons, C-compatible packets, finite/infinite input, safe names, upload ordering and map reset passed.');
})().catch(error=>{console.error(error);process.exitCode=1;});
