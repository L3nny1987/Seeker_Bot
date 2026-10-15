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
assert.ok(context.routeKnownFree(square,map,260));
const index=44*n+64;cells[index>>2]&=~(3<<((index&3)*2));
assert.ok(!context.routeKnownFree(square,map,260),'unknown floor must reject route');cells.fill(0x55);
cells[index>>2]|=3<<((index&3)*2);assert.ok(!context.routeKnownFree(square,map,260),'obstacle between vertices must reject route');cells.fill(0x55);
context.testMap=map;context.testPoints=prepared;
vm.runInContext('mapData=testMap;routeFrame=9;routeDraft=testPoints;routeClosed=true;',context);
node('route-laps').value='3';node('route-adaptive').checked=true;context.routeControls();assert.equal(node('route-start').disabled,false);
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
 console.log('Route UI: drawing geometry, known-free corridor, C-compatible packets, finite/infinite input, safe names, upload ordering and map reset passed.');
})().catch(error=>{console.error(error);process.exitCode=1;});
