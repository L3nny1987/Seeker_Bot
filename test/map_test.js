'use strict';
// Map decoder test: binary /api/map layout and map <-> world coordinates.
const fs = require('node:fs');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const html = fs.readFileSync('src/web/index.html', 'utf8');
const script = [...html.matchAll(/<script>([\s\S]*?)<\/script>/g)][1][1];
const node = () => ({style: {}, textContent: '', getBoundingClientRect: () => ({width: 0, height: 0})});
const context = vm.createContext({
  $: node, reasonText: {}, document: {hidden: true}, window: {addEventListener() {}},
  setTimeout() {}, clearTimeout() {}, DataView, Uint8Array, String, Math, Number, devicePixelRatio: 1,
});
vm.runInContext(script, context);

const n = 8, bytes = n * n / 4, count = 2;
const buffer = new ArrayBuffer(40 + bytes + count * 4);
const v = new DataView(buffer);
'SMAP'.split('').forEach((c, i) => v.setUint8(i, c.charCodeAt(0)));
v.setUint8(4, 1); v.setUint8(5, 1 | 4);
v.setUint16(6, n, true); v.setUint16(8, 50, true); v.setUint16(10, count, true);
v.setFloat32(12, 100, true); v.setFloat32(16, -50, true); v.setFloat32(20, 0.5, true);
v.setFloat32(24, 1500, true); v.setFloat32(28, 250, true);
v.setUint8(32, 87); v.setUint32(36, 42, true);
const cell = 2 * n + 5;                     /* gy = 2, gx = 5 */
v.setUint8(40 + (cell >> 2), 3 << ((cell & 3) * 2));
v.setInt16(40 + bytes, 10, true); v.setInt16(42 + bytes, -5, true);
v.setInt16(44 + bytes, 20, true); v.setInt16(46 + bytes, 0, true);

const m = context.parseMap(buffer);
assert.equal(m.n, n); assert.equal(m.res, 50);
assert.ok(m.poseValid && !m.lost && m.goalSet);
assert.equal(m.x, 100); assert.equal(m.y, -50); assert.ok(Math.abs(m.theta - 0.5) < 1e-6);
assert.equal(m.goalX, 1500); assert.equal(m.quality, 87); assert.equal(m.integrated, 42);
assert.equal(JSON.stringify(m.path), '[[100,-50],[200,0]]');
assert.equal((m.cells[cell >> 2] >> ((cell & 3) * 2)) & 3, 3);
assert.throws(() => context.parseMap(new ArrayBuffer(10)));

/* v2 RLE, including path data at an offset not divisible by four. */
const compressed=new ArrayBuffer(40+2+count*4), cv=new DataView(compressed);
new Uint8Array(compressed,0,40).set(new Uint8Array(buffer,0,40));
cv.setUint8(4,2);cv.setUint16(34,2,true);cv.setUint8(40,bytes);cv.setUint8(41,0xaa);
new Uint8Array(compressed,42).set(new Uint8Array(buffer,40+bytes,count*4));
const cm=context.parseMap(compressed);
assert.deepEqual([...cm.cells],Array(bytes).fill(0xaa));
assert.equal(JSON.stringify(cm.path),'[[100,-50],[200,0]]');
for(const badRun of [0,bytes-1,bytes+1]){
  cv.setUint8(40,badRun);assert.throws(()=>context.parseMap(compressed));
}
cv.setUint8(40,bytes);cv.setUint16(34,1,true);
assert.throws(()=>context.parseMap(compressed));
cv.setUint16(34,2,true);cv.setUint8(4,3);
assert.throws(()=>context.parseMap(compressed));
if(fs.existsSync('.pio/host-tests/map_rle_fixture.bin')){
  const encoded=fs.readFileSync('.pio/host-tests/map_rle_fixture.bin');
  const decoded=context.parseMap(encoded.buffer.slice(encoded.byteOffset,encoded.byteOffset+encoded.byteLength));
  assert.deepEqual(Buffer.from(decoded.cells),fs.readFileSync('.pio/host-tests/map_rle_expected.bin'));
  assert.equal(JSON.stringify(decoded.path),'[[100,-50],[200,0]]');
}

/* x forward is drawn up (smaller row), y left is drawn left (smaller col). */
const origin = context.toCell(m, 0, 0), ahead = context.toCell(m, 1000, 0), left = context.toCell(m, 0, 1000);
assert.ok(ahead.row < origin.row && ahead.col === origin.col);
assert.ok(left.col < origin.col && left.row === origin.row);
/* Click inverse used by the goal picker. */
const c = context.toCell(m, 300, 150);
assert.equal((n - 1 - c.row - n / 2) * m.res, 300);
assert.equal((n - 1 - c.col - n / 2) * m.res, 150);
console.log('Map decoder tests passed: v1/v2, C encoder round-trip, malformed RLE, paths, orientation, goal picking.');
