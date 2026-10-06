'use strict';
const fs = require('node:fs');
const html = fs.readFileSync('src/web/index.html', 'utf8');
const script = html.match(/<script>([\s\S]*?)<\/script>/);
if (!script) throw new Error('Dashboard script missing');
new Function(script[1]);
console.log('Dashboard JavaScript syntax valid');