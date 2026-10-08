'use strict';
const fs = require('node:fs');
const html = fs.readFileSync('src/web/index.html', 'utf8');
const scripts = [...html.matchAll(/<script>([\s\S]*?)<\/script>/g)].map(m => m[1]);
if (!scripts.length) throw new Error('Dashboard script missing');
/* Classic scripts share one global scope: check them together. */
new Function(scripts.join('\n;\n'));
console.log('Dashboard JavaScript syntax valid (' + scripts.length + ' scripts)');