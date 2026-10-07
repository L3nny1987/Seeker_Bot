'use strict';
// Compile the actual axis() implementation against mocked GPIO/LEDC functions.
// Never connects to hardware. Requires a host gcc.
const fs = require('node:fs');
const path = require('node:path');
const os = require('node:os');
const {spawnSync} = require('node:child_process');
const assert = require('node:assert/strict');
const source = fs.readFileSync('components/track_drive/track_drive.c', 'utf8');
const config = fs.readFileSync('components/seeker_config/include/seeker_pins.h', 'utf8');
const axis = source.slice(source.indexOf('static esp_err_t axis('), source.indexOf('static esp_err_t set_locked('));
const inverted = name => Number(config.match(new RegExp('#define\\s+' + name + '\\s+(\\d+)'))[1]);
const program = `
#include <stdbool.h>
#include <assert.h>
#include <stdio.h>
typedef int esp_err_t;
typedef int ledc_channel_t;
typedef int gpio_num_t;
#define ESP_OK 0
#define SEEKER_PWM_MODE 0
static int pins[40], duty[2], updates, delay_us;
static int gpio_set_level(int pin, int level) { pins[pin]=level; return 0; }
static int ledc_set_duty(int mode, int channel, int value) { (void)mode; duty[channel]=value; return 0; }
static int ledc_update_duty(int mode, int channel) { (void)mode; (void)channel; updates++; return 0; }
static void esp_rom_delay_us(int value) { delay_us+=value; }
${axis}
int main(void) {
  assert(axis(0,26,27,100,0,${inverted('SEEKER_DRIVE_INVERTED')})==0);
  assert(pins[26]==1 && pins[27]==0 && duty[0]==1024);
  assert(axis(0,26,27,-100,100,${inverted('SEEKER_DRIVE_INVERTED')})==0);
  assert(pins[26]==0 && pins[27]==1 && duty[0]==1024 && delay_us==1000);
  assert(axis(1,32,33,-100,0,${inverted('SEEKER_TURN_INVERTED')})==0);
  assert(pins[32]==1 && pins[33]==0 && duty[1]==1024); /* LEFT */
  int count=updates;
  assert(axis(1,32,33,-100,-100,${inverted('SEEKER_TURN_INVERTED')})==0);
  assert(updates==count); /* Constant duty must not be pulsed off. */
  assert(axis(1,32,33,50,-100,${inverted('SEEKER_TURN_INVERTED')})==0);
  assert(pins[32]==0 && pins[33]==1 && duty[1]==512); /* RIGHT */
  assert(axis(1,32,33,0,50,${inverted('SEEKER_TURN_INVERTED')})==0);
  assert(pins[32]==0 && pins[33]==0 && duty[1]==0);
  assert(axis(0,26,27,0,-100,${inverted('SEEKER_DRIVE_INVERTED')})==0);
  assert(pins[26]==0 && pins[27]==0 && duty[0]==0);
  puts("Motor tests passed: forward/reverse, left/right, full HIGH, PWM, unchanged duty, stop.");
}
`;
const exe = path.join(os.tmpdir(), `seeker-motor-test-${process.pid}.exe`);
try {
  const build = spawnSync('gcc', ['-x', 'c', '-std=c11', '-Wall', '-Wextra', '-Werror', '-', '-o', exe], {input: program, encoding: 'utf8'});
  assert.equal(build.status, 0, build.error?.message || build.stderr);
  const result = spawnSync(exe, [], {encoding: 'utf8'});
  assert.equal(result.status, 0, result.stderr);
  process.stdout.write(result.stdout);
} finally {if (fs.existsSync(exe)) fs.unlinkSync(exe);}