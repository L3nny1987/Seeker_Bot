'use strict';
const fs=require('node:fs');
const assert=require('node:assert/strict');
const pins=fs.readFileSync('components/seeker_config/include/seeker_pins.h','utf8');
const manager=fs.readFileSync('src/robot_manager.c','utf8');
const expected={SEEKER_MANUAL_STOP_MM:60,SEEKER_MANUAL_ROTATE_MM:30,
 SEEKER_STOP_DISTANCE_MM:100,SEEKER_ROTATE_CLEARANCE_MM:40,SEEKER_SLOW_DISTANCE_MM:300,
 SEEKER_AUTO_DRIVE_PERCENT:90,SEEKER_AUTO_TURN_PERCENT:100,SEEKER_AUTO_SLOW_PERCENT:85,
 SEEKER_RAMP_MS:1000,SEEKER_REVERSE_MS:500,SEEKER_TURN_MS:1600,SEEKER_RESUME_MARGIN_MM:50,
 SEEKER_ROBOT_WIDTH_MM:200,SEEKER_AUTO_LIMIT_S:300,SEEKER_TURN_RAMP_MS:400,
 SEEKER_STEER_MIN_PERCENT:80,SEEKER_WALL_DISTANCE_MM:250,
 SEEKER_DRIVE_MIN_PERCENT:85,SEEKER_TURN_MIN_PERCENT:80,SEEKER_DRIVE_KICK_MS:1000,SEEKER_TURN_KICK_MS:300};
const html=fs.readFileSync('src/web/index.html','utf8');
const dashboardDefaults=JSON.parse(html.match(/const firmwareDefaults=(\{[^}]*\});/)[1]);
const dashboardName={SEEKER_MANUAL_STOP_MM:'manualStopMm',SEEKER_MANUAL_ROTATE_MM:'manualRotateMm',
 SEEKER_STOP_DISTANCE_MM:'autoStopMm',SEEKER_ROTATE_CLEARANCE_MM:'autoRotateMm',SEEKER_SLOW_DISTANCE_MM:'autoSlowMm',
 SEEKER_AUTO_DRIVE_PERCENT:'autoDrivePercent',SEEKER_AUTO_TURN_PERCENT:'autoTurnPercent',SEEKER_AUTO_SLOW_PERCENT:'autoSlowPercent',
 SEEKER_RAMP_MS:'rampMs',SEEKER_REVERSE_MS:'reverseMs',SEEKER_TURN_MS:'turnMs',SEEKER_RESUME_MARGIN_MM:'resumeMarginMm',
 SEEKER_ROBOT_WIDTH_MM:'robotWidthMm',SEEKER_AUTO_LIMIT_S:'autoLimitS',SEEKER_TURN_RAMP_MS:'turnRampMs',
 SEEKER_STEER_MIN_PERCENT:'steerMinPercent',SEEKER_WALL_DISTANCE_MM:'wallDistanceMm',
 SEEKER_DRIVE_MIN_PERCENT:'driveMinPercent',SEEKER_TURN_MIN_PERCENT:'turnMinPercent',
 SEEKER_DRIVE_KICK_MS:'driveKickMs',SEEKER_TURN_KICK_MS:'turnKickMs'};
for(const [name,value] of Object.entries(expected)){
 const match=pins.match(new RegExp('^#define\\s+'+name+'\\s+(\\d+)','m'));
 assert.ok(match,name+' must exist');
 assert.equal(Number(match[1]),value,name);
 assert.ok(manager.includes('= '+name),name+' must initialize manager settings');
 assert.equal(dashboardDefaults[dashboardName[name]],value,name+' dashboard default');
}
assert.match(manager,/\.mode\s*=\s*ROBOT_IDLE/,'Boot must not start autonomous movement');
console.log('Motion defaults verified; boot remains IDLE.');