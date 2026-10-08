#pragma once
#include <stdbool.h>

typedef struct {
    int manual_stop_mm, manual_rotate_mm;
    int auto_stop_mm, auto_rotate_mm, auto_slow_mm;
    int auto_drive_percent, auto_turn_percent, auto_slow_percent;
    int ramp_ms;
    int reverse_ms, turn_ms, resume_margin_mm;
    int robot_width_mm, auto_limit_s;
    int turn_ramp_ms, steer_min_percent, wall_distance_mm;
    /* Output stage: below these PWM values the motors stall; every start
       from standstill gets a full-power kick of *_kick_ms. */
    int drive_min_percent, turn_min_percent, drive_kick_ms, turn_kick_ms;
} motion_settings_t;

/* Bump when the layout changes: stored NVS settings are then ignored. */
#define MOTION_SETTINGS_VERSION 4

static inline bool motion_settings_valid(motion_settings_t s)
{
    /* Distances are clearances from the robot body (bumper), not the LiDAR. */
    return s.manual_stop_mm >= 20 && s.manual_stop_mm <= 2000 &&
           s.manual_rotate_mm >= 10 && s.manual_rotate_mm <= 2000 &&
           s.auto_stop_mm >= 30 && s.auto_stop_mm <= 2000 &&
           s.auto_rotate_mm >= 10 && s.auto_rotate_mm <= 2000 &&
           s.auto_slow_mm > s.auto_stop_mm + s.resume_margin_mm && s.auto_slow_mm <= 4000 &&
           s.auto_drive_percent >= 1 && s.auto_drive_percent <= 100 &&
           s.auto_turn_percent >= 1 && s.auto_turn_percent <= 100 &&
           s.auto_slow_percent >= 1 && s.auto_slow_percent <= s.auto_drive_percent &&
           s.ramp_ms >= 0 && s.ramp_ms <= 5000 &&
           s.reverse_ms >= 100 && s.reverse_ms <= 2000 &&
           s.turn_ms >= 200 && s.turn_ms <= 4000 &&
           s.resume_margin_mm >= 0 && s.resume_margin_mm <= 300 &&
           s.robot_width_mm >= 100 && s.robot_width_mm <= 1000 &&
           s.auto_limit_s >= 10 && s.auto_limit_s <= 3600 &&
           s.turn_ramp_ms >= 0 && s.turn_ramp_ms <= 5000 &&
           s.steer_min_percent >= 0 && s.steer_min_percent <= s.auto_turn_percent &&
           s.wall_distance_mm >= s.robot_width_mm / 2 + 50 && s.wall_distance_mm <= 1500 &&
           s.drive_min_percent >= 0 && s.drive_min_percent <= 100 &&
           s.turn_min_percent >= 0 && s.turn_min_percent <= 100 &&
           s.drive_kick_ms >= 0 && s.drive_kick_ms <= 3000 &&
           s.turn_kick_ms >= 0 && s.turn_kick_ms <= 3000;
}

/* Immediate safety stops and zero crossing take precedence over the ramp. */
static inline int motion_ramp(int current, int target, int ramp_ms, int period_ms, int *remainder)
{
    if (!target || (current > 0 && target < 0) || (current < 0 && target > 0)) {
        *remainder = 0; return 0;
    }
    if (current == target) { *remainder = 0; return current; }
    int step = 100;
    if (ramp_ms) {
        *remainder += 100 * period_ms;
        step = *remainder / ramp_ms;
        *remainder %= ramp_ms;
    }
    if (current < target) return current + step > target ? target : current + step;
    if (current > target) return current - step < target ? target : current - step;
    return current;
}

typedef struct { int value, remainder, kick_ms; } motion_axis_t;

/* One motor axis per control period. Non-zero commands never go below
   min_percent (the motor would stall). A start from standstill or after a
   reversal runs at 100 % for kick_ms (static friction), then ramps to the
   target. Zero and sign changes stop at once. */
static inline int motion_axis_step(motion_axis_t *a, int target, int min_percent, int kick_ms,
                                   int ramp_ms, int period_ms)
{
    if (target) {
        int m = target < 0 ? -target : target;
        if (m < min_percent) m = min_percent;
        if (m > 100) m = 100;
        target = target < 0 ? -m : m;
    }
    if (!target || (a->value > 0 && target < 0) || (a->value < 0 && target > 0)) {
        a->value = a->remainder = a->kick_ms = 0;
        return 0;
    }
    int full = target > 0 ? 100 : -100;
    if (!a->value) {
        a->remainder = 0;
        a->kick_ms = kick_ms;
        a->value = kick_ms > 0 ? full : target;
        return a->value;
    }
    if (a->kick_ms > 0) {
        a->kick_ms -= period_ms;
        if (a->kick_ms > 0) return a->value = full;
        a->kick_ms = 0;
    }
    return a->value = motion_ramp(a->value, target, ramp_ms, period_ms, &a->remainder);
}