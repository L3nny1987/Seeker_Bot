#pragma once
#include <stdbool.h>

typedef struct {
    int manual_stop_mm, manual_rotate_mm;
    int auto_stop_mm, auto_rotate_mm, auto_slow_mm;
    int auto_drive_percent, auto_turn_percent, auto_slow_percent;
    int ramp_ms;
    int reverse_ms, turn_ms, resume_margin_mm;
    int robot_width_mm, auto_limit_s;
} motion_settings_t;

/* Bump when the layout changes: stored NVS settings are then ignored. */
#define MOTION_SETTINGS_VERSION 2

static inline bool motion_settings_valid(motion_settings_t s)
{
    return s.manual_stop_mm >= 100 && s.manual_stop_mm <= 2000 &&
           s.manual_rotate_mm >= 100 && s.manual_rotate_mm <= 2000 &&
           s.auto_stop_mm >= 100 && s.auto_stop_mm <= 2000 &&
           s.auto_rotate_mm >= 100 && s.auto_rotate_mm <= 2000 &&
           s.auto_slow_mm > s.auto_stop_mm + s.resume_margin_mm && s.auto_slow_mm <= 4000 &&
           s.auto_drive_percent >= 1 && s.auto_drive_percent <= 100 &&
           s.auto_turn_percent >= 1 && s.auto_turn_percent <= 100 &&
           s.auto_slow_percent >= 1 && s.auto_slow_percent <= s.auto_drive_percent &&
           s.ramp_ms >= 0 && s.ramp_ms <= 5000 &&
           s.reverse_ms >= 100 && s.reverse_ms <= 2000 &&
           s.turn_ms >= 200 && s.turn_ms <= 4000 &&
           s.resume_margin_mm >= 0 && s.resume_margin_mm <= 300 &&
           s.robot_width_mm >= 100 && s.robot_width_mm <= 1000 &&
           s.auto_limit_s >= 10 && s.auto_limit_s <= 3600;
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