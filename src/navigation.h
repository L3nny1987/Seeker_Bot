#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "motion_settings.h"

/* Independent of ESP-IDF so geometry and fail-closed behavior can be tested.
 * Robot frame: 0 deg = front, positive angles/headings/turns = right. */
#define NAV_HEADING_STEP_DEG 10
#define NAV_HEADINGS 19                 /* -90 .. +90 deg */
#define NAV_CENTER (NAV_HEADINGS / 2)
#define NAV_HORIZON_MM 4000             /* corridor length cap */
#define NAV_UNCERTAIN_MM 700            /* cap for mostly unmeasured corridors */

static inline int navigation_heading_deg(int index)
{
    return index * NAV_HEADING_STEP_DEG - 90;
}

typedef struct {
    uint16_t front_mm, rear_mm, left_mm, right_mm, nearest_mm;
    bool front_known, rear_known, rotation_known, left_known, right_known;
    /* Free straight-line travel of a robot-wide corridor per heading. */
    uint16_t free_mm[NAV_HEADINGS];
    uint32_t uncertain_mask;            /* bit k: corridor k mostly unmeasured */
    uint16_t rear_free_mm;
    bool rear_uncertain;
} navigation_view_t;

typedef struct {
    int drive, turn;
    const char *reason;
} navigation_plan_t;

/* Fill single missing returns of the current scan from the previous scan
   (previous may be NULL). Never removes a measured obstacle. */
void navigation_fuse(const uint16_t current[360], const uint16_t *previous, uint16_t out[360]);
navigation_view_t navigation_analyze(const uint16_t distance[360], int front_deg, bool clockwise,
                                     int robot_width_mm);

typedef enum { NAV_CRUISE, NAV_BRAKE, NAV_REVERSE, NAV_TURN, NAV_RECHECK, NAV_BLOCKED } navigation_phase_t;
typedef struct {
    navigation_phase_t phase, next_phase;
    uint64_t since_ms;
    uint32_t sequence;
    unsigned attempts, clear_scans;
    int turn_direction;
    int heading_deg;                    /* last selected target heading */
    int steer;                          /* cruise steering: -1, 0, 1 */
    uint64_t steer_since_ms;
    bool cruising, snapshot_valid;
    uint64_t cruise_since_ms, progress_ms;
    uint16_t snapshot[3];
    const char *trigger;                /* why the last maneuver started */
    uint16_t blocked_front, blocked_rear, blocked_nearest;
} navigation_auto_t;

void navigation_auto_reset(navigation_auto_t *state);
const char *navigation_phase_name(navigation_phase_t phase);
navigation_plan_t navigation_auto_step(navigation_auto_t *state, const navigation_view_t *view,
    motion_settings_t settings, uint64_t now_ms, uint32_t scan_sequence);