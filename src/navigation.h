#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "motion_settings.h"

/* Independent of ESP-IDF so geometry and fail-closed behavior can be tested.
 * Robot frame: 0 deg = front, positive angles/headings/turns = right.
 * Map/world frame (SLAM): x = initial forward, y = left, theta CCW (rad). */
#define NAV_HEADING_STEP_DEG 10
#define NAV_HEADINGS 19                 /* -90 .. +90 deg */
#define NAV_CENTER (NAV_HEADINGS / 2)
#define NAV_HORIZON_MM 4000             /* corridor length cap */
#define NAV_UNCERTAIN_MM 700            /* cap for mostly unmeasured corridors */

static inline int navigation_heading_deg(int index)
{
    return index * NAV_HEADING_STEP_DEG - 90;
}

/* Straight wall fitted to the side points. angle_deg > 0: the wall moves
   away from the robot when driving forward. */
typedef struct {
    bool valid;
    uint16_t distance_mm;
    int16_t angle_deg;
} navigation_wall_t;

typedef struct {
    uint16_t front_mm, rear_mm, left_mm, right_mm, nearest_mm;   /* raw, from the LiDAR */
    bool front_known, rear_known, rotation_known, left_known, right_known;
    /* Free straight-line travel of the front bumper in a corridor of the
       configured width, per heading (body front already subtracted). */
    uint16_t free_mm[NAV_HEADINGS];
    uint32_t uncertain_mask;            /* bit k: corridor k mostly unmeasured */
    uint16_t rear_free_mm;              /* travel of the rear bumper backwards */
    bool rear_uncertain;
    /* Margin of the body footprint when rotating in place. */
    uint16_t swing_mm;
    /* Bumper travel in a corridor of the bare body width (tight spots). */
    uint16_t tight_front_mm, tight_rear_mm;
    navigation_wall_t wall_left, wall_right;
    /* Filled by the caller from SLAM (optional). */
    bool pose_valid;
    float pose_x, pose_y, pose_theta;   /* mm, mm, rad */
    int16_t explore_bonus[NAV_HEADINGS];
    /* Map path to the goal (NAV_STRATEGY_PATH): next waypoint in line of
       sight and the planned goal (may be moved off a wall). */
    bool route_valid;
    float route_way_x, route_way_y, route_goal_x, route_goal_y;
    /* SLAM health: lost, or weak (young map / failing matches). Map-based
       strategies pause briefly so the map can lock on standing still. */
    bool map_lost, map_weak;
} navigation_view_t;

typedef struct {
    int drive, turn;
    const char *reason;
    bool done;                          /* strategy finished: stop auto */
    bool tight;                         /* short pulses: limit the start kick */
} navigation_plan_t;

/* Robot body relative to the LiDAR centre (mm). The LiDAR sits on the
   centre line; rotation is assumed around the middle of the body. */
typedef struct { int front_mm, rear_mm, half_width_mm; } navigation_body_t;

/* Fill single missing returns of the current scan from the previous scan
   (previous may be NULL). Never removes a measured obstacle. */
void navigation_fuse(const uint16_t current[360], const uint16_t *previous, uint16_t out[360]);
navigation_view_t navigation_analyze(const uint16_t distance[360], int front_deg, bool clockwise,
                                     int corridor_width_mm, navigation_body_t body);

typedef enum {
    NAV_STRATEGY_EXPLORE, NAV_STRATEGY_WALL_LEFT, NAV_STRATEGY_WALL_RIGHT, NAV_STRATEGY_GOAL,
    NAV_STRATEGY_PATH                   /* goal via the SLAM map path */
} navigation_strategy_t;

typedef enum {
    NAV_CRUISE, NAV_BRAKE, NAV_REVERSE, NAV_TURN, NAV_RECHECK, NAV_BLOCKED, NAV_WIGGLE
} navigation_phase_t;
typedef enum { NAV_GOAL_SEEK, NAV_GOAL_BOUNDARY } navigation_goal_phase_t;

typedef struct {
    /* Kept across navigation_auto_reset(). */
    navigation_strategy_t strategy;
    float goal_x, goal_y;
    /* Reactive layer. */
    navigation_phase_t phase, next_phase;
    uint64_t since_ms;
    uint32_t sequence;
    unsigned attempts, clear_scans;
    int turn_direction;
    int heading_deg;                    /* last selected target heading */
    float heading_filtered;
    bool cruising, snapshot_valid;
    bool driving;                       /* last cruise command drove forward */
    uint64_t cruise_since_ms, progress_ms;
    uint16_t snapshot[3];
    float progress_x, progress_y, progress_theta;
    const char *trigger;                /* why the last maneuver started */
    const char *detail;                 /* active behaviour */
    uint16_t blocked_front, blocked_rear, blocked_nearest;
    /* Tight-spot wiggling: short pulses with bare-body clearances. */
    unsigned wiggle_cycles, wiggle_slot;
    int wiggle_drive, wiggle_turn;
    /* Wall following. */
    int wall_side;                      /* +1 right, -1 left */
    uint64_t wall_seen_ms;
    /* Bug2. */
    navigation_goal_phase_t goal_phase;
    bool line_valid, last_valid;
    float line_x, line_y, hit_x, hit_y, hit_distance, travel_mm, last_x, last_y;
    /* Map path. */
    bool route_missing;
    uint64_t route_missing_ms;
    bool route_turning;                 /* rotating in place toward the waypoint */
    bool correcting;                    /* course correction active (hysteresis) */
    /* SLAM pause. */
    bool slam_waiting, slam_retry;
    uint64_t slam_wait_ms;
} navigation_auto_t;

void navigation_auto_reset(navigation_auto_t *state);
void navigation_auto_configure(navigation_auto_t *state, navigation_strategy_t strategy,
                               float goal_x, float goal_y);
const char *navigation_phase_name(navigation_phase_t phase);
const char *navigation_strategy_name(navigation_strategy_t strategy);
navigation_plan_t navigation_auto_step(navigation_auto_t *state, const navigation_view_t *view,
    motion_settings_t settings, uint64_t now_ms, uint32_t scan_sequence);
