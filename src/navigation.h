#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Independent of ESP-IDF so geometry and fail-closed behavior can be tested. */
typedef struct {
    uint16_t front_mm, rear_mm, left_mm, right_mm, nearest_mm;
    bool front_known, rear_known, rotation_known;
} navigation_view_t;

typedef struct {
    int drive, turn;
    const char *reason;
} navigation_plan_t;

navigation_view_t navigation_analyze(const uint16_t distance[360], int front_deg, bool clockwise);
navigation_plan_t navigation_plan(navigation_view_t view, int preferred_turn,
    unsigned stop_mm, unsigned rotate_mm, unsigned slow_mm, int drive_limit, int turn_limit);