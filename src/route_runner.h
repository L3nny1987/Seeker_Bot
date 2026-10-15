#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "motion_settings.h"

/* Fixed-size, allocation-free closed patrol routes in the current map frame. */
#define ROUTE_MAX_POINTS 64
#define ROUTE_CAPTURE_MM 200.0f         /* or the waypoint was passed along its edge */
#define ROUTE_APPROACH_MM 300.0f        /* above NAV_GOAL_RADIUS_MM: the runner sees arrival first */
#define ROUTE_MAX_DEVIATION_MM 600.0f   /* room for obstacle avoidance beside the edge */
#define ROUTE_PROGRESS_TIMEOUT_MS 45000
#define ROUTE_PACKET_HEADER 32
#define ROUTE_PACKET_MAX (ROUTE_PACKET_HEADER + ROUTE_MAX_POINTS * 4)

typedef struct { int16_t x, y; } route_point_t;
typedef struct {
    uint32_t generation, laps;          /* laps == 0: until explicitly stopped */
    uint16_t count;
    bool adaptive;
    route_point_t points[ROUTE_MAX_POINTS]; /* closure is implicit */
} route_definition_t;
typedef struct {
    route_definition_t definition;
    uint8_t score[ROUTE_MAX_POINTS];     /* incoming edge; heuristic, not probability */
    uint32_t completed, learned_laps, last_sequence;
    uint16_t target;
    uint8_t healthy_scans;
    bool configured, active, approaching, segment_clean, lap_clean, have_sequence;
    uint64_t last_ms, stalled_ms;
    float best_distance;
} route_runner_t;
typedef struct {
    float x, y, quality;
    uint32_t sequence, generation;
    uint64_t now_ms;
    bool healthy, trouble;
} route_sample_t;
typedef enum { ROUTE_WAIT, ROUTE_RUN, ROUTE_NEXT, ROUTE_DONE, ROUTE_FAILED } route_result_t;

const char *route_definition_error(const route_definition_t *definition);
/* Compact upload avoids a large cJSON tree. All multibyte fields little-endian. */
bool route_packet_decode(const uint8_t *data, unsigned size, route_definition_t *definition,
                         uint32_t *token, uint32_t *sequence);
/* NULL when every edge runs through known free cells and no mapped obstacle cell
   comes closer than radius_mm to it; otherwise "route_unexplored" or "route_obstacle". */
const char *route_grid_error(const route_definition_t *definition, const int8_t *grid,
                             unsigned size, unsigned resolution_mm, unsigned radius_mm);
void route_runner_clear(route_runner_t *runner);
void route_runner_start(route_runner_t *runner, const route_definition_t *definition, uint64_t now_ms);
void route_runner_stop(route_runner_t *runner, bool penalize);
void route_runner_trouble(route_runner_t *runner);
route_result_t route_runner_step(route_runner_t *runner, route_sample_t sample, const char **reason);
/* The map-path approach reached its (possibly shifted) goal: start the first edge. */
route_result_t route_runner_approach_done(route_runner_t *runner, const char **reason);
unsigned route_runner_confidence(const route_runner_t *runner);
int route_runner_power(const route_runner_t *runner, motion_settings_t settings,
                       float quality, unsigned clearance_mm, bool uncertain);
