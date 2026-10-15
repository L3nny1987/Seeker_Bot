#pragma once
#include "esp_err.h"
#include "navigation.h"
#include "motion_settings.h"
#include "mapper.h"
#include "route_runner.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum { ROBOT_IDLE, ROBOT_MANUAL, ROBOT_AUTO, ROBOT_ESTOP, ROBOT_FAULT } robot_mode_t;
typedef struct {
    robot_mode_t mode;
    const char *reason;
    bool network, commissioned, controller, lidar_healthy, link_ok;
    int drive, turn;
    navigation_view_t view;
    uint32_t scan_sequence;
    uint16_t coverage;
    uint32_t scan_age_ms;
    motion_settings_t settings;
    const char *auto_phase, *auto_trigger;
    int nav_heading;
    unsigned escape_attempts;
    uint32_t heartbeat_age_ms, auto_remaining_s;
    const char *strategy, *nav_detail;
    bool goal_set;
    float goal_x, goal_y;
    mapper_info_t map;
    bool route_active, route_configured, route_approaching, route_adaptive;
    uint32_t route_laps, route_completed, route_learned_laps;
    unsigned route_points, route_target, route_confidence;
    int route_power;
    uint32_t route_revision;
} robot_status_t;

/* Optional parameters of mode "auto" (NULL = explore). */
typedef struct {
    navigation_strategy_t strategy;
    float goal_x, goal_y;   /* map frame, mm (NAV_STRATEGY_GOAL) */
} robot_auto_request_t;

esp_err_t robot_manager_init(void);
void robot_manager_network(bool connected);
esp_err_t robot_manager_claim(uint32_t *token);
esp_err_t robot_manager_command(uint32_t token, uint32_t sequence, const char *command,
                                int drive, int turn, const char *mode,
                                const robot_auto_request_t *auto_request);
robot_status_t robot_manager_status(void);
esp_err_t robot_manager_configure(uint32_t token, uint32_t sequence, motion_settings_t settings);
/* Validates known-free geometry before taking the manager lock; never auto-resumes. */
esp_err_t robot_manager_start_route(uint32_t token, uint32_t sequence,
                                    const route_definition_t *definition, const char **reason);
void robot_manager_route_snapshot(route_runner_t *out);
const char *robot_mode_name(robot_mode_t mode);
