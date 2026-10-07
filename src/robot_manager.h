#pragma once
#include "esp_err.h"
#include "navigation.h"
#include "motion_settings.h"
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
} robot_status_t;

esp_err_t robot_manager_init(void);
void robot_manager_network(bool connected);
esp_err_t robot_manager_claim(uint32_t *token);
esp_err_t robot_manager_command(uint32_t token, uint32_t sequence, const char *command,
                                int drive, int turn, const char *mode);
robot_status_t robot_manager_status(void);
esp_err_t robot_manager_configure(uint32_t token, uint32_t sequence, motion_settings_t settings);
const char *robot_mode_name(robot_mode_t mode);