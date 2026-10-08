#pragma once
#include "esp_err.h"
#include "slam.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool valid, lost;
    float x, y, theta, quality;
    uint32_t scans, integrated, rejected, update_ms;
    unsigned known_cells;
    uint16_t path_count;
    bool imu;                    /* last scan used the gyro */
    float heading_sigma;         /* rad, gyro heading filter */
    float gyro_bias;             /* rad/s, residual bias learned by the filter */
    slam_target_t target;        /* exploration target, or path to the goal in goal mode */
    uint16_t avoided;            /* planning cells blacklisted as unreachable */
} mapper_info_t;

/* SLAM task on core 1: builds the occupancy map from every LiDAR scan. */
esp_err_t mapper_init(void);
/* Current motor command; selects the motion model for scan matching. */
void mapper_set_motion(int drive, int turn);
/* Go-to-point mode: target = shortest map path to (x, y) instead of the
   exploration target. Cheap; may be called every control tick. */
void mapper_set_goal(bool active, float x, float y);
mapper_info_t mapper_info(void);
/* Per robot heading: novelty along the ray plus a strong pull toward the
   waypoint of the current exploration target. */
void mapper_explore_bonus(int16_t out[SLAM_HEADINGS]);
void mapper_request_reset(void);
/* Read-only map access for streaming (cells may change while reading). */
const int8_t *mapper_grid(void);
uint16_t mapper_path(int16_t (*out)[2], uint16_t max);
