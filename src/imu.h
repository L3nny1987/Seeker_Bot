#pragma once
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

/* MPU6050 gyro/accelerometer on I2C (see seeker_pins.h).
 * The yaw axis is the measured gravity direction, so the mounting
 * orientation does not matter. Yaw is CCW positive (same as the SLAM).
 * The sensor is optional and hot-pluggable: without it everything runs
 * LiDAR-only as before. */
typedef struct {
    bool present, ready, calibrating;
    uint8_t address, who_am_i;
    float rate_dps;          /* yaw rate, CCW + */
    float bias_dps;          /* gyro bias on the yaw axis */
    float yaw_deg;           /* integrated yaw since calibration */
    float tilt_deg;          /* deviation from the calibrated "up" */
    float shock_g;           /* peak acceleration deviation, last 0.5 s */
    float sample_hz;         /* measured sensor sample rate */
    uint32_t samples, overflows, errors, epoch;
} imu_status_t;

esp_err_t imu_init(void);
/* Motor command active: no bias calibration while the robot may move. */
void imu_set_motion(bool moving);
/* Integrated yaw (rad, CCW, unwrapped) at an esp_timer time. epoch changes
   on every recalibration: only compare yaws of the same epoch. */
bool imu_yaw_at(int64_t t_us, double *yaw, uint32_t *epoch);
imu_status_t imu_status(void);
