#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

/* A scan pose must not remain usable indefinitely when the mapper stalls. */
#define POSE_FEEDBACK_MAX_AGE_US 300000
typedef struct {
    int64_t timestamp_us;
    double yaw;
    uint32_t epoch;
    bool gyro_valid;
} pose_feedback_t;

static inline bool pose_feedback_fresh(pose_feedback_t ref, int64_t now_us)
{
    return ref.timestamp_us > 0 && now_us >= ref.timestamp_us &&
           now_us - ref.timestamp_us <= POSE_FEEDBACK_MAX_AGE_US;
}

/* Advance heading only, using measured gyro motion since the matched scan.
   Never extrapolate position from an unverified velocity. */
static inline bool pose_feedback_heading(pose_feedback_t ref, int64_t now_us,
    double yaw, uint32_t epoch, float matched_theta, float bias, float *theta)
{
    if (!pose_feedback_fresh(ref, now_us) || !ref.gyro_valid || epoch != ref.epoch ||
        !isfinite(yaw) || !isfinite(ref.yaw)) return false;
    float a = matched_theta + (float)(yaw - ref.yaw) - bias * (now_us - ref.timestamp_us) * 1e-6f;
    *theta = atan2f(sinf(a), cosf(a));
    return true;
}
