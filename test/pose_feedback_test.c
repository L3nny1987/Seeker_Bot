#include "../src/pose_feedback.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    pose_feedback_t ref = {1000000, 10.0, 7, true};
    float theta = 0;
    assert(pose_feedback_fresh(ref, 1000000));
    assert(pose_feedback_fresh(ref, 1300000));
    assert(!pose_feedback_fresh(ref, 1300001));
    assert(!pose_feedback_fresh(ref, 999999));
    assert(pose_feedback_heading(ref, 1100000, 9.85, 7, 0.2f, 0.01f, &theta));
    assert(fabsf(theta - 0.049f) < 0.00001f);
    assert(!pose_feedback_heading(ref, 1100000, 9.85, 8, 0, 0, &theta));
    assert(!pose_feedback_heading(ref, 1400000, 9.85, 7, 0, 0, &theta));
    assert(!pose_feedback_heading(ref, 1100000, NAN, 7, 0, 0, &theta));
    ref.gyro_valid = false;
    assert(!pose_feedback_heading(ref, 1100000, 9.85, 7, 0, 0, &theta));
    puts("Pose feedback: fresh heading, bias, stale data and IMU epochs passed");
}
