#include "../src/navigation.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static const motion_settings_t cfg = {
    .auto_stop_mm=100, .resume_margin_mm=50, .auto_slow_mm=300,
    .auto_drive_percent=90, .auto_slow_percent=85, .auto_turn_percent=100,
    .steer_min_percent=80, .turn_min_percent=80, .turn_kick_ms=300,
    .turn_ramp_ms=400, .ramp_ms=1000, .turn_ms=1600, .reverse_ms=500
};

static void run(bool gyro)
{
    navigation_auto_t s;
    navigation_auto_configure(&s, NAV_STRATEGY_PATH, 3000, 0);
    navigation_view_t v = {.front_known=true, .rear_known=true, .rotation_known=true,
        .front_mm=4000, .rear_mm=4000, .rear_free_mm=3500, .swing_mm=1000,
        .pose_valid=true, .route_valid=true, .route_way_x=3000, .route_goal_x=3000};
    for (int i = 0; i < NAV_HEADINGS; i++) v.free_mm[i] = 3500;
    motion_axis_t motor = {0};
    float theta=0.2f, matched=theta, x=0, y=0, max_overshoot=0;
    int actual=0, reversals=0, last_direction=0, max_pulse=0, pulse=0;
    for (unsigned t=0; t<6000; t+=20) {
        if (t % 160 == 0) matched = theta;
        v.pose_sequence = t / 160 + 1;
        v.pose_x=x; v.pose_y=y; v.pose_theta=gyro ? theta : matched;
        v.heading_live=gyro; v.yaw_rate=-actual * 0.0148f;
        navigation_plan_t p = navigation_auto_step(&s, &v, cfg, t, t / 160 + 1);
        int kick = p.precision ? 20 : cfg.turn_kick_ms;
        actual = motion_axis_step(&motor, p.turn, cfg.turn_min_percent, kick,
                                  p.precision ? 0 : cfg.turn_ramp_ms, 20);
        if (actual) {
            pulse += 20;
            int direction = actual > 0 ? 1 : -1;
            if (last_direction && direction != last_direction) reversals++;
            last_direction = direction;
        } else pulse=0;
        if (pulse > max_pulse) max_pulse=pulse;
        theta -= actual * 0.0148f * 0.02f;
        x += (p.drive ? 3.0f : 0) * cosf(theta);
        y += (p.drive ? 3.0f : 0) * sinf(theta);
        float error = theta - atan2f(-y, 3000-x);
        if (-error > max_overshoot) max_overshoot=-error;
    }
    printf("course %s: max pulse %d ms, overshoot %.1f deg, reversals %d, lateral %.0f mm\n",
           gyro ? "gyro" : "scan", max_pulse, max_overshoot * 57.29578f, reversals, y);
    assert(max_pulse <= 80 && max_overshoot < 0.10f && reversals <= 2 && fabsf(y) < 100);
}

static void feedback_gate_test(void)
{
    navigation_auto_t s;
    navigation_auto_configure(&s, NAV_STRATEGY_PATH, 3000, 0);
    navigation_view_t v = {.front_known=true, .rear_known=true, .rotation_known=true,
        .rear_free_mm=3500, .swing_mm=1000, .pose_valid=true, .pose_sequence=1,
        .pose_theta=0.2f, .route_valid=true, .route_way_x=3000, .route_goal_x=3000};
    for (int i=0; i<NAV_HEADINGS; i++) v.free_mm[i]=3500;
    navigation_plan_t p = navigation_auto_step(&s, &v, cfg, 0, 1);
    assert(p.turn > 0 && p.precision);
    p = navigation_auto_step(&s, &v, cfg, 80, 1);
    assert(!p.turn);
    /* More LiDAR scans alone cannot authorize another course pulse when
       the mapper still publishes the same old heading. */
    p = navigation_auto_step(&s, &v, cfg, 240, 3);
    assert(!p.turn);
    v.pose_sequence=2; v.pose_theta=-0.2f;
    p = navigation_auto_step(&s, &v, cfg, 260, 3);
    assert(p.turn < 0);
    v.pose_theta=0.2f;
    p = navigation_auto_step(&s, &v, cfg, 280, 3);
    assert(!p.turn);                            /* sign flip stops, never reverses mid-pulse */
}

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    run(false);
    run(true);
    feedback_gate_test();
    puts("Closed-loop course corrections with real minimum PWM and kick timing passed");
}
