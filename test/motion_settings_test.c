#include "../src/motion_settings.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    motion_settings_t s = {200,200,500,550,1200,30,30,20,1000,800,1600,50,250,300,400,20,350,85,80,1000,300};
    assert(motion_settings_valid(s));
    s.manual_stop_mm = 99; assert(motion_settings_valid(s));
    s.manual_stop_mm = 19; assert(!motion_settings_valid(s)); s.manual_stop_mm = 200;
    s.auto_rotate_mm = 9; assert(!motion_settings_valid(s)); s.auto_rotate_mm = 550;
    s.drive_min_percent = 101; assert(!motion_settings_valid(s)); s.drive_min_percent = 85;
    s.turn_min_percent = -1; assert(!motion_settings_valid(s)); s.turn_min_percent = 80;
    s.drive_kick_ms = 3001; assert(!motion_settings_valid(s)); s.drive_kick_ms = 1000;
    s.turn_kick_ms = -1; assert(!motion_settings_valid(s)); s.turn_kick_ms = 300;
    s.auto_slow_percent = 31; assert(!motion_settings_valid(s)); s.auto_slow_percent = 20;
    s.auto_slow_mm = 550; assert(!motion_settings_valid(s)); s.auto_slow_mm = 1200;
    s.auto_drive_percent = 101; assert(!motion_settings_valid(s)); s.auto_drive_percent = 30;
    s.ramp_ms = -1; assert(!motion_settings_valid(s));
    s.ramp_ms = 1000;
    s.reverse_ms = 2001; assert(!motion_settings_valid(s));s.reverse_ms=800;
    s.turn_ms = 199; assert(!motion_settings_valid(s));s.turn_ms=1600;
    s.resume_margin_mm = -1; assert(!motion_settings_valid(s));s.resume_margin_mm=50;
    s.robot_width_mm = 99; assert(!motion_settings_valid(s));
    s.robot_width_mm = 1001; assert(!motion_settings_valid(s)); s.robot_width_mm = 250;
    s.auto_limit_s = 9; assert(!motion_settings_valid(s));
    s.auto_limit_s = 3601; assert(!motion_settings_valid(s)); s.auto_limit_s = 300;
    s.turn_ramp_ms = 5001; assert(!motion_settings_valid(s)); s.turn_ramp_ms = 400;
    s.steer_min_percent = 31; assert(!motion_settings_valid(s)); s.steer_min_percent = 20;
    s.wall_distance_mm = 174; assert(!motion_settings_valid(s));
    s.wall_distance_mm = 1501; assert(!motion_settings_valid(s)); s.wall_distance_mm = 350;
    assert(motion_settings_valid(s));
    const int times[] = {100,1000,1500,5000};
    for (unsigned t = 0; t < sizeof(times)/sizeof(times[0]); t++) {
        int current = 0, remainder = 0;
        for (int elapsed = 0; elapsed < times[t]; elapsed += 20)
            current = motion_ramp(current,100,times[t],20,&remainder);
        assert(current == 100);
        assert(motion_ramp(current,0,times[t],20,&remainder) == 0);
        assert(motion_ramp(100,-100,times[t],20,&remainder) == 0);
        current = 0;
        for (int elapsed = 0; elapsed < times[t]; elapsed += 20)
            current = motion_ramp(current,-100,times[t],20,&remainder);
        assert(current == -100);
    }
    int remainder = 0;
    assert(motion_ramp(0,100,0,20,&remainder) == 100);
    assert(motion_ramp(24,30,1000,20,&remainder) == 26);

    /* Motor axis: minimum PWM, start kick, immediate stop and reversal. */
    motion_axis_t a = {0};
    assert(motion_axis_step(&a, 30, 85, 0, 1000, 20) == 85);   /* never below min */
    assert(motion_axis_step(&a, 0, 85, 0, 1000, 20) == 0);
    assert(motion_axis_step(&a, 120, 85, 0, 1000, 20) == 100);
    a = (motion_axis_t){0};
    int value = 0, kicked = 0;
    for (int t = 0; t < 1000; t += 20) {
        value = motion_axis_step(&a, 90, 85, 1000, 1000, 20);
        kicked += value == 100;
    }
    assert(kicked == 50);                                      /* 1 s at 100 % */
    for (int t = 0; t < 200; t += 20) value = motion_axis_step(&a, 90, 85, 1000, 1000, 20);
    assert(value == 90);                                       /* then the set speed */
    assert(motion_axis_step(&a, 0, 85, 1000, 1000, 20) == 0);  /* stop at once */
    a = (motion_axis_t){0};
    (void)motion_axis_step(&a, 90, 85, 1000, 1000, 20);
    assert(motion_axis_step(&a, -90, 85, 1000, 1000, 20) == 0); /* reversal: stop first */
    assert(motion_axis_step(&a, -90, 85, 1000, 1000, 20) == -100);
    a = (motion_axis_t){0};
    assert(motion_axis_step(&a, -40, 80, 0, 400, 20) == -80);
    puts("Motion settings: validation, ramp timing, immediate stop and reversal passed.");
}