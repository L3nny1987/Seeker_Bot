#include "../src/motion_settings.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    motion_settings_t s = {200,200,500,550,1200,30,30,20,1000,800,1600,50,250,300};
    assert(motion_settings_valid(s));
    s.manual_stop_mm = 99; assert(!motion_settings_valid(s)); s.manual_stop_mm = 200;
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
    puts("Motion settings: validation, ramp timing, immediate stop and reversal passed.");
}