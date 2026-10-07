#include "../src/navigation.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

/* stop 200 + margin 50 = 250 mm threshold, slow zone 400 mm. */
static motion_settings_t config = {100,100,200,100,400,90,100,85,1000,800,1600,50,250,300};

static navigation_view_t open_view(uint16_t free_mm)
{
    navigation_view_t v = {.front_mm=2000,.rear_mm=2000,.left_mm=1500,.right_mm=800,
        .nearest_mm=800,.front_known=true,.rear_known=true,.rotation_known=true,
        .left_known=true,.right_known=true,.rear_free_mm=2000};
    for (int k = 0; k < NAV_HEADINGS; k++) v.free_mm[k] = free_mm;
    return v;
}

int main(void)
{
    navigation_auto_t s;
    navigation_auto_reset(&s);
    navigation_view_t v = open_view(2000);
    navigation_plan_t p = navigation_auto_step(&s,&v,config,0,1);
    assert(p.drive==90 && p.turn==0 && !strcmp(p.reason,"cruise"));
    v = open_view(300);
    p = navigation_auto_step(&s,&v,config,20,2);
    assert(p.drive==85 && !p.turn);               /* slow zone */
    v = open_view(2000); v.uncertain_mask = 1u << NAV_CENTER;
    p = navigation_auto_step(&s,&v,config,40,3);
    assert(p.drive==85);                          /* unmeasured corridor: slow */

    /* Steer toward the open corridor while driving, with hysteresis. */
    navigation_auto_reset(&s);
    v = open_view(1000); v.free_mm[NAV_CENTER+4] = 3000;
    p = navigation_auto_step(&s,&v,config,0,1);
    assert(p.drive==90 && !p.turn);
    p = navigation_auto_step(&s,&v,config,400,2);
    assert(p.drive==90 && p.turn==100 && !strcmp(p.reason,"cruise_steer"));
    v = open_view(2000);
    p = navigation_auto_step(&s,&v,config,500,3);
    assert(p.turn==100);                          /* minimum steering hold */
    p = navigation_auto_step(&s,&v,config,800,4);
    assert(!p.turn && !strcmp(p.reason,"cruise"));

    /* Front blocked, free gap on the right: brake, rotate in place to it. */
    navigation_auto_reset(&s);
    v = open_view(190); v.front_mm = 190;
    for (int k = NAV_CENTER+3; k < NAV_HEADINGS; k++) v.free_mm[k] = 2000;
    p = navigation_auto_step(&s,&v,config,0,1);
    assert(!p.drive && !p.turn && s.phase==NAV_BRAKE && !strcmp(s.trigger,"turn_to_gap"));
    p = navigation_auto_step(&s,&v,config,160,2);
    assert(!p.drive && p.turn==100 && !strcmp(p.reason,"escape_turn"));
    v = open_view(1000);
    p = navigation_auto_step(&s,&v,config,400,3);
    assert(p.turn==100 && s.clear_scans==1);
    p = navigation_auto_step(&s,&v,config,420,3);
    assert(p.turn==100 && s.clear_scans==1);      /* same scan is not progress */
    p = navigation_auto_step(&s,&v,config,1500,4);
    assert(!p.turn && !p.drive && s.phase==NAV_BRAKE);
    (void)navigation_auto_step(&s,&v,config,1700,5);
    p = navigation_auto_step(&s,&v,config,1720,5);
    assert(p.drive==90 && s.phase==NAV_CRUISE);

    /* Nothing free ahead: reverse; the rear corridor vetoes on every tick. */
    navigation_auto_reset(&s);
    v = open_view(190);
    p = navigation_auto_step(&s,&v,config,0,1);
    assert(s.phase==NAV_BRAKE && s.next_phase==NAV_REVERSE && !strcmp(s.trigger,"front_blocked"));
    p = navigation_auto_step(&s,&v,config,160,2);
    assert(p.drive==-85 && !p.turn && !strcmp(p.reason,"escape_reverse"));
    v.rear_free_mm = 150;
    p = navigation_auto_step(&s,&v,config,180,3);
    assert(!p.drive && !p.turn && s.next_phase==NAV_TURN);
    p = navigation_auto_step(&s,&v,config,340,4);
    assert(p.turn==-100);                         /* more space on the left */

    /* No measured way out: stand still. Rotation alone can still rescue. */
    navigation_auto_reset(&s);
    v = open_view(190); v.rear_free_mm = 190; v.rotation_known = false;
    p = navigation_auto_step(&s,&v,config,0,1);
    assert(!p.drive && !p.turn && !strcmp(p.reason,"escape_no_safe_path"));
    v.rotation_known = true;
    (void)navigation_auto_step(&s,&v,config,20,2);
    p = navigation_auto_step(&s,&v,config,180,3);
    assert(p.turn==-100 && !p.drive);
    v.nearest_mm = 90;
    p = navigation_auto_step(&s,&v,config,200,4);
    assert(!p.turn && !p.drive);

    /* Static trap: alternating directions, bounded series, pause, retry. */
    navigation_auto_reset(&s);
    v = open_view(190); v.rear_free_mm = 190;
    bool left = false, right = false, blocked = false, retried = false;
    for (unsigned t = 0; t < 20000; t += 20) {
        p = navigation_auto_step(&s,&v,config,t,t/160+1);
        left |= p.turn < 0; right |= p.turn > 0;
        if (s.phase == NAV_BLOCKED) { blocked = true; assert(s.attempts == 4); }
        retried |= blocked && p.turn != 0;
        assert(!p.drive);
    }
    assert(left && right && blocked && retried);
    v = open_view(1000);
    bool recovered = false;
    for (unsigned t = 20000; t < 25000 && !recovered; t += 20) {
        p = navigation_auto_step(&s,&v,config,t,t/160+1);
        recovered = p.drive == 90;
    }
    assert(recovered);

    /* Stuck detection: distances unchanged while driving -> escape. */
    navigation_auto_reset(&s);
    v = open_view(1000); v.front_mm = 1000; v.rear_mm = 1000;
    for (unsigned t = 0; t < 4500; t += 20) {
        p = navigation_auto_step(&s,&v,config,t,t/160+1);
        assert(p.drive==90);
    }
    p = navigation_auto_step(&s,&v,config,4500,100);
    assert(!p.drive && s.phase==NAV_BRAKE && s.next_phase==NAV_REVERSE);
    assert(!strcmp(s.trigger,"stuck_no_progress"));
    /* Real travel changes the distances: no false stuck detection. */
    navigation_auto_reset(&s);
    for (unsigned t = 0; t < 8000; t += 20) {
        v = open_view(1000); v.front_mm = 2000 - t / 10; v.rear_mm = 500 + t / 10;
        p = navigation_auto_step(&s,&v,config,t,t/160+1);
        assert(p.drive==90);
    }
    navigation_auto_reset(&s);
    assert(s.phase==NAV_CRUISE && !s.attempts && !s.turn_direction && !strcmp(s.trigger,"none"));
    puts("Escape tests: cruise/slow, steering, turn-to-gap, reverse veto, no safe path, trap retry, stuck detection passed.");
}