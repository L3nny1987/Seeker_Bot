#include "../src/navigation.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

/* stop 200 + margin 50 = 250 mm threshold, slow zone 400 mm, steer 50..100 %,
   wall distance 350 mm. */
static motion_settings_t config = {
    .manual_stop_mm = 100, .manual_rotate_mm = 100, .auto_stop_mm = 200, .auto_rotate_mm = 100,
    .auto_slow_mm = 400, .auto_drive_percent = 90, .auto_turn_percent = 100,
    .auto_slow_percent = 85, .ramp_ms = 1000, .reverse_ms = 800, .turn_ms = 1600,
    .resume_margin_mm = 50, .robot_width_mm = 250, .auto_limit_s = 300, .turn_ramp_ms = 400,
    .steer_min_percent = 50, .wall_distance_mm = 350, .drive_min_percent = 85,
    .turn_min_percent = 80, .drive_kick_ms = 1000, .turn_kick_ms = 300};

static navigation_view_t open_view(uint16_t free_mm)
{
    navigation_view_t v = {.front_mm=2000,.rear_mm=2000,.left_mm=1500,.right_mm=800,
        .nearest_mm=800,.front_known=true,.rear_known=true,.rotation_known=true,
        .left_known=true,.right_known=true,.rear_free_mm=2000,.swing_mm=800,
        .tight_front_mm=2000,.tight_rear_mm=2000};
    for (int k = 0; k < NAV_HEADINGS; k++) v.free_mm[k] = free_mm;
    return v;
}

static void pose(navigation_view_t *v, float x, float y, float theta)
{
    v->pose_valid = true; v->pose_x = x; v->pose_y = y; v->pose_theta = theta;
}

static void explore_tests(void)
{
    navigation_auto_t s = {0};
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

    /* Proportional steering toward the open corridor, smoothed per scan. */
    navigation_auto_reset(&s);
    v = open_view(1000); v.free_mm[NAV_CENTER+4] = 3000;
    p = navigation_auto_step(&s,&v,config,0,1);
    assert(p.turn==67 && p.drive==89 && !strcmp(p.reason,"cruise_steer"));
    p = navigation_auto_step(&s,&v,config,20,1);
    assert(p.turn==67);                           /* same scan: same command */
    p = navigation_auto_step(&s,&v,config,100,2);
    assert(p.turn==75 && p.drive==89);
    v = open_view(2000);
    p = navigation_auto_step(&s,&v,config,200,3);
    assert(p.turn==63);
    p = navigation_auto_step(&s,&v,config,300,4);
    assert(p.turn==56);
    p = navigation_auto_step(&s,&v,config,400,5);
    assert(!p.turn && p.drive==90 && !strcmp(p.reason,"cruise"));

    /* Front blocked, free gap on the right: rotate at once, no brake pause. */
    navigation_auto_reset(&s);
    v = open_view(190); v.front_mm = 190;
    for (int k = NAV_CENTER+3; k < NAV_HEADINGS; k++) v.free_mm[k] = 2000;
    p = navigation_auto_step(&s,&v,config,0,1);
    assert(!p.drive && p.turn==100 && s.phase==NAV_TURN && !strcmp(s.trigger,"turn_to_gap"));
    v = open_view(1000);
    p = navigation_auto_step(&s,&v,config,100,2);
    assert(p.turn==100 && s.clear_scans==1);      /* minimum turn time */
    p = navigation_auto_step(&s,&v,config,120,2);
    assert(p.turn==100 && s.clear_scans==1);      /* same scan is not progress */
    p = navigation_auto_step(&s,&v,config,200,2);
    assert(p.drive==90 && !p.turn && s.phase==NAV_CRUISE);

    /* Turn timeout: turn_ms. */
    navigation_auto_reset(&s);
    v = open_view(190); v.front_mm = 190;
    for (int k = NAV_CENTER+3; k < NAV_HEADINGS; k++) v.free_mm[k] = 2000;
    (void)navigation_auto_step(&s,&v,config,0,1);
    v = open_view(190);
    p = navigation_auto_step(&s,&v,config,1580,30);
    assert(p.turn==100 && s.phase==NAV_TURN);
    p = navigation_auto_step(&s,&v,config,1600,31);
    assert(s.phase==NAV_BRAKE && s.next_phase==NAV_REVERSE);

    /* Nothing free ahead: brake, reverse; the rear corridor vetoes per tick. */
    navigation_auto_reset(&s);
    v = open_view(190);
    p = navigation_auto_step(&s,&v,config,0,1);
    assert(s.phase==NAV_BRAKE && s.next_phase==NAV_REVERSE && !strcmp(s.trigger,"front_blocked"));
    p = navigation_auto_step(&s,&v,config,160,2);
    assert(p.drive==-85 && !p.turn && !strcmp(p.reason,"escape_reverse"));
    v.rear_free_mm = 150;
    p = navigation_auto_step(&s,&v,config,180,3);
    assert(!p.drive && p.turn==-100 && s.phase==NAV_TURN);   /* more space left */

    /* No measured way out: stand still. Rotation alone can still rescue. */
    navigation_auto_reset(&s);
    v = open_view(190); v.rear_free_mm = 190; v.rotation_known = false;
    p = navigation_auto_step(&s,&v,config,0,1);
    assert(!p.drive && !p.turn && !strcmp(p.reason,"escape_no_safe_path"));
    v.rotation_known = true;
    p = navigation_auto_step(&s,&v,config,20,2);
    assert(p.turn==-100 && !p.drive);
    v.swing_mm = 90;
    p = navigation_auto_step(&s,&v,config,40,3);
    assert(!p.turn && !p.drive && !strcmp(p.reason,"escape_rotation_blocked"));

    /* Static trap: alternating directions, then wiggling with tight
       clearances, bounded series, pause, retry. */
    navigation_auto_reset(&s);
    v = open_view(190); v.rear_free_mm = 190;
    bool left = false, right = false, blocked = false, retried = false, wiggled = false;
    for (unsigned t = 0; t < 40000; t += 20) {
        p = navigation_auto_step(&s,&v,config,t,t/160+1);
        left |= p.turn < 0; right |= p.turn > 0;
        wiggled |= s.phase == NAV_WIGGLE && p.tight && p.turn;
        if (s.phase == NAV_BLOCKED) { blocked = true; assert(s.attempts == 4); }
        retried |= blocked && p.turn != 0;
        if (p.drive) assert(s.phase == NAV_WIGGLE && p.tight);
    }
    assert(left && right && blocked && retried && wiggled);
    v = open_view(1000);
    bool recovered = false;
    for (unsigned t = 40000; t < 45000 && !recovered; t += 20) {
        p = navigation_auto_step(&s,&v,config,t,t/160+1);
        recovered = p.drive == 90;
    }
    assert(recovered);
}

static void wiggle_tests(void)
{
    navigation_auto_t s = {0};
    navigation_plan_t p;
    /* Normal clearances exhausted, bare body still has a few cm each way. */
    navigation_view_t v = open_view(190);
    v.rear_free_mm = 190; v.tight_front_mm = 50; v.tight_rear_mm = 120; v.swing_mm = 30;
    navigation_auto_reset(&s);
    s.attempts = 4;
    p = navigation_auto_step(&s,&v,config,0,1);
    assert(s.phase==NAV_WIGGLE && !strcmp(s.trigger,"tight_spot") && p.tight);
    bool back = false, ahead = false, spun = false;
    for (unsigned t = 20; t < 3000; t += 20) {
        p = navigation_auto_step(&s,&v,config,t,t/160+1);
        assert(s.phase==NAV_WIGGLE && p.tight);
        assert(!p.drive || p.drive == 85 || p.drive == -85);
        assert(!p.turn || p.turn == 80 || p.turn == -80);
        assert(!(p.drive && p.turn));
        back |= p.drive < 0; ahead |= p.drive > 0; spun |= p.turn != 0;
    }
    assert(back && ahead && spun);
    /* A pulse stops at once when its tight clearance closes. */
    unsigned t = 3000;
    while (!(p.drive < 0)) {
        p = navigation_auto_step(&s,&v,config,t,t/160+1);
        t += 20;
        assert(t < 6000);
    }
    v.tight_rear_mm = 30;
    p = navigation_auto_step(&s,&v,config,t,t/160+1);
    assert(!p.drive && !p.turn && s.phase==NAV_WIGGLE);
    /* Room ahead again: back to cruising. */
    v = open_view(1000);
    bool cruising = false;
    for (unsigned end = t + 1000; t < end && !cruising; t += 20) {
        p = navigation_auto_step(&s,&v,config,t,t/160+1);
        cruising = p.drive == 90 && !p.tight;
    }
    assert(cruising && s.phase==NAV_CRUISE);
    /* Nothing feasible at all: wiggling ends in BLOCKED. */
    navigation_auto_reset(&s);
    v = open_view(190); v.rear_free_mm = 190; v.tight_front_mm = 10; v.tight_rear_mm = 10; v.swing_mm = 5;
    s.attempts = 4;
    (void)navigation_auto_step(&s,&v,config,0,1);
    p = navigation_auto_step(&s,&v,config,200,2);
    assert(s.phase==NAV_BLOCKED && !p.drive && !p.turn);
}

static void stuck_tests(void)
{
    navigation_auto_t s = {0};
    navigation_plan_t p;
    navigation_view_t v;
    /* Without a pose: distances unchanged while driving -> escape. */
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
    /* With a SLAM pose the pose decides, even if distances flicker. */
    navigation_auto_reset(&s);
    for (unsigned t = 0; t < 4500; t += 20) {
        v = open_view(1000); v.front_mm = 1000 + (t % 400); pose(&v, 0, 0, 0);
        p = navigation_auto_step(&s,&v,config,t,t/160+1);
        assert(p.drive==90);
    }
    v = open_view(1000); pose(&v, 0, 0, 0);
    p = navigation_auto_step(&s,&v,config,4500,100);
    assert(!strcmp(s.trigger,"stuck_no_progress"));
    navigation_auto_reset(&s);
    for (unsigned t = 0; t < 8000; t += 20) {
        v = open_view(1000); pose(&v, t * 0.3f, 0, 0);
        p = navigation_auto_step(&s,&v,config,t,t/160+1);
        assert(p.drive==90);
    }
    navigation_auto_reset(&s);
    assert(s.phase==NAV_CRUISE && !s.attempts && !s.turn_direction && !strcmp(s.trigger,"none"));
}

static void wall_tests(void)
{
    navigation_auto_t s;
    navigation_plan_t p;
    navigation_view_t v;
    navigation_auto_configure(&s, NAV_STRATEGY_WALL_RIGHT, 0, 0);
    v = open_view(2000);
    v.wall_right = (navigation_wall_t){true, 350, 0};
    p = navigation_auto_step(&s,&v,config,0,1);
    assert(p.drive==90 && !p.turn && !strcmp(p.reason,"wall_follow"));
    v.wall_right.distance_mm = 500;               /* too far: steer to the wall */
    p = navigation_auto_step(&s,&v,config,100,2);
    assert(p.turn==65 && p.drive > 85);
    v.wall_right.distance_mm = 200;               /* too close: steer away */
    p = navigation_auto_step(&s,&v,config,200,3);
    assert(p.turn==-65);
    v.wall_right = (navigation_wall_t){true, 350, 20};   /* wall diverges */
    p = navigation_auto_step(&s,&v,config,300,4);
    assert(p.turn > 50);
    v.wall_right = (navigation_wall_t){true, 350, -20};  /* wall converges */
    p = navigation_auto_step(&s,&v,config,400,5);
    assert(p.turn < -50);

    /* Outer corner: arc around it, then search after the timeout. */
    v.wall_right.valid = false;
    p = navigation_auto_step(&s,&v,config,500,6);
    assert(p.turn==88 && p.drive==85 && !strcmp(p.reason,"wall_corner"));
    p = navigation_auto_step(&s,&v,config,4500,7);
    assert(!p.turn && p.drive==90 && !strcmp(p.reason,"wall_search"));

    /* Inner corner: rotate away from the wall until there is room. */
    v = open_view(450); v.wall_right = (navigation_wall_t){true, 350, 0};
    p = navigation_auto_step(&s,&v,config,5000,8);
    assert(p.turn==-100 && !p.drive && s.phase==NAV_TURN && !strcmp(s.trigger,"wall_inner_corner"));
    v = open_view(450);
    p = navigation_auto_step(&s,&v,config,5300,9);
    assert(p.turn==-100 && s.phase==NAV_TURN);    /* corner not yet cleared */
    v = open_view(1500); v.wall_right = (navigation_wall_t){true, 350, 0};
    p = navigation_auto_step(&s,&v,config,5400,10);
    assert(p.drive==90 && !p.turn && s.phase==NAV_CRUISE);

    /* Left wall mirrors the sign. */
    navigation_auto_configure(&s, NAV_STRATEGY_WALL_LEFT, 0, 0);
    v = open_view(2000); v.wall_left = (navigation_wall_t){true, 500, 0};
    p = navigation_auto_step(&s,&v,config,0,1);
    assert(p.turn==-65);
    v = open_view(450); v.wall_left = (navigation_wall_t){true, 350, 0};
    p = navigation_auto_step(&s,&v,config,100,2);
    assert(p.turn==100 && !strcmp(s.trigger,"wall_inner_corner"));
}

static void goal_tests(void)
{
    navigation_auto_t s;
    navigation_plan_t p;
    navigation_view_t v;
    /* No pose: wait without counting as stuck. */
    navigation_auto_configure(&s, NAV_STRATEGY_GOAL, 2000, 0);
    v = open_view(2000);
    for (unsigned t = 0; t < 6000; t += 20) {
        p = navigation_auto_step(&s,&v,config,t,t/160+1);
        assert(!p.drive && !p.turn && !strcmp(p.reason,"goal_localization_wait"));
    }
    /* Seek straight, rotate toward a goal behind, stop inside the radius. */
    navigation_auto_configure(&s, NAV_STRATEGY_GOAL, 2000, 0);
    v = open_view(2000); pose(&v, 0, 0, 0);
    p = navigation_auto_step(&s,&v,config,0,1);
    assert(p.drive==90 && !p.turn && !strcmp(p.reason,"goal_seek"));
    pose(&v, 0, 0, 3.14159f / 2);                 /* goal now 90 deg right */
    p = navigation_auto_step(&s,&v,config,100,2);
    assert(p.turn==100 && !p.drive && !strcmp(p.reason,"goal_turn"));
    pose(&v, 0, 0, 0.5f);                         /* ~29 deg right: steer */
    p = navigation_auto_step(&s,&v,config,200,3);
    assert(p.turn > 50 && p.turn < 100 && p.drive > 85);
    pose(&v, 1850, 50, 0);
    p = navigation_auto_step(&s,&v,config,300,4);
    assert(p.done && !p.drive && !p.turn && !strcmp(p.reason,"goal_reached"));

    /* Bug2: M-line blocked -> hit point, turn to the open side, follow the
       boundary with the obstacle on the other side, leave on the M-line. */
    navigation_auto_configure(&s, NAV_STRATEGY_GOAL, 3000, 0);
    v = open_view(2000);
    for (int k = 0; k < NAV_CENTER; k++) v.free_mm[k] = 600;
    v.free_mm[NAV_CENTER] = 300;
    pose(&v, 0, 0, 0);
    p = navigation_auto_step(&s,&v,config,0,1);
    assert(s.goal_phase==NAV_GOAL_BOUNDARY && s.wall_side==-1 && s.hit_distance > 2990);
    assert(p.turn==100 && s.phase==NAV_TURN && !strcmp(s.trigger,"goal_hit"));
    v = open_view(1500); v.wall_left = (navigation_wall_t){true, 350, 0}; pose(&v, 0, 0, -0.8f);
    p = navigation_auto_step(&s,&v,config,300,2);
    assert(s.phase==NAV_CRUISE && p.drive==90 && !p.turn && !strcmp(p.reason,"wall_follow"));
    assert(!strcmp(s.detail,"goal_boundary"));
    pose(&v, 500, -300, 0.3f);
    p = navigation_auto_step(&s,&v,config,400,3);
    assert(s.goal_phase==NAV_GOAL_BOUNDARY);      /* off the M-line */
    v = open_view(2000); pose(&v, 1200, 50, 0);
    p = navigation_auto_step(&s,&v,config,500,4);
    assert(s.goal_phase==NAV_GOAL_SEEK && !strcmp(s.trigger,"goal_leave"));
    assert(p.drive==90 && !strcmp(p.reason,"goal_seek"));

    /* Boundary loop back to the hit point: goal unreachable. */
    navigation_auto_configure(&s, NAV_STRATEGY_GOAL, 3000, 0);
    v = open_view(2000);
    for (int k = 0; k < NAV_CENTER; k++) v.free_mm[k] = 600;
    v.free_mm[NAV_CENTER] = 300;
    pose(&v, 0, 0, 0);
    (void)navigation_auto_step(&s,&v,config,0,1);
    v = open_view(1500); pose(&v, 0, 0, -0.8f);
    (void)navigation_auto_step(&s,&v,config,300,2);
    pose(&v, 600, 600, 2.0f);
    p = navigation_auto_step(&s,&v,config,400,3);
    assert(!p.done);
    pose(&v, -600, 600, 3.0f);
    p = navigation_auto_step(&s,&v,config,500,4);
    assert(!p.done);
    pose(&v, 0, 100, -1.5f);
    p = navigation_auto_step(&s,&v,config,600,5);
    assert(p.done && !strcmp(p.reason,"goal_unreachable"));
    assert(s.strategy==NAV_STRATEGY_GOAL && s.goal_x==3000);
    assert(!strcmp(navigation_strategy_name(NAV_STRATEGY_WALL_RIGHT),"wall_right"));
}

static void route(navigation_view_t *v, float way_x, float way_y, float goal_x, float goal_y)
{
    v->route_valid = true;
    v->route_way_x = way_x; v->route_way_y = way_y;
    v->route_goal_x = goal_x; v->route_goal_y = goal_y;
}

static void path_tests(void)
{
    navigation_auto_t s;
    navigation_plan_t p;
    navigation_view_t v;
    /* No pose: wait. Pose but no path yet: plan, then give up after 3 s. */
    navigation_auto_configure(&s, NAV_STRATEGY_PATH, 3000, 0);
    v = open_view(2000);
    p = navigation_auto_step(&s,&v,config,0,1);
    assert(!p.drive && !p.turn && !strcmp(p.reason,"path_localization_wait"));
    pose(&v, 0, 0, 0);
    p = navigation_auto_step(&s,&v,config,100,2);
    assert(!p.drive && !p.turn && !p.done && !strcmp(p.reason,"path_planning"));
    route(&v, 1000, 0, 9000, 9000);               /* stale path to another goal */
    p = navigation_auto_step(&s,&v,config,3000,3);
    assert(!p.done && !strcmp(p.reason,"path_planning"));
    p = navigation_auto_step(&s,&v,config,3100,4);
    assert(p.done && !strcmp(p.reason,"path_unreachable"));

    /* Waypoint straight ahead: drive at it. */
    navigation_auto_configure(&s, NAV_STRATEGY_PATH, 3000, 0);
    v = open_view(2000); pose(&v, 0, 0, 0); route(&v, 1500, 0, 3000, 0);
    p = navigation_auto_step(&s,&v,config,0,1);
    assert(p.drive==90 && !p.turn && !strcmp(p.reason,"path_follow"));
    /* Waypoint 30 deg left (detour around a wall): steer left, even though
       the best free heading points elsewhere. */
    v.free_mm[NAV_CENTER+5] = 4000;
    route(&v, 1000, 577, 3000, 0);
    p = navigation_auto_step(&s,&v,config,100,2);
    assert(p.turn < -50 && p.drive > 0 && !strcmp(p.reason,"path_follow"));
    /* 45 deg: rotate in place first, slower near the target heading, and
       keep rotating until aligned. */
    route(&v, 1000, 1000, 3000, 0);
    p = navigation_auto_step(&s,&v,config,150,3);
    assert(!p.drive && p.turn==-88 && !strcmp(p.reason,"path_turn"));
    pose(&v, 0, 0, 0.5f);                         /* 16 deg still to go */
    p = navigation_auto_step(&s,&v,config,160,4);
    assert(!p.drive && p.turn==-64 && !strcmp(p.reason,"path_turn"));
    pose(&v, 0, 0, 0.72f);                        /* aligned: drive */
    p = navigation_auto_step(&s,&v,config,170,5);
    assert(p.drive==90 && !p.turn && !strcmp(p.reason,"path_follow"));
    pose(&v, 0, 0, 0);
    v.free_mm[NAV_CENTER+5] = 2000;
    /* Waypoint behind: rotate in place toward it. */
    route(&v, -1000, -200, 3000, 0);
    p = navigation_auto_step(&s,&v,config,200,3);
    assert(!p.drive && p.turn==100 && !strcmp(p.reason,"path_turn"));
    /* Corridor to the waypoint blocked: avoid via the best heading. */
    v = open_view(2000); pose(&v, 0, 0, 0); route(&v, 1500, 0, 3000, 0);
    v.free_mm[NAV_CENTER] = 400;
    p = navigation_auto_step(&s,&v,config,300,4);
    assert(p.drive > 0 && !strcmp(p.reason,"path_avoid"));
    /* Front blocked: turn toward the waypoint side. */
    v = open_view(190); pose(&v, 0, 0, 0.3f); route(&v, 1500, -400, 3000, 0);
    for (int k = 0; k < NAV_CENTER - 2; k++) v.free_mm[k] = 2000;   /* gap on the left */
    p = navigation_auto_step(&s,&v,config,400,5);
    assert(p.turn > 0 && s.phase==NAV_TURN);
    /* Planned goal moved off a wall: reaching it counts. */
    navigation_auto_configure(&s, NAV_STRATEGY_PATH, 3000, 0);
    v = open_view(2000); pose(&v, 2800, 200, 0); route(&v, 2800, 250, 2800, 250);
    p = navigation_auto_step(&s,&v,config,0,1);
    assert(p.done && !strcmp(p.reason,"goal_reached"));
    assert(!strcmp(navigation_strategy_name(NAV_STRATEGY_PATH),"path"));
}

/* Exploration with a map target: rotate toward it, then drive straight;
   small heading errors are held, larger ones corrected to near zero. */
static void map_route_tests(void)
{
    navigation_auto_t s;
    navigation_plan_t p;
    navigation_view_t v;
    navigation_auto_configure(&s, NAV_STRATEGY_EXPLORE, 0, 0);
    v = open_view(2000); pose(&v, 0, 0, 0); route(&v, 0, 1500, 3000, 3000);
    p = navigation_auto_step(&s,&v,config,0,1);
    assert(!p.drive && p.turn==-100 && !strcmp(p.reason,"explore_turn"));
    pose(&v, 0, 0, 1.30f);                        /* 15 deg left: keep turning */
    p = navigation_auto_step(&s,&v,config,100,2);
    assert(!p.drive && p.turn < 0 && !strcmp(p.reason,"explore_turn"));
    pose(&v, 0, 0, 1.50f);                        /* aligned */
    p = navigation_auto_step(&s,&v,config,200,3);
    assert(p.drive==90 && !p.turn && !strcmp(p.reason,"explore_follow"));
    pose(&v, 0, 0, 1.45f);                        /* 7 deg: hold course */
    p = navigation_auto_step(&s,&v,config,300,4);
    assert(p.drive==90 && !p.turn);
    pose(&v, 0, 0, 1.38f);                        /* 11 deg: correct */
    p = navigation_auto_step(&s,&v,config,400,5);
    assert(p.drive > 85 && p.turn < -50 && !strcmp(p.reason,"explore_follow"));
    pose(&v, 0, 0, 1.50f);                        /* 4 deg: finish correcting */
    p = navigation_auto_step(&s,&v,config,500,6);
    assert(p.turn < 0);
    pose(&v, 0, 0, 1.56f);
    p = navigation_auto_step(&s,&v,config,600,7);
    assert(p.drive==90 && !p.turn);
    /* Target corridor blocked: avoid via the best heading. */
    v.free_mm[NAV_CENTER] = 400;
    p = navigation_auto_step(&s,&v,config,700,8);
    assert(p.drive > 0 && !strcmp(p.reason,"explore_avoid"));
    /* No route: reactive exploration. */
    v = open_view(2000); pose(&v, 0, 0, 0);
    p = navigation_auto_step(&s,&v,config,800,9);
    assert(p.drive==90 && !strcmp(p.reason,"cruise"));
}

/* Weak SLAM pauses map strategies for a while, then they carry on. */
static void slam_wait_tests(void)
{
    navigation_auto_t s;
    navigation_plan_t p;
    navigation_view_t v;
    navigation_auto_configure(&s, NAV_STRATEGY_EXPLORE, 0, 0);
    v = open_view(2000); v.map_weak = true;
    p = navigation_auto_step(&s,&v,config,0,1);
    assert(!p.drive && !p.turn && !strcmp(p.reason,"slam_wait"));
    p = navigation_auto_step(&s,&v,config,2900,20);
    assert(!p.drive && !strcmp(p.reason,"slam_wait"));
    p = navigation_auto_step(&s,&v,config,3000,21);
    assert(p.drive==90);                          /* gave up waiting */
    p = navigation_auto_step(&s,&v,config,3100,22);
    assert(p.drive==90);
    v.front_mm = 1500;                            /* progress, not stuck */
    p = navigation_auto_step(&s,&v,config,9100,60);
    assert(!p.drive && !strcmp(p.reason,"slam_wait"));
    v.map_weak = false;
    p = navigation_auto_step(&s,&v,config,9200,61);
    assert(p.drive==90);
    v.map_lost = true;                            /* lost counts as weak */
    p = navigation_auto_step(&s,&v,config,9300,62);
    assert(!strcmp(p.reason,"slam_wait"));
    /* Reactive strategies ignore the map. */
    navigation_auto_configure(&s, NAV_STRATEGY_WALL_RIGHT, 0, 0);
    v = open_view(2000); v.map_weak = true; v.wall_right = (navigation_wall_t){true, 350, 0};
    p = navigation_auto_step(&s,&v,config,0,1);
    assert(p.drive==90 && !strcmp(p.reason,"wall_follow"));
}

/* Stop threshold hysteresis: cruising keeps going down to auto_stop,
   starting needs auto_stop + resume_margin. */
static void hysteresis_tests(void)
{
    navigation_auto_t s = {0};
    navigation_plan_t p;
    navigation_view_t v;
    navigation_auto_reset(&s);
    v = open_view(2000);
    p = navigation_auto_step(&s,&v,config,0,1);
    assert(p.drive==90);
    v = open_view(220);                           /* between stop and resume */
    p = navigation_auto_step(&s,&v,config,100,2);
    assert(p.drive==85 && s.phase==NAV_CRUISE);
    v = open_view(190);
    p = navigation_auto_step(&s,&v,config,200,3);
    assert(!p.drive && s.phase==NAV_BRAKE);
    /* From standstill 220 mm is not enough to start. */
    navigation_auto_reset(&s);
    v = open_view(220);
    p = navigation_auto_step(&s,&v,config,0,1);
    assert(!p.drive && s.phase==NAV_BRAKE);
}

int main(void)
{
    explore_tests();
    wiggle_tests();
    stuck_tests();
    wall_tests();
    goal_tests();
    path_tests();
    map_route_tests();
    slam_wait_tests();
    hysteresis_tests();
    puts("Planner tests: smooth steering, fast turns, escapes, stuck detection, wall following, Bug2, map path, explore route, SLAM wait, hysteresis passed.");
    return 0;
}
