#include "../src/navigation.h"
#include <assert.h>
#include <stdio.h>

static uint16_t d[360];
static void fill(uint16_t value) { for (int i = 0; i < 360; i++) d[i] = value; }
static int raw(int robot_angle) { return (robot_angle + 360) % 360; }   /* front 0, clockwise */

int main(void)
{
    fill(0);
    navigation_view_t v = navigation_analyze(d, 0, true, 250);
    assert(!v.front_known && !v.rotation_known);
    /* Nothing measured is never free: corridor capped to the short horizon. */
    assert(v.free_mm[NAV_CENTER] == NAV_UNCERTAIN_MM && (v.uncertain_mask & (1u << NAV_CENTER)));
    assert(v.rear_free_mm == NAV_UNCERTAIN_MM && v.rear_uncertain);

    fill(2000);
    v = navigation_analyze(d, 0, true, 250);
    assert(v.front_known && v.rear_known && v.rotation_known && v.front_mm == 2000);
    assert(v.free_mm[NAV_CENTER] >= 1990 && v.free_mm[NAV_CENTER] <= 2000);
    assert(v.rear_free_mm >= 1990 && !v.uncertain_mask && !v.rear_uncertain);

    /* Typical LD20 dropouts no longer make the sector unknown ... */
    for (int a = 10; a < 15; a++) d[raw(a)] = 0;
    v = navigation_analyze(d, 0, true, 250);
    assert(v.front_known && v.rotation_known);
    /* ... but a large hole does. */
    for (int a = 10; a < 25; a++) d[raw(a)] = 0;
    v = navigation_analyze(d, 0, true, 250);
    assert(!v.front_known && v.rotation_known);
    fill(2000);
    for (int a = 100; a < 125; a++) d[raw(a)] = 0;
    v = navigation_analyze(d, 0, true, 250);
    assert(v.rotation_known);                 /* 25 deg gap tolerated */
    for (int a = 100; a < 140; a++) d[raw(a)] = 0;
    v = navigation_analyze(d, 0, true, 250);
    assert(!v.rotation_known);                /* 40 deg gap is not */

    /* Corridor: wall segment straight ahead, open space to the right. */
    fill(2000);
    for (int a = -30; a <= 15; a++) d[raw(a)] = 500;
    v = navigation_analyze(d, 0, true, 250);
    assert(v.free_mm[NAV_CENTER] <= 500);
    assert(v.free_mm[NAV_CENTER + 4] > 1000);  /* +40 deg passes the wall */
    assert(v.front_mm == 500 && v.nearest_mm == 500);

    /* Narrow gap: a robot-wide corridor does not fit between two posts. */
    fill(3000);
    d[raw(-4)] = 1000; d[raw(4)] = 1000;      /* ~140 mm apart at 1 m */
    v = navigation_analyze(d, 0, true, 250);
    assert(v.free_mm[NAV_CENTER] <= 1000);
    v = navigation_analyze(d, 0, true, 100);
    assert(v.free_mm[NAV_CENTER] >= 2900);    /* a 100 mm robot fits */

    /* Counter-clockwise mounting and a different zero angle. */
    fill(2000);
    d[90] = 700;
    v = navigation_analyze(d, 90, false, 250);
    assert(v.front_mm == 700 && v.front_known);
    d[0] = 400;
    v = navigation_analyze(d, 90, false, 250);
    assert(v.right_mm == 400 && v.nearest_mm == 400);

    /* Physical mounting: raw 180 is forward, clockwise remains clockwise. */
    fill(2000);
    d[180] = 600; d[0] = 1300; d[90] = 1500; d[270] = 900;
    v = navigation_analyze(d, 180, true, 250);
    assert(v.front_mm == 600 && v.rear_mm == 1300);
    assert(v.left_mm == 1500 && v.right_mm == 900);

    /* Fusion fills dropouts from the previous scan but keeps measurements. */
    uint16_t current[360] = {0}, previous[360], fused[360];
    for (int i = 0; i < 360; i++) previous[i] = 1500;
    current[5] = 300;
    navigation_fuse(current, previous, fused);
    assert(fused[5] == 300 && fused[6] == 1500);
    navigation_fuse(current, NULL, fused);
    assert(fused[5] == 300 && fused[6] == 0);
    puts("Navigation tests passed: gaps, corridors, robot width, orientation, fusion.");
    return 0;
}