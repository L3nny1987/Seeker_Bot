#include "../src/navigation.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    uint16_t d[360] = {0};
    navigation_view_t v = navigation_analyze(d, 0, true);
    assert(!v.front_known && !v.rotation_known);
    navigation_plan_t p = navigation_plan(v, 0, 500, 550, 1200, 30, 30);
    assert(p.drive == 0 && p.turn == 0);
    for (int i = 0; i < 360; i++) d[i] = 2000;
    v = navigation_analyze(d, 0, true);
    assert(v.front_known && v.rotation_known && v.front_mm == 2000);
    p = navigation_plan(v, 0, 500, 550, 1200, 30, 30);
    assert(p.drive == 30 && p.turn == 0);
    d[0] = 900;
    v = navigation_analyze(d, 0, true);
    p = navigation_plan(v, 0, 500, 550, 1200, 30, 30);
    assert(p.drive == 20 && p.turn == 0);
    d[0] = 600;
    v = navigation_analyze(d, 0, true);
    p = navigation_plan(v, 1, 500, 550, 1200, 30, 30);
    assert(p.drive == 0 && p.turn == 30);
    d[90] = 300;
    v = navigation_analyze(d, 0, true);
    p = navigation_plan(v, 0, 500, 550, 1200, 30, 30);
    assert(p.drive == 0 && p.turn == 0);
    for (int i = 0; i < 360; i++) d[i] = 2000;
    for (int i = 10; i < 15; i++) d[i] = 0;
    v = navigation_analyze(d, 0, true);
    assert(!v.front_known);
    p = navigation_plan(v, 0, 500, 550, 1200, 30, 30);
    assert(!p.drive && !p.turn);
    for (int i = 0; i < 360; i++) d[i] = 2000;
    d[90] = 700;
    v = navigation_analyze(d, 90, false);
    assert(v.front_mm == 700 && v.front_known);
    d[0] = 400;
    v = navigation_analyze(d, 90, false);
    assert(v.right_mm == 400 && v.nearest_mm == 400);
    puts("Navigation tests passed: unknown space, cruise, slowdown, avoidance, blockage, gaps, orientation.");
    return 0;
}