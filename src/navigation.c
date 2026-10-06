#include "navigation.h"
#include <limits.h>

static uint16_t sector(const uint16_t d[360], int front, bool clockwise,
                       int center, int half_width, bool *known)
{
    uint16_t min = UINT16_MAX;
    unsigned valid = 0, missing_run = 0, longest_gap = 0;
    unsigned total = 2 * half_width + 1;
    for (int a = center - half_width; a <= center + half_width; a++) {
        int index = (front + (clockwise ? a : -a) + 720) % 360;
        if (d[index]) {
            valid++;
            missing_run = 0;
            if (d[index] < min) min = d[index];
        } else {
            missing_run++;
            if (missing_run > longest_gap) longest_gap = missing_run;
        }
    }
    /* Unknown returns are NOT free space. Allow only tiny sampling gaps. */
    *known = valid * 100 >= total * 85 && longest_gap <= 3;
    return min == UINT16_MAX ? 0 : min;
}

navigation_view_t navigation_analyze(const uint16_t d[360], int front, bool clockwise)
{
    navigation_view_t v = {0};
    bool left, right;
    v.front_mm = sector(d, front, clockwise, 0, 35, &v.front_known);
    v.rear_mm = sector(d, front, clockwise, 180, 35, &v.rear_known);
    v.left_mm = sector(d, front, clockwise, -90, 35, &left);
    v.right_mm = sector(d, front, clockwise, 90, 35, &right);
    v.nearest_mm = sector(d, front, clockwise, 180, 179, &v.rotation_known);
    if (d[front % 360] && (!v.nearest_mm || d[front % 360] < v.nearest_mm))
        v.nearest_mm = d[front % 360];
    /* The full circle includes the omitted forward bin too. */
    v.rotation_known = v.rotation_known && d[front % 360] != 0;
    (void)left;
    (void)right;
    return v;
}

navigation_plan_t navigation_plan(navigation_view_t v, int preferred_turn,
    unsigned stop, unsigned rotate, unsigned slow, int drive_limit, int turn_limit)
{
    navigation_plan_t p = { .reason = "unknown_space" };
    if (!v.front_known) return p;
    if (v.front_mm > stop + 150) {
        p.drive = drive_limit;
        if (v.front_mm < slow) p.drive = drive_limit * 2 / 3;
        p.reason = "cruise";
    } else if (v.rotation_known && v.nearest_mm > rotate) {
        p.turn = (preferred_turn ? preferred_turn : (v.left_mm >= v.right_mm ? -1 : 1)) * turn_limit;
        p.reason = "avoid";
    } else {
        p.reason = "blocked";
    }
    return p;
}