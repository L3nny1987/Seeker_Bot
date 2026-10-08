#include "../src/slam.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define PI 3.14159265358979

static slam_t slam;

/* Room 4.0 x 3.0 m with a box obstacle; ray cast against axis-aligned boxes. */
typedef struct { double x0, y0, x1, y1; bool inside; } box_t;
static const box_t room[] = {
    {-1500, -1500, 2500, 1500, true},
    {800, 400, 1200, 900, false},
    {-900, -1200, -500, -700, false},
};
/* Large hall 16 x 11 m: start 5 m from the back wall, far wall 11 m ahead,
   a few pillars and a cabinet. */
static const box_t hall[] = {
    {-5000, -5500, 11000, 5500, true},
    {1800, 1800, 2200, 2200, false},
    {1800, -2200, 2200, -1800, false},
    {6000, 1800, 6400, 2200, false},
    {6000, -2200, 6400, -1800, false},
    {-3000, 4800, -1000, 5500, false},
};
static const box_t *boxes = room;
static unsigned box_count = sizeof(room) / sizeof(room[0]);
static double max_range = 8000, far_dropout = 0;

static double cast(double x, double y, double dir)
{
    double c = cos(dir), s = sin(dir), best = 1e9;
    for (unsigned b = 0; b < box_count; b++) {
        const box_t *q = &boxes[b];
        double xs[2] = {q->x0, q->x1}, ys[2] = {q->y0, q->y1};
        for (int i = 0; i < 2; i++) {
            if (fabs(c) > 1e-9) {
                double t = (xs[i] - x) / c, py = y + t * s;
                if (t > 1 && py >= q->y0 && py <= q->y1 && t < best) best = t;
            }
            if (fabs(s) > 1e-9) {
                double t = (ys[i] - y) / s, px = x + t * c;
                if (t > 1 && px >= q->x0 && px <= q->x1 && t < best) best = t;
            }
        }
    }
    return best;
}

/* Raw LD20 scan (front = raw 0, clockwise) ending at pose (x, y, th) while
   moving with (vx, vy, w): earlier bins are taken at earlier poses. */
static void scan(uint16_t raw[360], double x, double y, double th, double vx, double vy, double w)
{
    for (int i = 0; i < 360; i++) {
        double age = 0.1 * (359.5 - i) / 360.0;
        double px = x - vx * age, py = y - vy * age, pt = th - w * age;
        double d = cast(px, py, pt - i * PI / 180.0);
        d += ((rand() % 21) - 10);   /* +-10 mm noise */
        /* Far returns are weak: some drop out beyond 6 m. */
        bool weak = d > 6000 && rand() % 1000 < far_dropout * 1000;
        raw[i] = d > 80 && d < max_range && (i % 17) && !weak ? (uint16_t)d : 0;
    }
}

static double angle_error(double a, double b)
{
    return fabs(remainder(a - b, 2 * PI));
}

/* Occupied map cells more than 1.5 cells away from any true wall. */
static int smeared_cells(const slam_t *g)
{
    int bad = 0;
    for (int gy = 0; gy < SLAM_SIZE; gy++)
        for (int gx = 0; gx < SLAM_SIZE; gx++) {
            if (slam_cell_class(g->grid[gy * SLAM_SIZE + gx]) != 3) continue;
            double wx = (gx - SLAM_ORIGIN) * SLAM_RES_MM, wy = (gy - SLAM_ORIGIN) * SLAM_RES_MM, best = 1e9;
            for (unsigned b = 0; b < box_count; b++) {
                const box_t *q = &boxes[b];
                double cx = fmin(fmax(wx, q->x0), q->x1), cy = fmin(fmax(wy, q->y0), q->y1);
                double d = hypot(wx - cx, wy - cy);
                if (!d) d = fmin(fmin(wx - q->x0, q->x1 - wx), fmin(wy - q->y0, q->y1 - wy));
                if (d < best) best = d;
            }
            if (best > 1.5 * SLAM_RES_MM) bad++;
        }
    return bad;
}

/* Turning on the spot at 85 deg/s (real robot, full turn PWM) with the gyro
   40 ms out of sync with the LiDAR and 5 % scale error: walls must not be
   smeared into rings and the heading must stay locked to the map. */
static void latency_spin_test(void)
{
    const double w = 1.48, lag = 0.04, scale = 1.05;
    slam_t *g = calloc(1, sizeof(slam_t));
    uint16_t raw[360];
    slam_reset(g);
    double th = 0, yaw = 0, last_yaw = 0, max_err = 0;
    uint64_t t = 1000;
    for (int i = 0; i < 160; i++) {
        double ww = i >= 10 && i % 32 < 20 ? (i / 32 % 2 ? -w : w) : 0;
        th += ww * 0.1;
        yaw += ww * 0.1 * scale;
        double yaw_at = yaw - ww * scale * lag;
        slam_imu_t imu = {i > 0, (float)(yaw_at - last_yaw), (float)(ww * scale)};
        last_yaw = yaw_at;
        scan(raw, 0, 0, th, 0, 0, ww);
        slam_update(g, raw, 0, true, 100, t, ww != 0, &imu);
        t += 100;
        double e = angle_error(g->pose.theta, th);
        if (e > max_err) max_err = e;
    }
    int smeared = smeared_cells(g);
    printf("latency spin: max heading err %.1f deg, smeared cells %d, lost %d\n", max_err * 180 / PI, smeared, g->lost);
    assert(max_err < 0.05 && smeared == 0 && !g->lost);
    free(g);
}

/* Large hall with returns up to 12 m (half of them lost beyond 6 m): drive
   5.5 m ahead, turn, cross the hall, come back. Must never get lost. */
static void hall_test(void)
{
    const double v = 300, w = 1.48, scale = 1.02, bias = 0.004;
    static const struct { double drive_mm, turn_rad; } legs[] = {
        {5500, PI / 2}, {3000, PI / 2}, {8000, PI / 2}, {5000, PI / 2}, {4000, 0}};
    slam_t *g = calloc(1, sizeof(slam_t));
    uint16_t raw[360];
    boxes = hall; box_count = sizeof(hall) / sizeof(hall[0]);
    max_range = 12000; far_dropout = 0.5;
    slam_reset(g);
    double x = 0, y = 0, th = 0, yaw = 0, last_yaw = 0, max_err = 0, max_head = 0;
    uint64_t t = 1000;
    int lost_scans = 0, scans = 0;
    for (unsigned leg = 0; leg < sizeof(legs) / sizeof(legs[0]); leg++) {
        int drive_steps = (int)(legs[leg].drive_mm / (v * 0.1)), turn_steps = (int)(legs[leg].turn_rad / (w * 0.1));
        for (int i = -10; i < drive_steps + turn_steps; i++) {
            double vv = i >= 0 && i < drive_steps ? v : 0, ww = i >= drive_steps ? w : 0;
            th += ww * 0.1;
            x += vv * cos(th) * 0.1; y += vv * sin(th) * 0.1;
            yaw += (ww * scale + bias) * 0.1;
            slam_imu_t imu = {scans > 0, (float)(yaw - last_yaw), (float)(ww * scale + bias)};
            last_yaw = yaw;
            scan(raw, x, y, th, vv * cos(th), vv * sin(th), ww);
            slam_update(g, raw, 0, true, 100, t, vv || ww, &imu);
            t += 100; scans++;
            lost_scans += g->lost;
            double e = hypot(g->pose.x - x, g->pose.y - y), h = angle_error(g->pose.theta, th);
            if (e > max_err) max_err = e;
            if (h > max_head) max_head = h;
        }
    }
    printf("hall: scans %d lost %d max err %.0f mm %.1f deg integrated %u q %.2f end %.0f %.0f\n",
           scans, lost_scans, max_err, max_head * 180 / PI, g->integrated, g->quality, x, y);
    assert(lost_scans == 0 && max_err < 200 && max_head < 0.05);
    boxes = room; box_count = sizeof(room) / sizeof(room[0]);
    max_range = 8000; far_dropout = 0;
    free(g);
}

/* Planning-cell indices below are relative to a 192-cell grid (origin at
   planning cell 24); PO shifts them to the actual grid size. */
#define PO (SLAM_PLAN_SIZE / 2 - 24)

/* Fill one 20 cm planning cell (4x4 map cells). */
static void plan_cell(slam_t *g, int cx, int cy, int8_t value)
{
    cx += PO; cy += PO;
    for (int j = 0; j < SLAM_PLAN_CELLS; j++)
        for (int i = 0; i < SLAM_PLAN_CELLS; i++)
            g->grid[(cy * SLAM_PLAN_CELLS + j) * SLAM_SIZE + cx * SLAM_PLAN_CELLS + i] = value;
}

/* Room of planning cells 19..28 with walls around and a door at (29, 24). */
static void plan_room(slam_t *g)
{
    slam_reset(g);
    for (int cy = 18; cy <= 29; cy++)
        for (int cx = 18; cx <= 29; cx++) {
            bool wall = cx == 18 || cx == 29 || cy == 18 || cy == 29;
            plan_cell(g, cx, cy, wall ? 40 : -20);
        }
    plan_cell(g, 29, 24, 0);
}

static void explore_target_tests(void)
{
    slam_t *g = calloc(1, sizeof(slam_t));
    slam_target_t target;
    float door_x = ((28 + PO) * SLAM_PLAN_CELLS + 1.5f - SLAM_ORIGIN) * SLAM_RES_MM;
    float door_y = ((24 + PO) * SLAM_PLAN_CELLS + 1.5f - SLAM_ORIGIN) * SLAM_RES_MM;
    /* Frontier at the door, path length 4 planning cells. */
    plan_room(g);
    assert(slam_explore_target(g, (slam_pose_t){100, 100, 0}, NULL, &target));
    assert(target.valid && target.frontier && target.steps == 4);
    assert(fabsf(target.x - door_x) < 1 && fabsf(target.y - door_y) < 1);
    /* Inner wall at x = cell 26 with a gap at the top: path goes around. */
    for (int cy = 19; cy <= 27; cy++) plan_cell(g, 26, cy, 40);
    float wall_x = ((26 + PO) * SLAM_PLAN_CELLS - SLAM_ORIGIN) * SLAM_RES_MM;
    assert(slam_explore_target(g, (slam_pose_t){-700, 100, 0}, NULL, &target));
    printf("plan: target %.0f %.0f way %.0f %.0f steps %u\n",
           target.x, target.y, target.way_x, target.way_y, target.steps);
    assert(target.frontier && fabsf(target.x - door_x) < 1 && target.steps == 16);
    assert(target.way_x < wall_x);
    /* Door blacklisted: no frontier left, revisit coverage instead. */
    static uint8_t avoid[SLAM_PLAN_SIZE * SLAM_PLAN_SIZE];
    avoid[(24 + PO) * SLAM_PLAN_SIZE + 28 + PO] = 1;
    assert(slam_explore_target(g, (slam_pose_t){-700, 100, 0}, avoid, &target));
    assert(target.valid && !target.frontier && target.cell != (24 + PO) * SLAM_PLAN_SIZE + 28 + PO);
    /* Outside the map: no target. */
    assert(!slam_explore_target(g, (slam_pose_t){1e6f, 0, 0}, NULL, &target));
    free(g);
}

static float plan_center(int c) { return ((c + PO) * SLAM_PLAN_CELLS + 1.5f - SLAM_ORIGIN) * SLAM_RES_MM; }

static void plan_path_tests(void)
{
    slam_t *g = calloc(1, sizeof(slam_t));
    slam_target_t t;
    slam_pose_t start = {-700, 100, 0};
    /* Open room, goal in sight: the goal itself is the waypoint. */
    plan_room(g);
    assert(slam_plan_path(g, start, 500, 100, &t));
    assert(t.valid && !t.frontier && t.way_x == 500 && t.way_y == 100 && t.steps == 6);
    /* Inner wall with a gap at the top: waypoint stays on the robot's side
       and leads upwards around the wall. */
    for (int cy = 19; cy <= 27; cy++) plan_cell(g, 26, cy, 40);
    float wall_x = ((26 + PO) * SLAM_PLAN_CELLS - SLAM_ORIGIN) * SLAM_RES_MM;
    assert(slam_plan_path(g, start, plan_center(27), plan_center(24), &t));
    printf("path: way %.0f %.0f steps %u\n", t.way_x, t.way_y, t.steps);
    assert(t.valid && !t.frontier && t.steps == 15);
    assert(t.way_x < wall_x && t.way_y > 300);
    /* From the gap the goal is close: waypoint behind the wall end. */
    assert(slam_plan_path(g, (slam_pose_t){plan_center(25), plan_center(28), 0},
                          plan_center(27), plan_center(24), &t));
    assert(t.way_x > wall_x);
    /* Goal clicked into the wall: nearest open neighbour cell. */
    assert(slam_plan_path(g, start, plan_center(26), plan_center(22), &t));
    assert(t.valid && fabsf(t.x - plan_center(26)) > 100);
    /* Goal outside the door: only through unknown space. */
    assert(slam_plan_path(g, start, plan_center(31), plan_center(24), &t));
    assert(t.valid && t.frontier);
    /* Door closed: unreachable. Outside the map: no path. */
    plan_cell(g, 29, 24, 40);
    assert(!slam_plan_path(g, start, plan_center(31), plan_center(24), &t));
    assert(!slam_plan_path(g, start, 1e6f, 0, &t));
    free(g);
}

int main(void)
{
    uint16_t raw[360];
    setvbuf(stdout, NULL, _IONBF, 0);
    srand(7);
    slam_reset(&slam);
    double x = 0, y = 0, th = 0;
    uint64_t t = 1000;

    scan(raw, x, y, th, 0, 0, 0);
    assert(slam_update(&slam, raw, 0, true, 100, t, false, NULL));
    assert(slam.initialized && slam.integrated == 1);
    assert(slam_cell_class(slam.grid[SLAM_ORIGIN * SLAM_SIZE + SLAM_ORIGIN]) == 1);

    /* Standing still: pose stays put. */
    for (int i = 0; i < 5; i++) {
        t += 100;
        scan(raw, x, y, th, 0, 0, 0);
        slam_update(&slam, raw, 0, true, 100, t, false, NULL);
    }
    assert(fabs(slam.pose.x) < 20 && fabs(slam.pose.y) < 20 && angle_error(slam.pose.theta, 0) < 0.02);

    /* Drive forward 1 m at 400 mm/s with a gentle left curve. */
    for (int i = 0; i < 25; i++) {
        double w = 0.15, v = 400;
        th += w * 0.1; x += v * cos(th) * 0.1; y += v * sin(th) * 0.1; t += 100;
        scan(raw, x, y, th, v * cos(th), v * sin(th), w);
        slam_update(&slam, raw, 0, true, 100, t, true, NULL);
    }
    printf("drive: true %.0f %.0f %.2f est %.0f %.0f %.2f q %.2f\n",
           x, y, th, slam.pose.x, slam.pose.y, slam.pose.theta, slam.quality);
    assert(fabs(slam.pose.x - x) < 60 && fabs(slam.pose.y - y) < 60);
    assert(angle_error(slam.pose.theta, th) < 0.05);
    assert(!slam.lost && slam.quality > 0.5f);

    /* Spin in place 120 deg at 1.2 rad/s (scan skew must be corrected). */
    for (int i = 0; i < 18; i++) {
        double w = 1.2;
        th += w * 0.1; t += 100;
        scan(raw, x, y, th, 0, 0, w);
        slam_update(&slam, raw, 0, true, 100, t, true, NULL);
    }
    printf("spin:  true %.0f %.0f %.2f est %.0f %.0f %.2f q %.2f\n",
           x, y, th, slam.pose.x, slam.pose.y, slam.pose.theta, slam.quality);
    assert(fabs(slam.pose.x - x) < 80 && fabs(slam.pose.y - y) < 80);
    assert(angle_error(slam.pose.theta, th) < 0.06);

    /* Wall cells of the room appear as occupied in the map. */
    int gx = (int)lround(2500.0 / SLAM_RES_MM) + SLAM_ORIGIN, gy = SLAM_ORIGIN;
    int occupied = 0;
    for (int dx = -1; dx <= 1; dx++)
        occupied |= slam_cell_class(slam.grid[gy * SLAM_SIZE + gx + dx]) == 3;
    assert(occupied);
    assert(slam.known_cells > 1000);

    /* Path: ordered, starts at the origin, about every 5 cm. */
    static int16_t path[SLAM_PATH_MAX][2];
    uint16_t count = slam_path_copy(&slam, path, SLAM_PATH_MAX);
    printf("path: %u first %d %d\n", count, path[0][0], path[0][1]);
    assert(count >= 10 && path[0][0] == 0 && path[0][1] == 0);
    assert(abs(path[count - 1][0] - (int)lround(slam.pose.x / 10)) <= 8);

    /* Exploration bonus: unknown space scores higher than mapped space. */
    int16_t bonus[SLAM_HEADINGS];
    slam_explore_bonus(&slam, slam.pose, bonus);
    for (int k = 0; k < SLAM_HEADINGS; k++) assert(bonus[k] >= 0 && bonus[k] <= 700);
    slam_t *empty = calloc(1, sizeof(slam_t));
    slam_reset(empty);
    int16_t open[SLAM_HEADINGS];
    slam_explore_bonus(empty, (slam_pose_t){0, 0, 0}, open);
    assert(open[9] == 700);
    free(empty);

    /* Garbage scans while driving (reflections, LiDAR glitch): the pose must
       not run away, and real scans relocalise it afterwards. */
    slam_pose_t before = slam.pose;
    for (int i = 0; i < 30; i++) {
        for (int k = 0; k < 360; k++) raw[k] = rand() % 3 ? (uint16_t)(200 + rand() % 3000) : 0;
        t += 100;
        slam_update(&slam, raw, 0, true, 100, t, true, NULL);
    }
    double drift = hypot(slam.pose.x - before.x, slam.pose.y - before.y);
    printf("garbage: drift %.0f mm lost %d\n", drift, slam.lost);
    assert(drift < 300 && slam.lost);
    for (int i = 0; i < 20 && slam.lost; i++) {
        t += 100;
        scan(raw, x, y, th, 0, 0, 0);
        slam_update(&slam, raw, 0, true, 100, t, false, NULL);
    }
    printf("reloc: true %.0f %.0f %.2f est %.0f %.0f %.2f\n",
           x, y, th, slam.pose.x, slam.pose.y, slam.pose.theta);
    assert(!slam.lost && fabs(slam.pose.x - x) < 80 && fabs(slam.pose.y - y) < 80);
    assert(angle_error(slam.pose.theta, th) < 0.06);

    explore_target_tests();
    plan_path_tests();
    latency_spin_test();
    hall_test();

    /* Fast spin at 6.5 rad/s (37 deg per scan, beyond the LiDAR-only search
       window) with a gyro that has 3 % scale error and bias. */
    const double bias = 0.02, scale = 1.03, w = 6.5;
    for (int with_gyro = 0; with_gyro <= 1; with_gyro++) {
        slam_t *g = calloc(1, sizeof(slam_t));
        slam_reset(g);
        double gx2 = 0, gy2 = 0, gth = 0;
        uint64_t gt = 1000;
        for (int i = 0; i < 8; i++) {
            slam_imu_t imu = {i > 0, (float)(bias * 0.1), (float)bias};
            scan(raw, gx2, gy2, gth, 0, 0, 0);
            slam_update(g, raw, 0, true, 100, gt, false, with_gyro ? &imu : NULL);
            gt += 100;
        }
        unsigned lost_scans = 0;
        for (int i = 0; i < 16; i++) {
            gth += w * 0.1;
            slam_imu_t imu = {true, (float)(w * 0.1 * scale + bias * 0.1), (float)(w * scale + bias)};
            scan(raw, gx2, gy2, gth, 0, 0, w);
            if (!slam_update(g, raw, 0, true, 100, gt, true, with_gyro ? &imu : NULL)) lost_scans++;
            gt += 100;
        }
        double err = angle_error(g->pose.theta, gth);
        printf("fast spin %s: true %.2f est %.2f err %.3f failed %u sigma %.3f bias %.4f\n",
               with_gyro ? "gyro" : "lidar", fmod(gth, 2 * PI), g->pose.theta, err, lost_scans,
               slam_heading_sigma(g), g->gyro_bias);
        if (with_gyro) {
            assert(g->imu_used && err < 0.05 && lost_scans <= 2 && !g->lost);
            assert(fabs(g->pose.x) < 80 && fabs(g->pose.y) < 80);
            assert(slam_heading_sigma(g) > 0 && slam_heading_sigma(g) < 0.05f);
        }
        /* Standing still afterwards: the filter learns the residual bias. */
        if (with_gyro) {
            for (int i = 0; i < 150; i++) {
                slam_imu_t imu = {true, (float)(bias * 0.1), (float)bias};
                scan(raw, gx2, gy2, gth, 0, 0, 0);
                slam_update(g, raw, 0, true, 100, gt, false, &imu);
                gt += 100;
            }
            printf("bias learned %.4f (true %.4f), heading err %.3f\n", g->gyro_bias, bias,
                   angle_error(g->pose.theta, gth));
            assert(fabs(g->gyro_bias - bias) < 0.008 && angle_error(g->pose.theta, gth) < 0.03);
        }
        free(g);
    }

    puts("slam tests passed");
    return 0;
}
