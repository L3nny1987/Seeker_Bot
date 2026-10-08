#include "slam.h"
#include <math.h>
#include <string.h>

#define PI_F             3.14159265f
#define DEG2RAD          (PI_F / 180.0f)
#define L_HIT            20
#define L_MISS           (-5)
#define L_MAX            100
#define L_MIN            (-60)
#define L_INLIER         10         /* occupied evidence for the quality check */
#define MATCH_MAX_MM     10000
#define INTEGRATE_MAX_MM 8000
#define MIN_POINTS       40
#define MIN_KNOWN        30         /* quality denominator floor */
#define COARSE_STEP      4          /* every 4th point in the coarse search */
#define GOOD_QUALITY     0.25f
#define LOST_AFTER       15
/* Map updates only from scans that are reliable: slow rotation in this and
   the previous scan (no stop/start transition) and a good match. Scans taken
   while turning still track the pose, but a small heading or LiDAR/gyro
   timing error there smears walls into rings. */
#define MAP_OMEGA        0.35f      /* rad/s (20 deg/s) */
#define MAP_QUALITY      0.4f
/* A failed match must never move the pose far: velocities are bounded and
   decay, implausible jumps are rejected, a lost pose relocalises in a wide
   window around the last good position. */
#define MAX_SPEED_MM_S   700.0f
#define MAX_RATE         4.0f       /* rad/s, LiDAR-only rate estimate */
#define JUMP_MARGIN_MM   100.0f
#define RELOC_EVERY      5          /* scans between wide searches while lost */
#define RELOC_TRANS_MM   600.0f
#define RELOC_QUALITY    0.45f
#define GRID_LIMIT_MM    ((SLAM_ORIGIN - 2) * SLAM_RES_MM)
/* Heading filter tuning (MPU6050 at +-500 deg/s, 44 Hz DLPF). */
#define KF_GYRO_NOISE    0.002f     /* rad per scan step */
#define KF_GYRO_SCALE    0.03f      /* scale-factor error: 3 % of the turn */
#define KF_BIAS_WALK     1e-6f      /* (rad/s)^2 per second */
#define KF_BIAS_INIT     2.5e-5f    /* (0.3 deg/s)^2 after calibration */
#define KF_BIAS_MAX      0.05f      /* rad/s */
#define KF_MATCH_SIGMA   0.012f     /* rad, scan match at quality 1 */
#define KF_SYNC_S        0.03f      /* s, LiDAR vs gyro timestamp uncertainty */

static float s_value[256];          /* log-odds -> occupancy likelihood 0..1 */
static bool s_lut_ready;
static float s_px[360], s_py[360];
static int s_n;

static float wrap_pi(float a)
{
    while (a > PI_F) a -= 2 * PI_F;
    while (a < -PI_F) a += 2 * PI_F;
    return a;
}

static void lut(void)
{
    if (s_lut_ready) return;
    for (int i = 0; i < 256; i++) {
        int l = (int8_t)(uint8_t)i;
        s_value[i] = l <= 0 ? 0.0f : l >= 50 ? 1.0f : l / 50.0f;
    }
    s_lut_ready = true;
}

static inline float value(int8_t l) { return s_value[(uint8_t)l]; }

static int cell_of(float w) { return (int)lroundf(w / SLAM_RES_MM) + SLAM_ORIGIN; }

void slam_reset(slam_t *s)
{
    lut();
    memset(s, 0, sizeof(*s));
}

/* Bilinear occupancy likelihood and its gradient (per mm). */
static float sample(const slam_t *s, float wx, float wy, float *dx, float *dy)
{
    float gx = wx / SLAM_RES_MM + SLAM_ORIGIN, gy = wy / SLAM_RES_MM + SLAM_ORIGIN;
    int ix = (int)floorf(gx), iy = (int)floorf(gy);
    if (ix < 0 || iy < 0 || ix >= SLAM_SIZE - 1 || iy >= SLAM_SIZE - 1) {
        if (dx) { *dx = 0; *dy = 0; }
        return 0;
    }
    float fx = gx - ix, fy = gy - iy;
    const int8_t *g = &s->grid[iy * SLAM_SIZE + ix];
    float v00 = value(g[0]), v10 = value(g[1]), v01 = value(g[SLAM_SIZE]), v11 = value(g[SLAM_SIZE + 1]);
    if (dx) {
        *dx = ((1 - fy) * (v10 - v00) + fy * (v11 - v01)) / SLAM_RES_MM;
        *dy = ((1 - fx) * (v01 - v00) + fx * (v11 - v10)) / SLAM_RES_MM;
    }
    return (1 - fy) * ((1 - fx) * v00 + fx * v10) + fy * ((1 - fx) * v01 + fx * v11);
}

/* Scan points in the robot frame at the end of the scan (x forward, y left).
   Bins are measured in raw-angle order; the LD20 timestamp marks the end of
   the revolution, so earlier bins are corrected by the rotation since then. */
static void build_points(const uint16_t raw[360], int front, bool clockwise, float omega, float period_ms)
{
    s_n = 0;
    for (int i = 0; i < 360; i++) {
        uint16_t d = raw[i];
        if (!d || d > MATCH_MAX_MM) continue;
        int a = clockwise ? i - front : front - i;          /* robot CW degrees */
        float age = period_ms * 0.001f * (359.5f - i) / 360.0f;
        float alpha = -a * DEG2RAD - omega * age;
        s_px[s_n] = d * cosf(alpha);
        s_py[s_n] = d * sinf(alpha);
        s_n++;
    }
}

static float score(const slam_t *s, slam_pose_t p, int step)
{
    float c = cosf(p.theta), sn = sinf(p.theta), sum = 0;
    for (int i = 0; i < s_n; i += step)
        sum += sample(s, p.x + c * s_px[i] - sn * s_py[i], p.y + sn * s_px[i] + c * s_py[i], NULL, NULL);
    return sum;
}

static slam_pose_t coarse(const slam_t *s, slam_pose_t pred, float rot, float trans,
                          float rot_step, float trans_step, int point_step)
{
    slam_pose_t best = pred;
    float best_score = -1e9f;
    for (float dt = -rot; dt <= rot + 1e-4f; dt += rot_step)
        for (float ox = -trans; ox <= trans + 1e-3f; ox += trans_step)
            for (float oy = -trans; oy <= trans + 1e-3f; oy += trans_step) {
                slam_pose_t p = {pred.x + ox, pred.y + oy, pred.theta + dt};
                /* Mild prior toward the prediction breaks ties in corridors. */
                float sc = score(s, p, point_step) - 0.004f * (fabsf(ox) + fabsf(oy)) - 3.0f * fabsf(dt);
                if (sc > best_score) { best_score = sc; best = p; }
            }
    return best;
}

/* Gauss-Newton on 1 - M(S(p)) with bilinear map gradients. */
static slam_pose_t refine(const slam_t *s, slam_pose_t p)
{
    for (int it = 0; it < 8; it++) {
        float h00 = 0, h01 = 0, h02 = 0, h11 = 0, h12 = 0, h22 = 0, b0 = 0, b1 = 0, b2 = 0;
        float c = cosf(p.theta), sn = sinf(p.theta);
        for (int i = 0; i < s_n; i++) {
            float gx, gy;
            float m = sample(s, p.x + c * s_px[i] - sn * s_py[i], p.y + sn * s_px[i] + c * s_py[i], &gx, &gy);
            float r = 1.0f - m;
            float j2 = gx * (-sn * s_px[i] - c * s_py[i]) + gy * (c * s_px[i] - sn * s_py[i]);
            h00 += gx * gx; h01 += gx * gy; h02 += gx * j2;
            h11 += gy * gy; h12 += gy * j2; h22 += j2 * j2;
            b0 += gx * r; b1 += gy * r; b2 += j2 * r;
        }
        h00 = h00 * 1.1f + 1e-6f; h11 = h11 * 1.1f + 1e-6f; h22 = h22 * 1.1f + 1e-3f;
        float det = h00 * (h11 * h22 - h12 * h12) - h01 * (h01 * h22 - h12 * h02) + h02 * (h01 * h12 - h11 * h02);
        if (fabsf(det) < 1e-12f) break;
        float dx = (b0 * (h11 * h22 - h12 * h12) - h01 * (b1 * h22 - h12 * b2) + h02 * (b1 * h12 - h11 * b2)) / det;
        float dy = (h00 * (b1 * h22 - h12 * b2) - b0 * (h01 * h22 - h12 * h02) + h02 * (h01 * b2 - b1 * h02)) / det;
        float dth = (h00 * (h11 * b2 - b1 * h12) - h01 * (h01 * b2 - b1 * h02) + b0 * (h01 * h12 - h11 * h02)) / det;
        if (!isfinite(dx) || !isfinite(dy) || !isfinite(dth)) break;
        if (dx > 40) dx = 40; else if (dx < -40) dx = -40;
        if (dy > 40) dy = 40; else if (dy < -40) dy = -40;
        if (dth > 0.05f) dth = 0.05f; else if (dth < -0.05f) dth = -0.05f;
        p.x += dx; p.y += dy; p.theta += dth;
        if (fabsf(dx) < 1 && fabsf(dy) < 1 && fabsf(dth) < 0.001f) break;
    }
    p.theta = wrap_pi(p.theta);
    return p;
}

/* Fraction of scan points on mapped walls (an occupied cell within one cell),
   counted only where the map already knows something (3x3 cells around the
   point). Points in never seen space (far walls, new areas, beyond the grid)
   are new information, not a mismatch. Far walls are hit by only one beam
   per cell and stay weak in the log-odds map; a bilinear threshold rejected
   them. Both made the quality collapse in large rooms until the map stopped
   updating and the pose was lost. */
static float inliers(const slam_t *s, slam_pose_t p)
{
    float c = cosf(p.theta), sn = sinf(p.theta);
    int hits = 0, known = 0;
    for (int i = 0; i < s_n; i++) {
        int gx = cell_of(p.x + c * s_px[i] - sn * s_py[i]), gy = cell_of(p.y + sn * s_px[i] + c * s_py[i]);
        if (gx < 1 || gy < 1 || gx >= SLAM_SIZE - 1 || gy >= SLAM_SIZE - 1) continue;
        const int8_t *g = &s->grid[gy * SLAM_SIZE + gx];
        bool seen = false, hit = false;
        for (int dy = -1; dy <= 1; dy++)
            for (int dx = -1; dx <= 1; dx++) {
                int8_t l = g[dy * SLAM_SIZE + dx];
                seen |= l != 0;
                hit |= l >= L_INLIER;
            }
        known += seen;
        hits += hit;
    }
    return (float)hits / (float)(known > MIN_KNOWN ? known : MIN_KNOWN);
}

static void update_cell(slam_t *s, int gx, int gy, int delta)
{
    int8_t *cell = &s->grid[gy * SLAM_SIZE + gx];
    int l = *cell + delta;
    *cell = (int8_t)(l > L_MAX ? L_MAX : l < L_MIN ? L_MIN : l);
}

static void ray(slam_t *s, int x0, int y0, int x1, int y1)
{
    int dx = x1 > x0 ? x1 - x0 : x0 - x1, dy = y1 > y0 ? y0 - y1 : y1 - y0;
    int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1, err = dx + dy;
    for (;;) {
        if (x0 < 0 || y0 < 0 || x0 >= SLAM_SIZE || y0 >= SLAM_SIZE) return;
        if (x0 == x1 && y0 == y1) { update_cell(s, x0, y0, L_HIT); return; }
        update_cell(s, x0, y0, L_MISS);
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

static void integrate(slam_t *s, slam_pose_t p)
{
    float c = cosf(p.theta), sn = sinf(p.theta);
    int x0 = cell_of(p.x), y0 = cell_of(p.y);
    for (int i = 0; i < s_n; i++) {
        if (s_px[i] * s_px[i] + s_py[i] * s_py[i] > (float)INTEGRATE_MAX_MM * INTEGRATE_MAX_MM) continue;
        ray(s, x0, y0, cell_of(p.x + c * s_px[i] - sn * s_py[i]), cell_of(p.y + sn * s_px[i] + c * s_py[i]));
    }
    s->integrated_pose = p;
    s->since_integrate = 0;
    if (++s->integrated % 10 == 1) {
        unsigned known = 0;
        for (int i = 0; i < SLAM_SIZE * SLAM_SIZE; i++) known += slam_cell_class(s->grid[i]) != 0;
        s->known_cells = known;
    }
}

static void track(slam_t *s, slam_pose_t p)
{
    int vx = cell_of(p.x) / SLAM_VISIT_CELLS, vy = cell_of(p.y) / SLAM_VISIT_CELLS;
    if (vx >= 0 && vy >= 0 && vx < SLAM_VISIT_SIZE && vy < SLAM_VISIT_SIZE) {
        uint8_t *visit = &s->visits[vy * SLAM_VISIT_SIZE + vx];
        if (*visit < 255) (*visit)++;
    }
    if (s->path_count && hypotf(p.x - s->path_x, p.y - s->path_y) < 50.0f) return;
    s->path_x = p.x; s->path_y = p.y;
    s->path[s->path_head][0] = (int16_t)lroundf(p.x / 10.0f);
    s->path[s->path_head][1] = (int16_t)lroundf(p.y / 10.0f);
    s->path_head = (s->path_head + 1) % SLAM_PATH_MAX;
    if (s->path_count < SLAM_PATH_MAX) s->path_count++;
}

bool slam_update(slam_t *s, const uint16_t raw[360], int front_deg, bool clockwise,
                 float period_ms, uint64_t now_ms, bool moving, const slam_imu_t *imu)
{
    lut();
    s->scans++;
    float dt = s->last_ms ? (now_ms - s->last_ms) * 0.001f : 0.1f;
    if (dt < 0.02f) dt = 0.02f;
    if (dt > 0.5f) dt = 0.5f;
    s->last_ms = now_ms;
    if (!moving) { s->vx = 0; s->vy = 0; s->omega = 0; }
    bool gyro = imu && imu->valid && s->initialized;
    s->imu_used = gyro;
    if (!gyro) s->kf_ready = false;
    /* The gyro also sees rotations without a motor command (pushed, slipping). */
    float omega = gyro ? imu->omega - s->gyro_bias : s->omega;
    float previous_rate = s->last_rate;
    build_points(raw, front_deg, clockwise, omega, period_ms);
    s->last_rate = omega;
    if (s_n < MIN_POINTS) { s->rejected++; return false; }
    if (!s->initialized) {
        s->initialized = true;
        integrate(s, s->pose);
        track(s, s->pose);
        s->quality = 1;
        return true;
    }
    slam_pose_t pred = {s->pose.x + s->vx * dt, s->pose.y + s->vy * dt, s->pose.theta + s->omega * dt};
    float rot = !moving ? 0.06f : fabsf(s->omega) > 0.4f ? 0.35f : 0.17f;
    float trans = moving ? 150.0f : 50.0f;
    if (gyro) {
        if (!s->kf_ready) {
            s->kf_p00 = 0.02f * 0.02f;
            s->kf_p01 = 0;
            if (s->kf_p11 <= 0) s->kf_p11 = KF_BIAS_INIT;
            s->kf_ready = true;
        }
        /* Predict: theta += gyro - bias * dt; bias is a random walk. */
        float step = imu->dtheta - s->gyro_bias * dt;
        /* A timing offset between scan and gyro turns rotation into heading
           error: while turning, the scan match must dominate the gyro. */
        float rate = fabsf(imu->omega) > fabsf(previous_rate) ? imu->omega : previous_rate;
        float q = KF_GYRO_NOISE * KF_GYRO_NOISE + (KF_GYRO_SCALE * step) * (KF_GYRO_SCALE * step) +
                  (KF_SYNC_S * rate) * (KF_SYNC_S * rate);
        s->kf_p00 += -2 * dt * s->kf_p01 + dt * dt * s->kf_p11 + q;
        s->kf_p01 -= dt * s->kf_p11;
        s->kf_p11 += KF_BIAS_WALK * dt;
        pred.theta = s->pose.theta + step;
        /* Search only where the filter says the heading can be (3 sigma). */
        rot = 3 * sqrtf(s->kf_p00);
        rot = rot < 0.035f ? 0.035f : rot > 0.35f ? 0.35f : rot;
        if (s->bad_matches) trans += 50.0f * (s->bad_matches < 3 ? s->bad_matches : 3);
    }
    pred.theta = wrap_pi(pred.theta);
    bool reloc = s->lost;
    slam_pose_t p;
    if (reloc) {
        /* Wide search is expensive: only every few scans while lost. */
        if (s->bad_matches++ % RELOC_EVERY) {
            s->rejected++;
            s->vx = s->vy = 0;
            s->pose.theta = pred.theta;
            return false;
        }
        float wide = gyro ? (rot > 0.35f ? rot : 0.35f) : PI_F;
        p = refine(s, coarse(s, pred, wide, RELOC_TRANS_MM, 0.07f, 100.0f, 2 * COARSE_STEP));
        trans = RELOC_TRANS_MM; rot = wide;
    } else {
        p = refine(s, coarse(s, pred, rot, trans, rot > 0.2f ? 0.035f : 0.0175f, 50.0f, COARSE_STEP));
    }
    s->quality = inliers(s, p);
    bool plausible = hypotf(p.x - pred.x, p.y - pred.y) <= trans + JUMP_MARGIN_MM &&
                     fabsf(wrap_pi(p.theta - pred.theta)) <= rot + 0.1f &&
                     fabsf(p.x) < GRID_LIMIT_MM && fabsf(p.y) < GRID_LIMIT_MM;
    if (s->quality < (reloc ? RELOC_QUALITY : GOOD_QUALITY) || !plausible) {
        s->rejected++;
        if (!reloc && ++s->bad_matches >= LOST_AFTER) s->lost = true;
        /* Hold the position (decaying velocity), heading from the gyro:
           extrapolating an unverified velocity let the pose run away. */
        s->vx *= 0.5f; s->vy *= 0.5f;
        if (!gyro) s->omega *= 0.5f;
        if (fabsf(pred.x) < GRID_LIMIT_MM && fabsf(pred.y) < GRID_LIMIT_MM) s->pose = pred;
        else s->pose.theta = pred.theta;
        return false;
    }
    s->bad_matches = 0;
    s->lost = false;
    if (reloc) { s->vx = s->vy = s->omega = 0; }
    if (gyro) {
        /* Update with the matched heading; its noise grows with poor matches. */
        float r = KF_MATCH_SIGMA / s->quality;
        float innovation = wrap_pi(p.theta - pred.theta);
        float sum = s->kf_p00 + r * r, k0 = s->kf_p00 / sum, k1 = s->kf_p01 / sum;
        p.theta = wrap_pi(pred.theta + k0 * innovation);
        s->gyro_bias += k1 * innovation;
        if (s->gyro_bias > KF_BIAS_MAX) s->gyro_bias = KF_BIAS_MAX;
        if (s->gyro_bias < -KF_BIAS_MAX) s->gyro_bias = -KF_BIAS_MAX;
        s->kf_p11 -= k1 * s->kf_p01;
        s->kf_p01 *= 1 - k0;
        s->kf_p00 *= 1 - k0;
    }
    if (moving && !reloc) {
        float measured = wrap_pi(p.theta - s->pose.theta) / dt;
        s->vx = 0.5f * s->vx + 0.5f * (p.x - s->pose.x) / dt;
        s->vy = 0.5f * s->vy + 0.5f * (p.y - s->pose.y) / dt;
        s->omega = 0.5f * s->omega + 0.5f * measured;
        float speed = hypotf(s->vx, s->vy);
        if (speed > MAX_SPEED_MM_S) { s->vx *= MAX_SPEED_MM_S / speed; s->vy *= MAX_SPEED_MM_S / speed; }
        if (s->omega > MAX_RATE) s->omega = MAX_RATE;
        if (s->omega < -MAX_RATE) s->omega = -MAX_RATE;
    }
    s->pose = p;
    track(s, p);
    s->since_integrate++;
    bool moved = hypotf(p.x - s->integrated_pose.x, p.y - s->integrated_pose.y) > 50.0f ||
                 fabsf(wrap_pi(p.theta - s->integrated_pose.theta)) > 0.05f;
    bool steady = fabsf(omega) < MAP_OMEGA && fabsf(previous_rate) < MAP_OMEGA;
    if (steady && s->quality >= MAP_QUALITY && (moved || s->since_integrate >= 10)) integrate(s, p);
    return true;
}

float slam_heading_sigma(const slam_t *s) { return s->kf_ready ? sqrtf(s->kf_p00) : 0; }

void slam_explore_bonus(const slam_t *s, slam_pose_t pose, int16_t out[SLAM_HEADINGS])
{
    for (int k = 0; k < SLAM_HEADINGS; k++) {
        float dir = pose.theta - (k * 10 - 90) * DEG2RAD;
        float c = cosf(dir), sn = sinf(dir);
        int unknown = 0, unvisited = 0;
        for (int d = 300; d <= 2000; d += 100) {
            int gx = cell_of(pose.x + c * d), gy = cell_of(pose.y + sn * d);
            if (gx < 0 || gy < 0 || gx >= SLAM_SIZE || gy >= SLAM_SIZE) break;
            int8_t l = s->grid[gy * SLAM_SIZE + gx];
            if (l >= SLAM_OCCUPIED) break;
            if (l > SLAM_FREE) unknown++;
            if (!s->visits[(gy / SLAM_VISIT_CELLS) * SLAM_VISIT_SIZE + gx / SLAM_VISIT_CELLS]) unvisited++;
        }
        int bonus = unknown * 25 + unvisited * 15;
        out[k] = (int16_t)(bonus > 700 ? 700 : bonus);
    }
}

/* Planning grid: 20 cm cells. Blocked if any fine cell is occupied, free
   if at least a quarter of it was seen free, otherwise unknown. One free
   cell between two blocked ones leaves >= 20 cm: wider than the body. */
#define PLAN_N           SLAM_PLAN_SIZE
#define PLAN_UNKNOWN     0
#define PLAN_FREE        1
#define PLAN_BLOCKED     2
#define PLAN_MIN_FREE    4
#define PLAN_FRONTIER_MIN 3         /* frontiers closer than 60 cm are done */
#define PLAN_COVER_MIN   5          /* revisit targets at least 1 m away */
#define PLAN_COVER_COST  40         /* frontiers win up to 8 m further away */
#define PLAN_LOOKAHEAD   5          /* waypoint ~1 m along the path */

static uint8_t s_plan[PLAN_N * PLAN_N];
static uint16_t s_steps[PLAN_N * PLAN_N], s_queue[PLAN_N * PLAN_N];

static void plan_classify(const slam_t *s)
{
    for (int cy = 0; cy < PLAN_N; cy++)
        for (int cx = 0; cx < PLAN_N; cx++) {
            int free = 0, occupied = 0;
            for (int j = 0; j < SLAM_PLAN_CELLS; j++) {
                const int8_t *row = &s->grid[(cy * SLAM_PLAN_CELLS + j) * SLAM_SIZE + cx * SLAM_PLAN_CELLS];
                for (int i = 0; i < SLAM_PLAN_CELLS; i++) {
                    if (row[i] >= SLAM_OCCUPIED) occupied++;
                    else if (row[i] <= SLAM_FREE) free++;
                }
            }
            s_plan[cy * PLAN_N + cx] = occupied ? PLAN_BLOCKED : free >= PLAN_MIN_FREE ? PLAN_FREE : PLAN_UNKNOWN;
        }
}

static float plan_world(int c)
{
    return ((float)c * SLAM_PLAN_CELLS + 0.5f * (SLAM_PLAN_CELLS - 1) - SLAM_ORIGIN) * SLAM_RES_MM;
}

bool slam_explore_target(const slam_t *s, slam_pose_t pose, const uint8_t *avoid, slam_target_t *out)
{
    static const int DX[4] = {1, -1, 0, 0}, DY[4] = {0, 0, 1, -1};
    memset(out, 0, sizeof(*out));
    int gx = cell_of(pose.x), gy = cell_of(pose.y);
    if (gx < 0 || gy < 0 || gx >= SLAM_SIZE || gy >= SLAM_SIZE) return false;
    plan_classify(s);
    for (int i = 0; i < PLAN_N * PLAN_N; i++) s_steps[i] = UINT16_MAX;
    int start = (gy / SLAM_PLAN_CELLS) * PLAN_N + gx / SLAM_PLAN_CELLS;
    int head = 0, tail = 0, best = -1;
    unsigned best_cost = UINT32_MAX;
    bool best_frontier = false;
    s_steps[start] = 0;
    s_queue[tail++] = (uint16_t)start;
    while (head < tail) {
        int c = s_queue[head++], cx = c % PLAN_N, cy = c / PLAN_N;
        unsigned steps = s_steps[c];
        bool frontier = false;
        for (int n = 0; n < 4; n++) {
            int nx = cx + DX[n], ny = cy + DY[n];
            if (nx < 0 || ny < 0 || nx >= PLAN_N || ny >= PLAN_N) continue;
            int m = ny * PLAN_N + nx;
            if (s_plan[m] == PLAN_UNKNOWN) frontier = true;
            if (s_plan[m] == PLAN_FREE && s_steps[m] == UINT16_MAX) {
                s_steps[m] = (uint16_t)(steps + 1);
                s_queue[tail++] = (uint16_t)m;
            }
        }
        if (avoid && avoid[c]) continue;
        unsigned cost = UINT32_MAX;
        if (frontier && steps >= PLAN_FRONTIER_MIN) cost = steps;
        else if (!frontier && steps >= PLAN_COVER_MIN) {
            int vx = cx * SLAM_PLAN_CELLS / SLAM_VISIT_CELLS, vy = cy * SLAM_PLAN_CELLS / SLAM_VISIT_CELLS;
            unsigned visits = s->visits[vy * SLAM_VISIT_SIZE + vx];
            cost = PLAN_COVER_COST + steps + (visits > 60 ? 60 : visits);
        }
        if (cost < best_cost) { best_cost = cost; best = c; best_frontier = frontier; }
    }
    if (best < 0) return false;
    /* Walk back along decreasing step counts to the look-ahead waypoint. */
    int way = best;
    while (s_steps[way] > PLAN_LOOKAHEAD) {
        int wx = way % PLAN_N, wy = way / PLAN_N;
        for (int n = 0; n < 4; n++) {
            int nx = wx + DX[n], ny = wy + DY[n];
            if (nx < 0 || ny < 0 || nx >= PLAN_N || ny >= PLAN_N) continue;
            if (s_steps[ny * PLAN_N + nx] == s_steps[way] - 1) { way = ny * PLAN_N + nx; break; }
        }
    }
    out->valid = true;
    out->frontier = best_frontier;
    out->cell = (uint16_t)best;
    out->steps = s_steps[best];
    out->x = plan_world(best % PLAN_N);
    out->y = plan_world(best / PLAN_N);
    out->way_x = plan_world(way % PLAN_N);
    out->way_y = plan_world(way / PLAN_N);
    return true;
}

#define PATH_LOOKAHEAD   8          /* waypoint up to 1.6 m along the path */

static bool plan_passable(int c, bool unknown)
{
    return s_plan[c] == PLAN_FREE || (unknown && s_plan[c] == PLAN_UNKNOWN);
}

/* Straight segment between two planning cells (start cell ignored: the
   robot may stand next to a wall). Sampled every quarter cell. */
static bool plan_visible(int a, int b, bool unknown)
{
    float ax = a % PLAN_N + 0.5f, ay = a / PLAN_N + 0.5f;
    float bx = b % PLAN_N + 0.5f, by = b / PLAN_N + 0.5f;
    int n = (int)(hypotf(bx - ax, by - ay) * 4.0f) + 1;
    for (int i = 1; i <= n; i++) {
        float t = (float)i / (float)n;
        int c = (int)(ay + (by - ay) * t) * PLAN_N + (int)(ax + (bx - ax) * t);
        if (c != a && !plan_passable(c, unknown)) return false;
    }
    return true;
}

/* Breadth-first from the goal; the start cell is always enterable. */
static bool plan_from_goal(int goal, int start, bool unknown)
{
    static const int DX[4] = {1, -1, 0, 0}, DY[4] = {0, 0, 1, -1};
    for (int i = 0; i < PLAN_N * PLAN_N; i++) s_steps[i] = UINT16_MAX;
    int head = 0, tail = 0;
    s_steps[goal] = 0;
    s_queue[tail++] = (uint16_t)goal;
    while (head < tail) {
        int c = s_queue[head++], cx = c % PLAN_N, cy = c / PLAN_N;
        if (c == start) return true;
        for (int n = 0; n < 4; n++) {
            int nx = cx + DX[n], ny = cy + DY[n];
            if (nx < 0 || ny < 0 || nx >= PLAN_N || ny >= PLAN_N) continue;
            int m = ny * PLAN_N + nx;
            if (s_steps[m] != UINT16_MAX || (m != start && !plan_passable(m, unknown))) continue;
            s_steps[m] = (uint16_t)(s_steps[c] + 1);
            s_queue[tail++] = (uint16_t)m;
        }
    }
    return false;
}

bool slam_plan_path(const slam_t *s, slam_pose_t pose, float goal_x, float goal_y, slam_target_t *out)
{
    static const int DX[8] = {1, -1, 0, 0, 1, 1, -1, -1}, DY[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    memset(out, 0, sizeof(*out));
    int rx = cell_of(pose.x), ry = cell_of(pose.y), gx = cell_of(goal_x), gy = cell_of(goal_y);
    if (rx < 0 || ry < 0 || rx >= SLAM_SIZE || ry >= SLAM_SIZE) return false;
    if (gx < 0 || gy < 0 || gx >= SLAM_SIZE || gy >= SLAM_SIZE) return false;
    plan_classify(s);
    int start = (ry / SLAM_PLAN_CELLS) * PLAN_N + rx / SLAM_PLAN_CELLS;
    int goal = (gy / SLAM_PLAN_CELLS) * PLAN_N + gx / SLAM_PLAN_CELLS;
    out->x = goal_x;
    out->y = goal_y;
    /* Goal inside an obstacle cell (click next to a wall): nearest open
       neighbour cell becomes the goal. */
    if (s_plan[goal] == PLAN_BLOCKED && goal != start) {
        int best = -1;
        float best_d = 1e9f;
        for (int n = 0; n < 8; n++) {
            int nx = goal % PLAN_N + DX[n], ny = goal / PLAN_N + DY[n];
            if (nx < 0 || ny < 0 || nx >= PLAN_N || ny >= PLAN_N || s_plan[ny * PLAN_N + nx] == PLAN_BLOCKED) continue;
            float d = hypotf(plan_world(nx) - goal_x, plan_world(ny) - goal_y);
            if (d < best_d) { best_d = d; best = ny * PLAN_N + nx; }
        }
        if (best < 0) return false;
        goal = best;
        out->x = plan_world(goal % PLAN_N);
        out->y = plan_world(goal / PLAN_N);
    }
    /* Known free space first, unknown space only as a fallback. */
    bool unknown = false;
    if (!plan_from_goal(goal, start, false)) {
        unknown = true;
        if (!plan_from_goal(goal, start, true)) return false;
    }
    /* Follow decreasing step counts (towards the goal when tied) and keep
       the farthest cell still in straight line of sight. */
    int c = start, way = start;
    for (int k = 0; k < PATH_LOOKAHEAD && c != goal; k++) {
        int next = -1;
        float next_d = 1e9f;
        for (int n = 0; n < 4; n++) {
            int nx = c % PLAN_N + DX[n], ny = c / PLAN_N + DY[n];
            if (nx < 0 || ny < 0 || nx >= PLAN_N || ny >= PLAN_N) continue;
            int m = ny * PLAN_N + nx;
            if (s_steps[m] != s_steps[c] - 1) continue;
            float d = hypotf(plan_world(nx) - out->x, plan_world(ny) - out->y);
            if (d < next_d) { next_d = d; next = m; }
        }
        if (next < 0) break;
        c = next;
        if (!plan_visible(start, c, unknown)) break;
        way = c;
    }
    out->valid = true;
    out->frontier = unknown;
    out->cell = (uint16_t)goal;
    out->steps = s_steps[start];
    if (way == goal) {
        out->way_x = out->x;
        out->way_y = out->y;
    } else {
        out->way_x = plan_world(way % PLAN_N);
        out->way_y = plan_world(way / PLAN_N);
    }
    return true;
}

uint16_t slam_path_copy(const slam_t *s, int16_t (*out)[2], uint16_t max)
{
    uint16_t count = s->path_count, head = s->path_head;
    if (count > max) count = max;
    for (uint16_t i = 0; i < count; i++) {
        uint16_t index = (uint16_t)((head + SLAM_PATH_MAX - count + i) % SLAM_PATH_MAX);
        out[i][0] = s->path[index][0];
        out[i][1] = s->path[index][1];
    }
    return count;
}
