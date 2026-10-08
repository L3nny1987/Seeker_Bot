#include "navigation.h"
#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* LD20 scans regularly miss single returns (dark/glossy surfaces, > 8 m).
   Sector statistics tolerate small gaps; corridors that are mostly
   unmeasured are capped to a short horizon instead of counted as free. */
#define SECTOR_MIN_PERCENT   60
#define SECTOR_MAX_GAP       10
#define ROTATION_MIN_PERCENT 60
#define ROTATION_MAX_GAP     30
#define CONE_DEG             20
#define CONE_MIN_PERCENT     50
#define WALL_RANGE_MM        2000
#define WALL_MAX_RMS_MM      60
#define PI_F                 3.14159265f
#define DEG2RAD              (PI_F / 180.0f)
#define RAD2DEG              (180.0f / PI_F)

static float s_sin[360];
static bool s_sin_ready;

static float sin_deg(int a) { return s_sin[((a % 360) + 360) % 360]; }
static float cos_deg(int a) { return sin_deg(a + 90); }
static float clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }

static uint16_t sector(const uint16_t r[360], int center, int half_width, bool *known)
{
    uint16_t min = UINT16_MAX;
    unsigned valid = 0, run = 0, gap = 0, total = 2 * half_width + 1;
    for (int a = center - half_width; a <= center + half_width; a++) {
        uint16_t value = r[(a + 720) % 360];
        if (value) {
            valid++; run = 0;
            if (value < min) min = value;
        } else if (++run > gap) gap = run;
    }
    *known = valid * 100 >= total * SECTOR_MIN_PERCENT && gap <= SECTOR_MAX_GAP;
    return min == UINT16_MAX ? 0 : min;
}

static uint16_t full_circle(const uint16_t r[360], bool *known)
{
    uint16_t min = UINT16_MAX;
    unsigned valid = 0, run = 0, gap = 0;
    for (int i = 0; i < 720; i++) {   /* twice: gaps across 0 deg count */
        uint16_t value = r[i % 360];
        if (value) {
            run = 0;
            if (i < 360) { valid++; if (value < min) min = value; }
        } else if (++run > gap) gap = run;
    }
    *known = valid * 100 >= 360 * ROTATION_MIN_PERCENT && gap <= ROTATION_MAX_GAP;
    return min == UINT16_MAX ? 0 : min;
}

/* Free length of a straight corridor (robot width) toward heading. */
static uint16_t corridor(const uint16_t r[360], int heading, int half_width, bool *uncertain)
{
    float length = NAV_HORIZON_MM;
    unsigned cone = 0, measured = 0;
    for (int a = 0; a < 360; a++) {
        int rel = ((a - heading) % 360 + 540) % 360 - 180;
        if (rel <= -90 || rel >= 90) continue;
        if (rel >= -CONE_DEG && rel <= CONE_DEG) { cone++; if (r[a]) measured++; }
        if (!r[a]) continue;
        float x = r[a] * cos_deg(rel), y = r[a] * sin_deg(rel);
        if (fabsf(y) < half_width && x < length) length = x;
    }
    *uncertain = measured * 100 < cone * CONE_MIN_PERCENT;
    if (*uncertain && length > NAV_UNCERTAIN_MM) length = NAV_UNCERTAIN_MM;
    return length <= 0 ? 0 : (uint16_t)length;
}

/* Total-least-squares line through the nearest surface on one side
   (side +1 = right, -1 = left). x forward, y lateral toward that side. */
static navigation_wall_t fit_wall(const uint16_t r[360], int side)
{
    navigation_wall_t w = {0};
    float nearest = WALL_RANGE_MM + 1;
    for (int b = 60; b <= 120; b++) {
        uint16_t d = r[(side * b + 360) % 360];
        if (d && d <= WALL_RANGE_MM && d * sin_deg(b) < nearest) nearest = d * sin_deg(b);
    }
    if (nearest > WALL_RANGE_MM) return w;
    float sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0;
    unsigned n = 0;
    for (int b = 45; b <= 135; b++) {
        uint16_t d = r[(side * b + 360) % 360];
        if (!d || d > WALL_RANGE_MM) continue;
        float x = d * cos_deg(b), y = d * sin_deg(b);
        if (y > nearest + 250) continue;          /* only the nearest surface */
        sx += x; sy += y; sxx += x * x; syy += y * y; sxy += x * y; n++;
    }
    w.valid = true;
    w.distance_mm = (uint16_t)nearest;
    if (n < 8) return w;
    float mx = sx / n, my = sy / n;
    float cxx = sxx / n - mx * mx, cyy = syy / n - my * my, cxy = sxy / n - mx * my;
    float phi = 0.5f * atan2f(2 * cxy, cxx - cyy);
    float half = 0.5f * (cxx - cyy);
    float lambda = 0.5f * (cxx + cyy) - sqrtf(half * half + cxy * cxy);
    float rms = sqrtf(lambda > 0 ? lambda : 0);
    if (rms > WALL_MAX_RMS_MM || fabsf(phi) > 60 * DEG2RAD) return w;   /* corner/clutter */
    w.distance_mm = (uint16_t)fabsf(my * cosf(phi) - mx * sinf(phi));
    w.angle_deg = (int16_t)lroundf(phi * RAD2DEG);
    return w;
}

void navigation_fuse(const uint16_t current[360], const uint16_t *previous, uint16_t out[360])
{
    for (int i = 0; i < 360; i++)
        out[i] = current[i] ? current[i] : previous ? previous[i] : 0;
}

static uint16_t minus(uint16_t value, int body)
{
    return value > body ? (uint16_t)(value - body) : 0;
}

/* Margin of the rotating body: nearest point to the rotation centre minus
   the radius swept by the body corners. */
static uint16_t swing(const uint16_t r[360], navigation_body_t body)
{
    float cx = 0.5f * (body.front_mm - body.rear_mm);
    float radius = hypotf(0.5f * (body.front_mm + body.rear_mm), (float)body.half_width_mm);
    float nearest = -1;
    for (int a = 0; a < 360; a++) {
        if (!r[a]) continue;
        float distance = hypotf(r[a] * cos_deg(a) - cx, r[a] * sin_deg(a));
        if (nearest < 0 || distance < nearest) nearest = distance;
    }
    return nearest <= radius ? 0 : (uint16_t)(nearest - radius);
}

#define NAV_TIGHT_SIDE_MM 15            /* lateral reserve in tight spots */

navigation_view_t navigation_analyze(const uint16_t d[360], int front, bool clockwise,
                                     int corridor_width_mm, navigation_body_t body)
{
    if (!s_sin_ready) {
        for (int i = 0; i < 360; i++) s_sin[i] = sinf(i * DEG2RAD);
        s_sin_ready = true;
    }
    uint16_t r[360];   /* robot frame: index = clockwise angle from front */
    for (int a = 0; a < 360; a++)
        r[a] = d[((front + (clockwise ? a : -a)) % 360 + 720) % 360];
    navigation_view_t v = {0};
    v.front_mm = sector(r, 0, 35, &v.front_known);
    v.rear_mm = sector(r, 180, 35, &v.rear_known);
    v.left_mm = sector(r, -90, 35, &v.left_known);
    v.right_mm = sector(r, 90, 35, &v.right_known);
    v.nearest_mm = full_circle(r, &v.rotation_known);
    int half = corridor_width_mm / 2;
    for (int k = 0; k < NAV_HEADINGS; k++) {
        bool uncertain;
        v.free_mm[k] = minus(corridor(r, navigation_heading_deg(k), half, &uncertain), body.front_mm);
        if (uncertain) v.uncertain_mask |= 1u << k;
    }
    v.rear_free_mm = minus(corridor(r, 180, half, &v.rear_uncertain), body.rear_mm);
    bool ignored;
    int tight = body.half_width_mm + NAV_TIGHT_SIDE_MM;
    v.tight_front_mm = minus(corridor(r, 0, tight, &ignored), body.front_mm);
    v.tight_rear_mm = minus(corridor(r, 180, tight, &ignored), body.rear_mm);
    v.swing_mm = swing(r, body);
    v.wall_left = fit_wall(r, -1);
    v.wall_right = fit_wall(r, 1);
    return v;
}

void navigation_auto_reset(navigation_auto_t *s)
{
    navigation_strategy_t strategy = s->strategy;
    float gx = s->goal_x, gy = s->goal_y;
    memset(s, 0, sizeof(*s));
    s->strategy = strategy; s->goal_x = gx; s->goal_y = gy;
    s->trigger = "none";
    s->detail = "idle";
}

void navigation_auto_configure(navigation_auto_t *s, navigation_strategy_t strategy,
                               float goal_x, float goal_y)
{
    s->strategy = strategy; s->goal_x = goal_x; s->goal_y = goal_y;
    navigation_auto_reset(s);
}

const char *navigation_phase_name(navigation_phase_t phase)
{
    static const char *names[] = {"cruise", "brake", "reverse", "turn", "recheck", "blocked", "wiggle"};
    return (unsigned)phase < sizeof(names)/sizeof(names[0]) ? names[phase] : "blocked";
}

const char *navigation_strategy_name(navigation_strategy_t strategy)
{
    static const char *names[] = {"explore", "wall_left", "wall_right", "goal", "path"};
    return (unsigned)strategy < sizeof(names)/sizeof(names[0]) ? names[strategy] : "explore";
}

static void phase(navigation_auto_t *s, navigation_phase_t next, uint64_t now)
{
    s->phase = next; s->since_ms = now; s->clear_scans = 0;
}

static void brake(navigation_auto_t *s, navigation_phase_t next, uint64_t now)
{
    phase(s, NAV_BRAKE, now); s->next_phase = next;
}

static unsigned difference(unsigned a, unsigned b)
{
    return a > b ? a - b : b - a;
}

#define NAV_BRAKE_MS            150
#define NAV_MAX_ATTEMPTS        4
#define NAV_BLOCKED_WAIT_MS     3000
#define NAV_TURN_MIN_MS         200
#define NAV_STEER_DEADBAND_DEG  4.0f
#define NAV_SPIN_DEG            70.0f
#define NAV_PROGRESS_WINDOW_MS  3000
#define NAV_PROGRESS_MIN_MM     30
#define NAV_PROGRESS_MAX_MM     3000
#define NAV_POSE_PROGRESS_MM    60.0f
#define NAV_POSE_PROGRESS_RAD   (5.0f * DEG2RAD)
#define NAV_SCORE_CAP_MM        3000
#define NAV_SETTLED_MS          1500
#define NAV_WALL_LOOKAHEAD_MM   400.0f
#define NAV_WALL_GAIN           0.12f   /* deg heading per mm distance error */
#define NAV_WALL_CORNER_MS      4000
#define NAV_GOAL_RADIUS_MM      250.0f
#define NAV_MLINE_MM            150.0f
#define NAV_ROUTE_WAIT_MS       3000    /* no map path that long: unreachable */
#define NAV_ROUTE_MATCH_MM      400.0f  /* planned goal belongs to this goal */
/* Tight spots: after NAV_MAX_ATTEMPTS normal escapes, short pulses with
   bare-body clearances (reverse, rotate, forward, rotate). */
#define NAV_WIGGLE_CYCLES       8
#define NAV_WIGGLE_CLEAR_MM     40
#define NAV_WIGGLE_SWING_MM     15
#define NAV_WIGGLE_DRIVE_MS     350
#define NAV_WIGGLE_TURN_MS      450
#define NAV_WIGGLE_PAUSE_MS     150

static int sign(int value) { return (value > 0) - (value < 0); }

static long capped(uint16_t value) { return value < NAV_SCORE_CAP_MM ? value : NAV_SCORE_CAP_MM; }

static float wrap_pi(float a)
{
    while (a > PI_F) a -= 2 * PI_F;
    while (a < -PI_F) a += 2 * PI_F;
    return a;
}

/* Best free heading: long corridors, little steering, stable choice,
   measured space preferred, optional SLAM bonus for unexplored space. */
static int best_heading(const navigation_view_t *v, unsigned threshold, int previous, bool explore)
{
    int best = -1;
    long best_score = LONG_MIN;
    for (int k = 0; k < NAV_HEADINGS; k++) {
        if (v->free_mm[k] <= threshold) continue;
        int h = navigation_heading_deg(k);
        long score = capped(v->free_mm[k]) - 6L * abs(h) - 3L * abs(h - previous);
        if (v->uncertain_mask & (1u << k)) score -= 800;
        if (explore && v->pose_valid) score += v->explore_bonus[k];
        if (score > best_score) { best_score = score; best = k; }
    }
    return best;
}

static int open_side(const navigation_view_t *v)
{
    long left = 0, right = 0;
    for (int k = 0; k < NAV_CENTER; k++) left += capped(v->free_mm[k]);
    for (int k = NAV_CENTER + 1; k < NAV_HEADINGS; k++) right += capped(v->free_mm[k]);
    if (left == right) return v->left_mm >= v->right_mm ? -1 : 1;
    return left > right ? -1 : 1;
}

/* Proportional steering toward a robot-frame heading error (deg, + = right).
   Speed follows the free distance ahead; large errors rotate in place. The
   turn motor only moves above steer_min_percent, so output starts there. */
static navigation_plan_t steer(const motion_settings_t *cfg, float error, unsigned free_ahead,
                               bool uncertain, const char *reason)
{
    navigation_plan_t p = {.reason = reason};
    float magnitude = clampf(fabsf(error) / 60.0f, 0, 1);
    if (fabsf(error) >= NAV_STEER_DEADBAND_DEG) {
        int turn = cfg->steer_min_percent +
            (int)lroundf((cfg->auto_turn_percent - cfg->steer_min_percent) * magnitude);
        p.turn = error > 0 ? turn : -turn;
    }
    if (fabsf(error) > NAV_SPIN_DEG) return p;
    unsigned stop = cfg->auto_stop_mm + cfg->resume_margin_mm;
    unsigned full = (unsigned)cfg->auto_slow_mm * 2;
    if (full <= stop) full = stop + 1;
    float room = free_ahead <= stop ? 0 : clampf((float)(free_ahead - stop) / (float)(full - stop), 0, 1);
    int span = cfg->auto_drive_percent - cfg->auto_slow_percent;
    p.drive = cfg->auto_slow_percent + (int)lroundf(span * room * (1.0f - 0.5f * magnitude));
    if (uncertain || free_ahead < (unsigned)cfg->auto_slow_mm) p.drive = cfg->auto_slow_percent;
    return p;
}

/* No odometry: SLAM pose change (or, without a pose, changing distances)
   proves travel. Otherwise the tracks slip or something below the scan
   plane blocks the robot. */
static bool stalled(navigation_auto_t *s, const navigation_view_t *v, int ramp_ms, uint64_t now)
{
    if (now - s->cruise_since_ms < (uint64_t)ramp_ms + 500) return false;
    uint16_t sample[3] = {v->front_mm, v->rear_mm,
        (v->uncertain_mask & (1u << NAV_CENTER)) ? 0 : v->free_mm[NAV_CENTER]};
    if (!s->snapshot_valid) {
        memcpy(s->snapshot, sample, sizeof(sample));
        s->progress_x = v->pose_x; s->progress_y = v->pose_y; s->progress_theta = v->pose_theta;
        s->progress_ms = now;
        s->snapshot_valid = true;
        return false;
    }
    if (now - s->progress_ms < NAV_PROGRESS_WINDOW_MS) return false;
    unsigned judged = 0, moved = 0;
    if (v->pose_valid) {
        judged = 1;
        moved = hypotf(v->pose_x - s->progress_x, v->pose_y - s->progress_y) >= NAV_POSE_PROGRESS_MM ||
                fabsf(wrap_pi(v->pose_theta - s->progress_theta)) >= NAV_POSE_PROGRESS_RAD;
    } else {
        for (int i = 0; i < 3; i++) {
            if (!sample[i] || !s->snapshot[i] || sample[i] >= NAV_PROGRESS_MAX_MM ||
                s->snapshot[i] >= NAV_PROGRESS_MAX_MM) continue;
            judged++;
            if (difference(sample[i], s->snapshot[i]) >= NAV_PROGRESS_MIN_MM) moved++;
        }
    }
    memcpy(s->snapshot, sample, sizeof(sample));
    s->progress_x = v->pose_x; s->progress_y = v->pose_y; s->progress_theta = v->pose_theta;
    s->progress_ms = now;
    if (!judged || moved) s->attempts = s->wiggle_cycles = 0;   /* travel ends an escape series */
    return judged && !moved;
}

static navigation_plan_t block(navigation_auto_t *s, const navigation_view_t *v, uint64_t now)
{
    phase(s, NAV_BLOCKED, now);
    s->blocked_front = v->front_mm; s->blocked_rear = v->rear_mm;
    s->blocked_nearest = v->nearest_mm;
    return (navigation_plan_t){.reason = "escape_exhausted"};
}

/* Pulse at the motors' minimum power; the output stage adds a short kick. */
static navigation_plan_t wiggle_pulse(const navigation_auto_t *s, const motion_settings_t *cfg)
{
    return (navigation_plan_t){.drive = s->wiggle_drive * cfg->drive_min_percent,
        .turn = s->wiggle_turn * cfg->turn_min_percent, .reason = "wiggle", .tight = true};
}

static navigation_plan_t wiggle_step(navigation_auto_t *s, const navigation_view_t *v,
    const motion_settings_t *cfg, uint64_t now, bool forward)
{
    navigation_plan_t p = {.reason = "wiggle_pause", .tight = true};
    bool back = v->tight_rear_mm > NAV_WIGGLE_CLEAR_MM;
    bool ahead = v->tight_front_mm > NAV_WIGGLE_CLEAR_MM;
    bool spin = v->rotation_known && v->swing_mm > NAV_WIGGLE_SWING_MM;
    uint64_t elapsed = now - s->since_ms;
    if (s->wiggle_drive || s->wiggle_turn) {
        /* Clearances are re-checked on every tick of a pulse. */
        bool safe = s->wiggle_drive > 0 ? ahead : s->wiggle_drive < 0 ? back : spin;
        unsigned limit = s->wiggle_turn ? NAV_WIGGLE_TURN_MS : NAV_WIGGLE_DRIVE_MS;
        if (safe && elapsed < limit) return wiggle_pulse(s, cfg);
        s->wiggle_drive = s->wiggle_turn = 0;
        s->since_ms = now;
        return p;
    }
    if (elapsed < NAV_WIGGLE_PAUSE_MS) return p;
    if (forward) {                       /* normal way ahead: back to cruising */
        s->attempts = 0;
        phase(s, NAV_RECHECK, now);
        return p;
    }
    for (int tries = 0; tries < 4; tries++) {
        unsigned slot = s->wiggle_slot;
        if (++s->wiggle_slot == 4) {
            /* One round done: rotate the other way in the next round. */
            s->wiggle_slot = 0;
            s->turn_direction = -s->turn_direction;
            if (++s->wiggle_cycles >= NAV_WIGGLE_CYCLES) return block(s, v, now);
        }
        if (slot == 0 && back) s->wiggle_drive = -1;
        else if (slot == 2 && ahead) s->wiggle_drive = 1;
        else if ((slot == 1 || slot == 3) && spin) s->wiggle_turn = s->turn_direction;
        else continue;
        s->since_ms = now;
        return wiggle_pulse(s, cfg);
    }
    return block(s, v, now);
}

/* preferred: forced rotation direction (wall corners, Bug2 hit), 0 = auto.
   Rotations start immediately; only a drive reversal gets a brake pause. */
static navigation_plan_t start_maneuver(navigation_auto_t *s, const navigation_view_t *v,
    const motion_settings_t *cfg, int best, bool rotate, bool rear, bool stuck, uint64_t now,
    const char *trigger, int preferred)
{
    navigation_plan_t p = {.reason = "escape_brake"};
    s->cruising = false;
    if (s->attempts >= NAV_MAX_ATTEMPTS) {
        if (s->wiggle_cycles >= NAV_WIGGLE_CYCLES) return block(s, v, now);
        /* Normal escapes failed: wiggle out with tight clearances. */
        s->wiggle_cycles++;
        s->wiggle_slot = 0;
        s->wiggle_drive = s->wiggle_turn = 0;
        if (!s->turn_direction) s->turn_direction = open_side(v);
        s->trigger = "tight_spot";
        phase(s, NAV_WIGGLE, now);
        s->since_ms = now - NAV_WIGGLE_PAUSE_MS;
        p.reason = "wiggle_pause";
        p.tight = true;
        return p;
    }
    int gap = best >= 0 ? sign(navigation_heading_deg(best)) : 0;
    if (rotate && !stuck && (preferred || gap)) {
        s->turn_direction = preferred ? preferred : gap;
        s->attempts++;
        s->trigger = preferred ? trigger : "turn_to_gap";
        phase(s, NAV_TURN, now);
        p.turn = s->turn_direction * cfg->auto_turn_percent;
        p.reason = "escape_turn";
        return p;
    }
    if (!rear && !rotate) {
        phase(s, NAV_RECHECK, now);
        p.reason = "escape_no_safe_path";
        return p;
    }
    /* Alternate blind escapes so a static trap is explored on both sides. */
    int side = preferred ? preferred : gap ? gap : open_side(v);
    s->turn_direction = (!gap && !preferred && s->attempts % 2 == 1 && s->turn_direction)
                        ? -s->turn_direction : side;
    s->attempts++;
    s->trigger = trigger;
    if (rear) brake(s, NAV_REVERSE, now);
    else {
        phase(s, NAV_TURN, now);
        p.turn = s->turn_direction * cfg->auto_turn_percent;
        p.reason = "escape_turn";
    }
    return p;
}

typedef struct {
    unsigned threshold, center;
    int best;
    bool forward, rear, rotate, uncertain;
} context_t;

static navigation_plan_t explore_step(navigation_auto_t *s, const navigation_view_t *v,
    const motion_settings_t *cfg, uint64_t now, const context_t *c)
{
    s->detail = "explore";
    if (!c->forward)
        return start_maneuver(s, v, cfg, c->best, c->rotate, c->rear, false, now, "front_blocked", 0);
    float target = s->heading_filtered;
    return steer(cfg, target, c->center, c->uncertain,
                 fabsf(target) >= NAV_STEER_DEADBAND_DEG ? "cruise_steer" : "cruise");
}

/* Wall following: P-control on the look-ahead distance to a fitted wall
   line, inner corners by rotating away, outer corners by an arc. */
static navigation_plan_t wall_step(navigation_auto_t *s, const navigation_view_t *v,
    const motion_settings_t *cfg, uint64_t now, const context_t *c, int side)
{
    const navigation_wall_t *w = side > 0 ? &v->wall_right : &v->wall_left;
    float ref = (float)cfg->wall_distance_mm;
    unsigned corner = (unsigned)cfg->wall_distance_mm + 100;
    if (corner < c->threshold) corner = c->threshold;
    if (c->center <= corner) {
        s->detail = "wall_corner_inner";
        return start_maneuver(s, v, cfg, c->best, c->rotate, c->rear, false, now,
                              "wall_inner_corner", -side);
    }
    if (w->valid && w->distance_mm < ref * 2.5f) {
        s->wall_seen_ms = now;
        s->detail = "wall_follow";
        float look = w->distance_mm + NAV_WALL_LOOKAHEAD_MM * sinf(w->angle_deg * DEG2RAD);
        float error = clampf((look - ref) * NAV_WALL_GAIN, -50, 50);
        return steer(cfg, side * error, c->center, c->uncertain, "wall_follow");
    }
    if (s->wall_seen_ms && now - s->wall_seen_ms < NAV_WALL_CORNER_MS &&
        v->free_mm[NAV_CENTER + side * 4] > c->threshold) {
        s->detail = "wall_corner_outer";
        return steer(cfg, side * 45.0f, c->center, true, "wall_corner");
    }
    s->detail = "wall_search";
    return steer(cfg, s->heading_filtered, c->center, c->uncertain, "wall_search");
}

/* Bug2: follow the M-line (start -> goal); on a blocked line follow the
   obstacle boundary until the line is met again closer to the goal. */
static navigation_plan_t goal_step(navigation_auto_t *s, const navigation_view_t *v,
    const motion_settings_t *cfg, uint64_t now, const context_t *c)
{
    navigation_plan_t p = {.reason = "goal_localization_wait"};
    if (!v->pose_valid) { s->detail = "goal_wait"; return p; }
    float x = v->pose_x, y = v->pose_y;
    if (!s->line_valid) {
        s->line_x = x; s->line_y = y; s->line_valid = true;
        s->goal_phase = NAV_GOAL_SEEK;
    }
    if (s->last_valid) s->travel_mm += hypotf(x - s->last_x, y - s->last_y);
    s->last_x = x; s->last_y = y; s->last_valid = true;
    float dx = s->goal_x - x, dy = s->goal_y - y, dist = hypotf(dx, dy);
    if (dist <= NAV_GOAL_RADIUS_MM) {
        s->detail = "goal_reached";
        p.reason = "goal_reached"; p.done = true;
        return p;
    }
    float error = -wrap_pi(atan2f(dy, dx) - v->pose_theta) * RAD2DEG;
    bool ahead = fabsf(error) <= 90.0f;
    int k = (int)lroundf((error + 90.0f) / NAV_HEADING_STEP_DEG);
    if (k < 0) k = 0;
    if (k >= NAV_HEADINGS) k = NAV_HEADINGS - 1;
    unsigned toward = ahead ? v->free_mm[k] : 0;
    bool clear = ahead && toward > c->threshold && (toward >= dist || toward > c->threshold + 300);

    if (s->goal_phase == NAV_GOAL_BOUNDARY) {
        float lx = s->goal_x - s->line_x, ly = s->goal_y - s->line_y, len = hypotf(lx, ly);
        float off = len > 1 ? fabsf(lx * (y - s->line_y) - ly * (x - s->line_x)) / len : 0;
        if (s->travel_mm > 300 && off < NAV_MLINE_MM && dist < s->hit_distance - 150 &&
            fabsf(error) <= 60 && clear) {
            s->goal_phase = NAV_GOAL_SEEK;
            s->trigger = "goal_leave";
        } else if (s->travel_mm > 1500 && hypotf(x - s->hit_x, y - s->hit_y) < 250) {
            s->detail = "goal_unreachable";
            p.reason = "goal_unreachable"; p.done = true;
            return p;
        } else {
            navigation_plan_t w = wall_step(s, v, cfg, now, c, s->wall_side);
            s->detail = "goal_boundary";
            return w;
        }
    }
    s->detail = "goal_seek";
    if (fabsf(error) > 60.0f) {
        if (!c->rotate)
            return start_maneuver(s, v, cfg, c->best, c->rotate, c->rear, false, now, "goal_turn", 0);
        p.turn = error > 0 ? cfg->auto_turn_percent : -cfg->auto_turn_percent;
        p.reason = "goal_turn";
        return p;
    }
    if (!clear || !c->forward) {
        /* Hit point: keep the obstacle on the side away from the open space. */
        s->goal_phase = NAV_GOAL_BOUNDARY;
        s->hit_x = x; s->hit_y = y; s->hit_distance = dist; s->travel_mm = 0;
        s->wall_side = -open_side(v);
        s->wall_seen_ms = 0;
        s->detail = "goal_boundary";
        return start_maneuver(s, v, cfg, c->best, c->rotate, c->rear, false, now, "goal_hit",
                              -s->wall_side);
    }
    unsigned ahead_free = toward < c->center ? toward : c->center;
    return steer(cfg, error, ahead_free, c->uncertain, "goal_seek");
}

/* Go to a point along the SLAM map path: steer straight at the waypoint
   when its corridor is open, otherwise take the best free heading (which
   carries the waypoint pull). The reactive layer still guards every move. */
static navigation_plan_t path_step(navigation_auto_t *s, const navigation_view_t *v,
    const motion_settings_t *cfg, uint64_t now, const context_t *c)
{
    navigation_plan_t p = {.reason = "path_localization_wait"};
    if (!v->pose_valid) { s->detail = "path_wait"; s->route_missing = false; return p; }
    float x = v->pose_x, y = v->pose_y;
    bool route = v->route_valid &&
        hypotf(v->route_goal_x - s->goal_x, v->route_goal_y - s->goal_y) < NAV_ROUTE_MATCH_MM;
    if (hypotf(s->goal_x - x, s->goal_y - y) <= NAV_GOAL_RADIUS_MM ||
        (route && hypotf(v->route_goal_x - x, v->route_goal_y - y) <= NAV_GOAL_RADIUS_MM)) {
        s->detail = "goal_reached";
        p.reason = "goal_reached"; p.done = true;
        return p;
    }
    if (!route) {
        if (!s->route_missing) { s->route_missing = true; s->route_missing_ms = now; }
        if (now - s->route_missing_ms >= NAV_ROUTE_WAIT_MS) {
            s->detail = "path_unreachable";
            p.reason = "path_unreachable"; p.done = true;
            return p;
        }
        s->detail = "path_planning";
        p.reason = "path_planning";
        return p;
    }
    s->route_missing = false;
    float dx = v->route_way_x - x, dy = v->route_way_y - y;
    float error = -wrap_pi(atan2f(dy, dx) - v->pose_theta) * RAD2DEG;
    int side = error > 0 ? 1 : -1;
    if (fabsf(error) > NAV_SPIN_DEG) {
        s->detail = "path_turn";
        if (!c->rotate)
            return start_maneuver(s, v, cfg, c->best, c->rotate, c->rear, false, now, "path_turn", side);
        p.turn = side * cfg->auto_turn_percent;
        p.reason = "path_turn";
        return p;
    }
    if (!c->forward) {
        s->detail = "path_blocked";
        return start_maneuver(s, v, cfg, c->best, c->rotate, c->rear, false, now, "front_blocked",
                              c->rotate ? side : 0);
    }
    int k = (int)lroundf((error + 90.0f) / NAV_HEADING_STEP_DEG);
    if (k < 0) k = 0;
    if (k >= NAV_HEADINGS) k = NAV_HEADINGS - 1;
    unsigned toward = v->free_mm[k];
    float way = hypotf(dx, dy);
    if (toward > c->threshold && (toward + 200.0f >= way || toward > c->threshold + 300)) {
        s->detail = "path_follow";
        return steer(cfg, error, toward < c->center ? toward : c->center, c->uncertain, "path_follow");
    }
    s->detail = "path_avoid";
    return steer(cfg, s->heading_filtered, c->center, c->uncertain, "path_avoid");
}

navigation_plan_t navigation_auto_step(navigation_auto_t *s, const navigation_view_t *v,
    motion_settings_t cfg, uint64_t now, uint32_t sequence)
{
    navigation_plan_t p = {.reason = "escape_brake"};
    bool new_scan = sequence != s->sequence;
    s->sequence = sequence;
    context_t c = {0};
    /* Hysteresis: while already driving forward the robot keeps going down
       to the stop distance; starting again needs the resume margin on top. */
    unsigned resume = cfg.auto_stop_mm + cfg.resume_margin_mm;
    bool driving = s->driving && s->cruising && s->phase == NAV_CRUISE;
    c.threshold = driving ? (unsigned)cfg.auto_stop_mm : resume;
    c.center = v->free_mm[NAV_CENTER];
    c.forward = c.center > c.threshold;
    c.rear = v->rear_free_mm > resume;
    c.rotate = v->rotation_known && v->swing_mm > cfg.auto_rotate_mm;
    c.uncertain = v->uncertain_mask & (1u << NAV_CENTER);
    bool explore = s->strategy == NAV_STRATEGY_EXPLORE || s->strategy == NAV_STRATEGY_WALL_LEFT ||
                   s->strategy == NAV_STRATEGY_WALL_RIGHT || s->strategy == NAV_STRATEGY_PATH;
    c.best = best_heading(v, c.threshold, s->heading_deg, explore);
    if (c.best >= 0) s->heading_deg = navigation_heading_deg(c.best);
    if (new_scan || !s->cruising)
        s->heading_filtered += 0.5f * (s->heading_deg - s->heading_filtered);

    if (s->phase == NAV_WIGGLE) {
        navigation_plan_t w = wiggle_step(s, v, &cfg, now, c.forward);
        if (s->phase == NAV_WIGGLE) return w;
        if (s->phase == NAV_BLOCKED) return w;
    }
    if (s->phase == NAV_BLOCKED) {
        bool changed = new_scan && (difference(v->front_mm, s->blocked_front) > 100 ||
            difference(v->rear_mm, s->blocked_rear) > 100 ||
            difference(v->nearest_mm, s->blocked_nearest) > 100);
        /* Pause, then explore again; the auto time limit bounds retries. */
        if (c.forward || changed || now - s->since_ms >= NAV_BLOCKED_WAIT_MS) {
            s->attempts = s->wiggle_cycles = 0;
            phase(s, NAV_RECHECK, now);
        } else { p.reason = "escape_exhausted"; return p; }
    }
    if (s->phase == NAV_CRUISE || s->phase == NAV_RECHECK) {
        if (!s->cruising) {
            phase(s, NAV_CRUISE, now);
            s->cruising = true;
            s->cruise_since_ms = now;
            s->snapshot_valid = false;
        }
        if (s->attempts && now - s->cruise_since_ms >= NAV_SETTLED_MS && c.forward)
            s->attempts = s->wiggle_cycles = 0;
        if (stalled(s, v, cfg.ramp_ms, now))
            return start_maneuver(s, v, &cfg, c.best, c.rotate, c.rear, true, now, "stuck_no_progress", 0);
        navigation_plan_t plan;
        if (s->strategy == NAV_STRATEGY_GOAL) plan = goal_step(s, v, &cfg, now, &c);
        else if (s->strategy == NAV_STRATEGY_PATH) plan = path_step(s, v, &cfg, now, &c);
        else if (s->strategy == NAV_STRATEGY_WALL_LEFT) plan = wall_step(s, v, &cfg, now, &c, -1);
        else if (s->strategy == NAV_STRATEGY_WALL_RIGHT) plan = wall_step(s, v, &cfg, now, &c, 1);
        else plan = explore_step(s, v, &cfg, now, &c);
        s->driving = s->cruising && plan.drive > 0;
        /* Waiting is not stalling: restart the progress window. */
        if (!plan.drive && !plan.turn) s->snapshot_valid = false;
        return plan;
    }
    if (s->phase == NAV_BRAKE) {
        if (now - s->since_ms < NAV_BRAKE_MS) return p;
        phase(s, s->next_phase, now);
    }
    if (s->phase == NAV_REVERSE) {
        /* The rear corridor is checked on every tick, not only at start. */
        if (v->rear_free_mm <= (unsigned)cfg.auto_stop_mm ||
            now - s->since_ms >= (uint64_t)cfg.reverse_ms) {
            phase(s, c.rotate ? NAV_TURN : NAV_RECHECK, now);
            if (!c.rotate) return p;
            p.turn = s->turn_direction * cfg.auto_turn_percent;
            p.reason = "escape_turn";
            return p;
        }
        p.drive = -cfg.auto_slow_percent;
        p.reason = "escape_reverse";
        return p;
    }
    if (s->phase == NAV_TURN) {
        if (!c.rotate) {
            phase(s, NAV_RECHECK, now);
            p.reason = "escape_rotation_blocked";
            return p;
        }
        /* Leave as soon as one fresh scan shows a usable way ahead. Wall
           modes need room beyond the corner distance, else they re-turn. */
        if (new_scan) {
            bool wall_mode = s->strategy == NAV_STRATEGY_WALL_LEFT ||
                s->strategy == NAV_STRATEGY_WALL_RIGHT ||
                (s->strategy == NAV_STRATEGY_GOAL && s->goal_phase == NAV_GOAL_BOUNDARY);
            unsigned wall_clear = (unsigned)cfg.wall_distance_mm + 150;
            if (wall_clear < (unsigned)cfg.auto_slow_mm) wall_clear = cfg.auto_slow_mm;
            bool aligned = c.best >= 0 && abs(navigation_heading_deg(c.best)) <= 20;
            bool clear = wall_mode ? c.center > wall_clear
                                   : c.center > (unsigned)cfg.auto_slow_mm || (c.forward && aligned);
            s->clear_scans = clear ? s->clear_scans + 1 : 0;
        }
        uint64_t elapsed = now - s->since_ms;
        if ((s->clear_scans >= 1 && elapsed >= NAV_TURN_MIN_MS) ||
            elapsed >= (uint64_t)cfg.turn_ms) {
            phase(s, NAV_RECHECK, now);
            s->heading_filtered = 0;
            return navigation_auto_step(s, v, cfg, now, sequence);
        }
        p.turn = s->turn_direction * cfg.auto_turn_percent;
        p.reason = "escape_turn";
        return p;
    }
    return p;
}
