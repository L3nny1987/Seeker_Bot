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

static float s_sin[360];
static bool s_sin_ready;

static float sin_deg(int a) { return s_sin[((a % 360) + 360) % 360]; }
static float cos_deg(int a) { return sin_deg(a + 90); }

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

void navigation_fuse(const uint16_t current[360], const uint16_t *previous, uint16_t out[360])
{
    for (int i = 0; i < 360; i++)
        out[i] = current[i] ? current[i] : previous ? previous[i] : 0;
}

navigation_view_t navigation_analyze(const uint16_t d[360], int front, bool clockwise,
                                     int robot_width_mm)
{
    if (!s_sin_ready) {
        for (int i = 0; i < 360; i++) s_sin[i] = sinf(i * 3.14159265f / 180.0f);
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
    int half = robot_width_mm / 2;
    for (int k = 0; k < NAV_HEADINGS; k++) {
        bool uncertain;
        v.free_mm[k] = corridor(r, navigation_heading_deg(k), half, &uncertain);
        if (uncertain) v.uncertain_mask |= 1u << k;
    }
    v.rear_free_mm = corridor(r, 180, half, &v.rear_uncertain);
    return v;
}

void navigation_auto_reset(navigation_auto_t *s)
{
    memset(s, 0, sizeof(*s));
    s->trigger = "none";
}

const char *navigation_phase_name(navigation_phase_t phase)
{
    static const char *names[] = {"cruise", "brake", "reverse", "turn", "recheck", "blocked"};
    return (unsigned)phase < sizeof(names)/sizeof(names[0]) ? names[phase] : "blocked";
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
#define NAV_STEER_ON_DEG        20
#define NAV_STEER_OFF_DEG       10
#define NAV_STEER_HOLD_MS       300
#define NAV_PROGRESS_WINDOW_MS  3000
#define NAV_PROGRESS_MIN_MM     30
#define NAV_PROGRESS_MAX_MM     3000
#define NAV_SCORE_CAP_MM        3000

static int sign(int value) { return (value > 0) - (value < 0); }

static long capped(uint16_t value) { return value < NAV_SCORE_CAP_MM ? value : NAV_SCORE_CAP_MM; }

/* Best free heading: long corridors, little steering, stable choice,
   measured space preferred over unmeasured space. -1 = none free. */
static int best_heading(const navigation_view_t *v, unsigned threshold, int previous)
{
    int best = -1;
    long best_score = LONG_MIN;
    for (int k = 0; k < NAV_HEADINGS; k++) {
        if (v->free_mm[k] <= threshold) continue;
        int h = navigation_heading_deg(k);
        long score = capped(v->free_mm[k]) - 6L * abs(h) - 3L * abs(h - previous);
        if (v->uncertain_mask & (1u << k)) score -= 800;
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

/* No odometry: if front/rear distances do not change while driving, the
   tracks are slipping or something below the scan plane blocks the robot. */
static bool stalled(navigation_auto_t *s, const navigation_view_t *v, int ramp_ms, uint64_t now)
{
    if (now - s->cruise_since_ms < (uint64_t)ramp_ms + 500) return false;
    uint16_t sample[3] = {v->front_mm, v->rear_mm,
        (v->uncertain_mask & (1u << NAV_CENTER)) ? 0 : v->free_mm[NAV_CENTER]};
    if (!s->snapshot_valid) {
        memcpy(s->snapshot, sample, sizeof(sample));
        s->progress_ms = now;
        s->snapshot_valid = true;
        return false;
    }
    if (now - s->progress_ms < NAV_PROGRESS_WINDOW_MS) return false;
    unsigned judged = 0, moved = 0;
    for (int i = 0; i < 3; i++) {
        if (!sample[i] || !s->snapshot[i] || sample[i] >= NAV_PROGRESS_MAX_MM ||
            s->snapshot[i] >= NAV_PROGRESS_MAX_MM) continue;
        judged++;
        if (difference(sample[i], s->snapshot[i]) >= NAV_PROGRESS_MIN_MM) moved++;
    }
    memcpy(s->snapshot, sample, sizeof(sample));
    s->progress_ms = now;
    if (!judged || moved) s->attempts = 0;   /* sustained travel ends an escape series */
    return judged && !moved;
}

static navigation_plan_t start_maneuver(navigation_auto_t *s, const navigation_view_t *v,
    int best, bool rotate, bool rear, bool stuck, uint64_t now, const char *trigger)
{
    navigation_plan_t p = {.reason = "escape_brake"};
    s->cruising = false;
    s->steer = 0;
    if (s->attempts >= NAV_MAX_ATTEMPTS) {
        phase(s, NAV_BLOCKED, now);
        s->blocked_front = v->front_mm; s->blocked_rear = v->rear_mm;
        s->blocked_nearest = v->nearest_mm;
        p.reason = "escape_exhausted";
        return p;
    }
    int gap = best >= 0 ? sign(navigation_heading_deg(best)) : 0;
    if (gap && rotate && !stuck) {
        /* Free corridor to the side: rotate in place toward it. */
        s->turn_direction = gap;
        s->attempts++;
        s->trigger = "turn_to_gap";
        brake(s, NAV_TURN, now);
        return p;
    }
    if (!rear && !rotate) {
        phase(s, NAV_RECHECK, now);
        p.reason = "escape_no_safe_path";
        return p;
    }
    /* Alternate blind escapes so a static trap is explored on both sides. */
    int side = gap ? gap : open_side(v);
    s->turn_direction = (!gap && s->attempts % 2 == 1 && s->turn_direction) ? -s->turn_direction : side;
    s->attempts++;
    s->trigger = trigger;
    brake(s, rear ? NAV_REVERSE : NAV_TURN, now);
    return p;
}

navigation_plan_t navigation_auto_step(navigation_auto_t *s, const navigation_view_t *v,
    motion_settings_t cfg, uint64_t now, uint32_t sequence)
{
    navigation_plan_t p = {.reason = "escape_brake"};
    bool new_scan = sequence != s->sequence;
    s->sequence = sequence;
    unsigned threshold = cfg.auto_stop_mm + cfg.resume_margin_mm;
    uint16_t center = v->free_mm[NAV_CENTER];
    bool forward = center > threshold;
    bool rear = v->rear_free_mm > threshold;
    bool rotate = v->rotation_known && v->nearest_mm > cfg.auto_rotate_mm;
    unsigned reverse_ramp = cfg.ramp_ms * cfg.auto_slow_percent / 100;
    unsigned turn_ramp = cfg.ramp_ms * cfg.auto_turn_percent / 100;
    int best = best_heading(v, threshold, s->heading_deg);
    if (best >= 0) s->heading_deg = navigation_heading_deg(best);

    if (s->phase == NAV_BLOCKED) {
        bool changed = new_scan && (difference(v->front_mm, s->blocked_front) > 100 ||
            difference(v->rear_mm, s->blocked_rear) > 100 ||
            difference(v->nearest_mm, s->blocked_nearest) > 100);
        /* Pause, then explore again; the auto time limit bounds retries. */
        if (forward || changed || now - s->since_ms >= NAV_BLOCKED_WAIT_MS) {
            s->attempts = 0;
            phase(s, NAV_RECHECK, now);
        } else { p.reason = "escape_exhausted"; return p; }
    }
    if (s->phase == NAV_CRUISE || s->phase == NAV_RECHECK) {
        if (!forward) return start_maneuver(s, v, best, rotate, rear, false, now, "front_blocked");
        if (!s->cruising) {
            phase(s, NAV_CRUISE, now);
            s->cruising = true;
            s->cruise_since_ms = s->steer_since_ms = now;
            s->snapshot_valid = false;
            s->steer = 0;
        }
        if (stalled(s, v, cfg.ramp_ms, now))
            return start_maneuver(s, v, best, rotate, rear, true, now, "stuck_no_progress");
        bool uncertain = v->uncertain_mask & (1u << NAV_CENTER);
        p.drive = (center < cfg.auto_slow_mm || uncertain) ? cfg.auto_slow_percent : cfg.auto_drive_percent;
        /* Curve toward the most open corridor instead of driving into a wall. */
        int target = s->heading_deg;
        int steer = s->steer;
        if (target >= NAV_STEER_ON_DEG) steer = 1;
        else if (target <= -NAV_STEER_ON_DEG) steer = -1;
        else if (abs(target) <= NAV_STEER_OFF_DEG) steer = 0;
        if (steer != s->steer && now - s->steer_since_ms >= NAV_STEER_HOLD_MS) {
            s->steer = steer;
            s->steer_since_ms = now;
        }
        p.turn = s->steer * cfg.auto_turn_percent;
        p.reason = s->steer ? "cruise_steer" : "cruise";
        return p;
    }
    if (s->phase == NAV_BRAKE) {
        if (now - s->since_ms < NAV_BRAKE_MS) return p;
        phase(s, s->next_phase, now);
    }
    if (s->phase == NAV_REVERSE) {
        /* The rear corridor is checked on every tick, not only at start. */
        if (v->rear_free_mm <= (unsigned)cfg.auto_stop_mm ||
            now - s->since_ms >= (uint64_t)cfg.reverse_ms + reverse_ramp) {
            brake(s, rotate ? NAV_TURN : NAV_RECHECK, now);
            return p;
        }
        p.drive = -cfg.auto_slow_percent;
        p.reason = "escape_reverse";
        return p;
    }
    if (s->phase == NAV_TURN) {
        if (!rotate) {
            brake(s, NAV_RECHECK, now);
            p.reason = "escape_rotation_blocked";
            return p;
        }
        /* Two DISTINCT clear scans are needed to leave the maneuver early;
           repeated manager ticks on one scan are not progress. */
        if (new_scan) s->clear_scans = center > (unsigned)cfg.auto_slow_mm ? s->clear_scans + 1 : 0;
        uint64_t elapsed = now - s->since_ms;
        if ((s->clear_scans >= 2 && elapsed >= turn_ramp + 300) ||
            elapsed >= (uint64_t)cfg.turn_ms + turn_ramp) {
            brake(s, NAV_RECHECK, now);
            return p;
        }
        p.turn = s->turn_direction * cfg.auto_turn_percent;
        p.reason = "escape_turn";
        return p;
    }
    return p;
}