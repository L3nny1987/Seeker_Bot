#include "route_runner.h"
#include "slam.h"
#include <math.h>
#include <stddef.h>
#include <string.h>
#include <limits.h>

static float distance(route_point_t a, route_point_t b)
{
    return hypotf((float)a.x - b.x, (float)a.y - b.y);
}

const char *route_definition_error(const route_definition_t *d)
{
    if (!d || !d->generation || d->count < 3 || d->count > ROUTE_MAX_POINTS)
        return "route_invalid_points";
    double area = 0;
    float length = 0;
    for (unsigned i = 0; i < d->count; i++) {
        route_point_t a = d->points[i], b = d->points[(i + 1) % d->count];
        if (a.x < -6200 || a.x > 6200 || a.y < -6200 || a.y > 6200)
            return "route_outside_map";
        float edge = distance(a, b);
        if (edge < 250 || edge > 2000) return "route_point_spacing";
        length += edge;
        area += (double)a.x * b.y - (double)b.x * a.y;
    }
    if (length < 1000 || fabs(area) < 180000) return "route_too_small";
    return NULL;
}

static uint16_t read16(const uint8_t *p) { return p[0] | (uint16_t)p[1] << 8; }
static uint32_t read32(const uint8_t *p) { return read16(p) | (uint32_t)read16(p + 2) << 16; }

bool route_packet_decode(const uint8_t *data, unsigned size, route_definition_t *d,
                         uint32_t *token, uint32_t *sequence)
{
    if (!data || !d || !token || !sequence || size < ROUTE_PACKET_HEADER ||
        memcmp(data, "SRTE", 4) || data[4] != 1 || data[5] > 1) return false;
    unsigned count = read16(data + 6);
    if (count < 3 || count > ROUTE_MAX_POINTS || size != ROUTE_PACKET_HEADER + count * 4 ||
        read32(data + 24) || read32(data + 28)) return false;
    memset(d, 0, sizeof(*d));
    d->count = (uint16_t)count;
    d->adaptive = data[5] != 0;
    *token = read32(data + 8);
    *sequence = read32(data + 12);
    d->generation = read32(data + 16);
    d->laps = read32(data + 20);
    for (unsigned i = 0; i < count; i++) {
        d->points[i].x = (int16_t)read16(data + ROUTE_PACKET_HEADER + i * 4);
        d->points[i].y = (int16_t)read16(data + ROUTE_PACKET_HEADER + i * 4 + 2);
    }
    return route_definition_error(d) == NULL;
}

static float segment_distance(float ax, float ay, float bx, float by, float px, float py)
{
    float dx = bx - ax, dy = by - ay, n = dx * dx + dy * dy;
    float t = n > 0 ? ((px - ax) * dx + (py - ay) * dy) / n : 0;
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    return hypotf(px - ax - t * dx, py - ay - t * dy);
}

static float box_distance(float px, float py, float x0, float y0, float x1, float y1)
{
    float dx = px < x0 ? x0 - px : px > x1 ? px - x1 : 0;
    float dy = py < y0 ? y0 - py : py > y1 ? py - y1 : 0;
    return hypotf(dx, dy);
}

/* Exact distance between edge a-b and an axis-aligned cell square. */
static float edge_cell_distance(route_point_t a, route_point_t b, float x0, float y0, float x1, float y1)
{
    float t0 = 0, t1 = 1, dx = (float)b.x - a.x, dy = (float)b.y - a.y;
    float p[4] = {-dx, dx, -dy, dy}, q[4] = {a.x - x0, x1 - a.x, a.y - y0, y1 - a.y};
    bool crosses = true;
    for (int i = 0; i < 4 && crosses; i++) {
        if (p[i] == 0) { crosses = q[i] >= 0; continue; }
        float r = q[i] / p[i];
        if (p[i] < 0) { if (r > t0) t0 = r; } else if (r < t1) t1 = r;
        crosses = t0 <= t1;
    }
    if (crosses) return 0;
    float best = fminf(box_distance(a.x, a.y, x0, y0, x1, y1), box_distance(b.x, b.y, x0, y0, x1, y1));
    float cx[4] = {x0, x1, x0, x1}, cy[4] = {y0, y0, y1, y1};
    for (int i = 0; i < 4; i++) best = fminf(best, segment_distance(a.x, a.y, b.x, b.y, cx[i], cy[i]));
    return best;
}

const char *route_grid_error(const route_definition_t *d, const int8_t *grid,
                             unsigned size, unsigned res, unsigned radius)
{
    if (route_definition_error(d) || !grid || !res || size < 2 || size > 512 || radius > 1000)
        return "route_invalid_points";
    int half = (int)size / 2;
    for (unsigned i = 0; i < d->count; i++) {
        route_point_t a = d->points[i], b = d->points[(i + 1) % d->count];
        unsigned steps = (unsigned)ceilf(distance(a, b) / (res * 0.5f));
        for (unsigned step = 0; step <= steps; step++) {
            float t = (float)step / steps;
            int gx = (int)floorf((a.x + (b.x - a.x) * t) / res) + half;
            int gy = (int)floorf((a.y + (b.y - a.y) * t) / res) + half;
            if (gx < 0 || gy < 0 || gx >= (int)size || gy >= (int)size) return "route_unexplored";
            /* The centre line runs through space the robot has seen free. */
            uint8_t centre = slam_cell_class(grid[gy * size + gx]);
            if (centre >= 2) return "route_obstacle";
            if (centre == 0) return "route_unexplored";
        }
        /* Beside it only mapped obstacles count, measured exactly against the
           corridor the live navigation drives with; the reactive layer handles the rest. */
        int x_lo = (int)floorf(((a.x < b.x ? a.x : b.x) - (float)radius) / res) + half;
        int x_hi = (int)floorf(((a.x > b.x ? a.x : b.x) + (float)radius) / res) + half;
        int y_lo = (int)floorf(((a.y < b.y ? a.y : b.y) - (float)radius) / res) + half;
        int y_hi = (int)floorf(((a.y > b.y ? a.y : b.y) + (float)radius) / res) + half;
        if (x_lo < 0) x_lo = 0;
        if (y_lo < 0) y_lo = 0;
        if (x_hi >= (int)size) x_hi = (int)size - 1;
        if (y_hi >= (int)size) y_hi = (int)size - 1;
        for (int y = y_lo; y <= y_hi; y++)
            for (int x = x_lo; x <= x_hi; x++) {
                if (slam_cell_class(grid[y * size + x]) < 2) continue;
                float x0 = (float)(x - half) * res, y0 = (float)(y - half) * res;
                if (edge_cell_distance(a, b, x0, y0, x0 + res, y0 + res) < radius) return "route_obstacle";
            }
    }
    return NULL;
}

void route_runner_clear(route_runner_t *s) { memset(s, 0, sizeof(*s)); }

void route_runner_start(route_runner_t *s, const route_definition_t *d, uint64_t now)
{
    bool same = s->configured && s->definition.generation == d->generation &&
                s->definition.count == d->count &&
                !memcmp(s->definition.points, d->points, d->count * sizeof(d->points[0]));
    if (!same) route_runner_clear(s);
    s->definition = *d;
    s->configured = s->active = s->approaching = s->segment_clean = s->lap_clean = true;
    s->target = 0;
    s->completed = 0;
    s->healthy_scans = 0;
    s->have_sequence = false;
    s->last_ms = now;
    s->stalled_ms = 0;
    s->best_distance = INFINITY;
}

void route_runner_trouble(route_runner_t *s)
{
    if (!s->active) return;
    if (!s->approaching && s->segment_clean) {
        unsigned score = s->score[s->target];
        s->score[s->target] = score > 30 ? (uint8_t)(score - 30) : 0;
    }
    s->segment_clean = s->lap_clean = false;
}

void route_runner_stop(route_runner_t *s, bool penalize)
{
    if (penalize) route_runner_trouble(s);
    s->active = false;
    s->healthy_scans = 0;
}

static float deviation(route_point_t a, route_point_t b, float x, float y)
{
    return segment_distance(a.x, a.y, b.x, b.y, x, y);
}

/* Beyond the line through b perpendicular to a->b: the waypoint was passed. */
static bool passed(route_point_t a, route_point_t b, float x, float y)
{
    float dx = b.x - a.x, dy = b.y - a.y;
    return (x - b.x) * dx + (y - b.y) * dy >= 0;
}

static route_result_t reach(route_runner_t *s, const char **reason)
{
    if (s->approaching) s->approaching = false;
    else {
        if (s->segment_clean && s->score[s->target] < 100) {
            unsigned score = s->score[s->target] + 20;
            s->score[s->target] = score > 100 ? 100 : (uint8_t)score;
        }
        if (!s->target) {
            if (s->completed < UINT32_MAX) s->completed++;
            if (s->lap_clean && s->learned_laps < UINT32_MAX) s->learned_laps++;
            if (s->definition.laps && s->completed >= s->definition.laps) {
                *reason = "route_complete";
                route_runner_stop(s, false);
                return ROUTE_DONE;
            }
            s->lap_clean = true;
        }
    }
    s->target = (s->target + 1) % s->definition.count;
    s->segment_clean = true;
    s->best_distance = INFINITY;
    s->stalled_ms = 0;
    *reason = "route_waypoint";
    return ROUTE_NEXT;
}

route_result_t route_runner_approach_done(route_runner_t *s, const char **reason)
{
    *reason = "route_wait";
    if (!s->active || !s->approaching) return ROUTE_WAIT;
    return reach(s, reason);
}

route_result_t route_runner_step(route_runner_t *s, route_sample_t v, const char **reason)
{
    *reason = "route_wait";
    if (!s->active) return ROUTE_WAIT;
    uint64_t elapsed = v.now_ms >= s->last_ms ? v.now_ms - s->last_ms : 0;
    s->last_ms = v.now_ms;
    if (v.generation != s->definition.generation) {
        *reason = "route_map_changed";
        route_runner_stop(s, true);
        return ROUTE_FAILED;
    }
    if (!v.healthy || !isfinite(v.x) || !isfinite(v.y) || !isfinite(v.quality)) {
        route_runner_trouble(s);
        s->healthy_scans = 0;
        return ROUTE_WAIT;
    }
    if (s->healthy_scans >= 3) s->stalled_ms += elapsed;
    bool fresh = !s->have_sequence || v.sequence != s->last_sequence;
    if (v.trouble) route_runner_trouble(s);
    if (!fresh) return s->healthy_scans >= 3 ? ROUTE_RUN : ROUTE_WAIT;
    s->have_sequence = true;
    s->last_sequence = v.sequence;
    if (!v.trouble && s->healthy_scans < 3) s->healthy_scans++;
    if (v.quality < 0.75f) {
        /* Low confidence never counts as a successful training observation. */
        if (!s->approaching && s->segment_clean) {
            unsigned score = s->score[s->target];
            s->score[s->target] = score > 30 ? (uint8_t)(score - 30) : 0;
        }
        s->segment_clean = s->lap_clean = false;
    }
    if (s->healthy_scans < 3) return ROUTE_WAIT;
    route_point_t goal = s->definition.points[s->target];
    route_point_t from = s->definition.points[(s->target + s->definition.count - 1) % s->definition.count];
    float remaining = hypotf(v.x - goal.x, v.y - goal.y);
    if (!s->approaching && deviation(from, goal, v.x, v.y) > ROUTE_MAX_DEVIATION_MM) {
        *reason = "route_deviation";
        route_runner_stop(s, true);
        return ROUTE_FAILED;
    }
    if (remaining + 50 < s->best_distance) {
        s->best_distance = remaining;
        s->stalled_ms = 0;
    }
    if (s->stalled_ms >= ROUTE_PROGRESS_TIMEOUT_MS) {
        *reason = "route_no_progress";
        route_runner_stop(s, true);
        return ROUTE_FAILED;
    }
    /* A small capture circle alone makes a coarse drive orbit the waypoint. */
    bool arrived = s->approaching ? remaining <= ROUTE_APPROACH_MM
                                  : remaining <= ROUTE_CAPTURE_MM || passed(from, goal, v.x, v.y);
    if (!arrived) {
        *reason = s->approaching ? "route_approach" : "route_follow";
        return ROUTE_RUN;
    }
    return reach(s, reason);
}

unsigned route_runner_confidence(const route_runner_t *s)
{
    if (!s->configured || !s->definition.count) return 0;
    unsigned total = 0;
    for (unsigned i = 0; i < s->definition.count; i++) total += s->score[i];
    return total / s->definition.count;
}

int route_runner_power(const route_runner_t *s, motion_settings_t cfg,
                       float quality, unsigned clearance, bool uncertain)
{
    /* The output stage lifts any drive command to the motor minimum; report that effective value. */
    int maximum = cfg.auto_drive_percent > cfg.drive_min_percent ? cfg.auto_drive_percent : cfg.drive_min_percent;
    int base = cfg.auto_slow_percent > cfg.drive_min_percent ? cfg.auto_slow_percent : cfg.drive_min_percent;
    if (base > maximum) base = maximum;
    if (!s->definition.adaptive) return maximum;
    if (s->approaching || s->learned_laps < 2 || !s->segment_clean || s->healthy_scans < 3 ||
        !isfinite(quality) || quality < 0.75f || uncertain || clearance <= (unsigned)cfg.auto_slow_mm + 200)
        return base;
    /* Corners stay conservative even after repeated clean laps. */
    unsigned target = s->target, previous = (target + s->definition.count - 1) % s->definition.count;
    unsigned next = (target + 1) % s->definition.count;
    route_point_t a = s->definition.points[previous], b = s->definition.points[target], c = s->definition.points[next];
    float cosine = ((float)(b.x - a.x) * (c.x - b.x) + (float)(b.y - a.y) * (c.y - b.y)) /
                   (distance(a, b) * distance(b, c));
    if (cosine < 0.8660254f) return base; /* direction change > 30 degrees */
    return base + (maximum - base) * s->score[target] / 100;
}
