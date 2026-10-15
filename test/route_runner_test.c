#include "route_runner.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static route_definition_t circle(uint32_t laps)
{
    route_definition_t d = {.generation = 9, .laps = laps, .count = 16, .adaptive = true};
    for (unsigned i = 0; i < d.count; i++) {
        float angle = i * 6.28318530718f / d.count;
        d.points[i] = (route_point_t){(int16_t)lroundf(1000 * cosf(angle)), (int16_t)lroundf(1000 * sinf(angle))};
    }
    return d;
}

static uint32_t sequence;
static uint64_t now;
static route_result_t at(route_runner_t *s, route_point_t p, float quality)
{
    const char *reason;
    now += 100;
    return route_runner_step(s, (route_sample_t){.x = p.x, .y = p.y, .quality = quality,
        .sequence = ++sequence, .generation = 9, .now_ms = now, .healthy = true}, &reason);
}

static void approach(route_runner_t *s, const route_definition_t *d)
{
    route_runner_start(s, d, now);
    assert(at(s, d->points[0], 1) == ROUTE_WAIT);
    assert(at(s, d->points[0], 1) == ROUTE_WAIT);
    assert(at(s, d->points[0], 1) == ROUTE_NEXT);
    assert(!s->approaching && s->target == 1 && s->completed == 0);
}

static uint32_t rng = 2463534242u;
static int rnd(int lo, int hi) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return lo + (int)(rng % (uint32_t)(hi - lo + 1)); }

/* Random map + random loops with the firmware verdict, replayed by routes_ui_test.js:
   the dashboard check must decide exactly like route_grid_error(). */
static void corridor_fixture(void)
{
    enum { N = 128, RES = 50, RADIUS = 100 };
    static int8_t grid[N * N];
    static const int8_t logodds[4] = {0, -20, 20, 40};
    uint8_t classes[N * N / 4] = {0};
    for (int gy = 0; gy < N; gy++)
        for (int gx = 0; gx < N; gx++) {
            int x = (gx - N / 2) * RES, y = (gy - N / 2) * RES, r = rnd(0, 999), k = 1;
            if (x * x + y * y > 2600 * 2600) k = 0;
            else if (r < 3) k = 3;
            else if (r < 4) k = 2;
            else if (r < 6) k = 0;
            grid[gy * N + gx] = logodds[k];
            classes[(gy * N + gx) >> 2] |= (uint8_t)(k << (((gy * N + gx) & 3) * 2));
        }
    FILE *out = fopen(".pio/host-tests/route_corridor_fixture.json", "w");
    assert(out);
    fprintf(out, "{\"n\":%d,\"res\":%d,\"radius\":%d,\"cells\":[", N, RES, RADIUS);
    for (unsigned i = 0; i < sizeof(classes); i++) fprintf(out, i ? ",%u" : "%u", classes[i]);
    fprintf(out, "],\"routes\":[");
    unsigned written = 0, verdicts[3] = {0};
    while (written < 300) {
        route_definition_t r = {.generation = 1, .count = (uint16_t)rnd(3, 6)};
        int cx = rnd(-1500, 1500), cy = rnd(-1500, 1500), span = rnd(300, 900);
        for (unsigned i = 0; i < r.count; i++)
            r.points[i] = (route_point_t){(int16_t)(cx + rnd(-span, span)), (int16_t)(cy + rnd(-span, span))};
        if (route_definition_error(&r)) continue;
        const char *code = route_grid_error(&r, grid, N, RES, RADIUS);
        verdicts[!code ? 0 : !strcmp(code, "route_obstacle") ? 1 : 2]++;
        fprintf(out, "%s{\"code\":\"%s\",\"points\":[", written++ ? "," : "", code ? code : "ok");
        for (unsigned i = 0; i < r.count; i++) fprintf(out, i ? ",[%d,%d]" : "[%d,%d]", r.points[i].x, r.points[i].y);
        fprintf(out, "]}");
    }
    fprintf(out, "]}\n");
    fclose(out);
    assert(verdicts[0] && verdicts[1] && verdicts[2]);   /* every verdict is exercised */
}

static void write16(uint8_t *p, uint16_t x) { p[0] = x; p[1] = x >> 8; }
static void write32(uint8_t *p, uint32_t x) { write16(p, x); write16(p + 2, x >> 16); }

int main(void)
{
    route_definition_t d = circle(3);
    assert(!route_definition_error(&d));
    route_runner_t s = {0};
    approach(&s, &d);
    const char *reason;
    unsigned previous_target = s.target;
    assert(route_runner_step(&s, (route_sample_t){.x = d.points[1].x, .y = d.points[1].y,
        .quality = 1, .sequence = sequence, .generation = 9, .now_ms = now, .healthy = true}, &reason) == ROUTE_RUN);
    assert(s.target == previous_target); /* old pose must never advance a waypoint */
    for (unsigned lap = 0; lap < 3; lap++) {
        for (unsigned i = 0; i < d.count; i++) {
            route_result_t result = at(&s, d.points[s.target], 1);
            assert(result == (lap == 2 && i + 1 == d.count ? ROUTE_DONE : ROUTE_NEXT));
            if (i + 1 < d.count) assert(s.completed == lap);
        }
        assert(s.completed == lap + 1);
    }
    assert(!s.active && s.learned_laps == 3 && route_runner_confidence(&s) == 60);
    /* Re-running the same shape/map keeps evidence, editing or map reset does not. */
    approach(&s, &d); assert(s.learned_laps == 3);
    d.points[2].x += 30;
    approach(&s, &d); assert(s.learned_laps == 0 && route_runner_confidence(&s) == 0);
    d = circle(0);
    approach(&s, &d);
    for (unsigned i = 0; i < d.count * 4; i++) assert(at(&s, d.points[s.target], 1) == ROUTE_NEXT);
    assert(s.active && s.completed == 4 && s.learned_laps == 4);
    s.completed = UINT32_MAX;
    for (unsigned i = 0; i < d.count; i++) assert(at(&s, d.points[s.target], 1) == ROUTE_NEXT);
    assert(s.completed == UINT32_MAX && s.active);
    route_runner_stop(&s, true); assert(!s.active);

    approach(&s, &d);
    s.score[s.target] = 80;
    assert(at(&s, d.points[0], 0.6f) == ROUTE_RUN);
    assert(s.score[s.target] == 50 && !s.lap_clean);
    for (int i = 0; i < 10; i++) at(&s, d.points[0], 0.6f);
    assert(s.score[s.target] == 50); /* no 50 Hz penalty avalanche */
    route_runner_stop(&s, false);
    approach(&s, &d);
    uint32_t completed = s.completed;
    now += 50000;
    route_runner_step(&s, (route_sample_t){.generation = 9, .now_ms = now, .healthy = false}, &reason);
    assert(s.completed == completed && s.active && s.healthy_scans == 0);
    assert(at(&s, d.points[0], 1) == ROUTE_WAIT);
    assert(at(&s, d.points[0], 1) == ROUTE_WAIT);
    assert(at(&s, d.points[0], 1) == ROUTE_RUN);
    assert(route_runner_step(&s, (route_sample_t){.generation = 10, .now_ms = now, .healthy = true}, &reason) == ROUTE_FAILED);
    assert(!strcmp(reason, "route_map_changed"));
    approach(&s, &d);
    assert(at(&s, (route_point_t){-2000,-2000}, 1) == ROUTE_FAILED);
    assert(!s.active);

    /* A waypoint counts once it is passed along its edge, even outside the capture
       circle; before that the robot keeps following the edge. */
    approach(&s, &d);
    route_point_t beside_before = {1207, 240}, beside_after = {1146, 547};
    assert(at(&s, beside_before, 1) == ROUTE_RUN && s.target == 1);
    assert(at(&s, beside_after, 1) == ROUTE_NEXT && s.target == 2);
    route_runner_stop(&s, false);

    /* The map path may end on a shifted start point: its arrival starts the first edge. */
    route_runner_start(&s, &d, now);
    for (int i = 0; i < 3; i++) assert(at(&s, (route_point_t){0, 0}, 1) == (i < 2 ? ROUTE_WAIT : ROUTE_RUN));
    assert(route_runner_approach_done(&s, &reason) == ROUTE_NEXT && !s.approaching && s.target == 1);
    assert(route_runner_approach_done(&s, &reason) == ROUTE_WAIT && s.target == 1);
    route_runner_stop(&s, false);

    approach(&s, &d);
    assert(at(&s, d.points[0], 1) == ROUTE_RUN);
    route_result_t result = ROUTE_RUN;
    for (unsigned i = 0; i < 2300 && result == ROUTE_RUN; i++) {
        now += 20;
        if (i % 5 == 0) sequence++;
        result = route_runner_step(&s, (route_sample_t){.x=d.points[0].x,.y=d.points[0].y,
            .quality=1,.sequence=sequence,.generation=9,.now_ms=now,.healthy=true}, &reason);
    }
    assert(result == ROUTE_FAILED && !strcmp(reason, "route_no_progress"));

    motion_settings_t cfg = {.auto_drive_percent=95,.auto_slow_percent=85,.drive_min_percent=85,.auto_slow_mm=300};
    approach(&s, &d); s.learned_laps=5; memset(s.score,100,sizeof(s.score));
    assert(route_runner_power(&s,cfg,1,2000,false)==95);
    assert(route_runner_power(&s,cfg,0.6f,2000,false)==85);
    assert(route_runner_power(&s,cfg,1,2000,true)==85);
    assert(route_runner_power(&s,cfg,1,500,false)==85);
    s.definition.adaptive=false; assert(route_runner_power(&s,cfg,1,2000,true)==95);
    /* Auto power below the motor minimum is valid: report what the output stage drives. */
    motion_settings_t low = {.auto_drive_percent=80,.auto_slow_percent=80,.drive_min_percent=85,.auto_slow_mm=300};
    assert(route_runner_power(&s,low,1,2000,false)==85);
    s.definition.adaptive=true; assert(route_runner_power(&s,low,1,2000,false)==85);

    static int8_t grid[256*256]; memset(grid,-20,sizeof(grid));
    assert(!route_grid_error(&d,grid,256,50,100));
    /* (148,128) lies on the centre line at the route point (1000,0). */
    grid[128*256+148]=30; assert(!strcmp(route_grid_error(&d,grid,256,50,100),"route_obstacle"));
    grid[128*256+148]=12; assert(!strcmp(route_grid_error(&d,grid,256,50,100),"route_obstacle"));
    grid[128*256+148]=0; assert(!strcmp(route_grid_error(&d,grid,256,50,100),"route_unexplored"));
    grid[128*256+148]=-20;
    /* Beside the line the exact gap to the cell square counts: cell (149,128)
       spans x 1050..1100 mm, 50 mm from the corner (1000,0); (151,128) is 150 mm away.
       Unknown space beside the line is acceptable, a mapped obstacle is not. */
    grid[128*256+149]=0; assert(!route_grid_error(&d,grid,256,50,100));
    grid[128*256+149]=30; assert(!strcmp(route_grid_error(&d,grid,256,50,100),"route_obstacle"));
    assert(!route_grid_error(&d,grid,256,50,40));
    grid[128*256+149]=-20; grid[128*256+151]=30; assert(!route_grid_error(&d,grid,256,50,100));
    assert(!strcmp(route_grid_error(&d,grid,256,50,160),"route_obstacle"));
    grid[128*256+151]=-20;
    d.points[1]=d.points[0]; assert(route_definition_error(&d));
    d=circle(1); d.points[0].x=6300; assert(route_definition_error(&d));

    uint8_t packet[48]={0}; memcpy(packet,"SRTE",4);packet[4]=1;packet[5]=1;write16(packet+6,4);
    write32(packet+8,7);write32(packet+12,8);write32(packet+16,9);write32(packet+20,0);
    route_point_t square[4]={{-1000,-1000},{1000,-1000},{1000,1000},{-1000,1000}};
    for(unsigned i=0;i<4;i++){write16(packet+32+i*4,(uint16_t)square[i].x);write16(packet+34+i*4,(uint16_t)square[i].y);}
    uint32_t token,serial;
    assert(route_packet_decode(packet,sizeof(packet),&d,&token,&serial));
    assert(token==7&&serial==8&&d.generation==9&&d.laps==0&&d.adaptive&&d.count==4);
    FILE *out=fopen(".pio/host-tests/route_packet_fixture.bin","wb");assert(out);assert(fwrite(packet,1,sizeof(packet),out)==sizeof(packet));fclose(out);
    packet[24]=1;assert(!route_packet_decode(packet,sizeof(packet),&d,&token,&serial));packet[24]=0;
    packet[5]=2;assert(!route_packet_decode(packet,sizeof(packet),&d,&token,&serial));packet[5]=1;
    assert(!route_packet_decode(packet,sizeof(packet)-1,&d,&token,&serial));
    corridor_fixture();
    puts("Route runner: ordered closure, pass-through waypoints, approach hand-off, finite/infinite laps, fresh poses, map binding, confidence, power, corridor and packet checks passed.");
    return 0;
}
