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

    static int8_t grid[256*256]; memset(grid,-20,sizeof(grid));
    assert(route_grid_clear(&d,grid,256,50,260));
    grid[128*256+148]=30; assert(!route_grid_clear(&d,grid,256,50,260));
    grid[128*256+148]=0; assert(!route_grid_clear(&d,grid,256,50,260));
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
    puts("Route runner: ordered closure, finite/infinite laps, fresh poses, map binding, confidence, power gates, grid and packet checks passed.");
    return 0;
}
