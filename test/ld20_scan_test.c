#include "../components/ld20_lidar/ld20_scan_builder.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static ld20_scan_builder_t builder;
static ld20_scan_t scan;
static void put16(uint8_t *p, unsigned n) { p[0] = n; p[1] = n >> 8; }

static int feed(unsigned absolute_start, int64_t delay_us, uint16_t distance)
{
    uint8_t p[47] = {0x54, 0x2c};
    put16(p + 2, 2160);                         /* 6 Hz, 360 RPM */
    put16(p + 4, absolute_start % 36000);
    put16(p + 42, (absolute_start + 1100) % 36000);
    for (int i = 0; i < 12; i++) { put16(p + 6 + 3 * i, distance); p[8 + 3 * i] = 100; }
    int64_t received = 1000000 + (int64_t)(absolute_start + 1100) * 10000 / 2160 + 2040 + delay_us;
    return ld20_scan_builder_packet(&builder, p, received, 10, &scan);
}

int main(void)
{
    ld20_scan_builder_reset(&builder);
    unsigned a = 600;                           /* wrap inside a packet */
    while (!scan.sequence) { feed(a, 0, 1000); a += 1200; assert(a < 80000); }
    assert(scan.valid_sectors == 360 && scan.speed_rpm == 360);
    assert(scan.timestamp_us >= 1333332 && scan.timestamp_us <= 1333334);
    for (int i = 0; i < 360; i++) assert(scan.distance_mm[i] == 1000);
    uint32_t sequence = scan.sequence;

    /* An angular gap must discard the partial revolution, never merge old
       returns with the following one. */
    feed(a, 0, 1000); a += 1200;
    a += 6000;
    assert(!feed(a, 0, 2000)); a += 1200;
    unsigned limit = ((a / 36000) + 1) * 36000;
    while (a < limit) { feed(a, 0, 2000); a += 1200; }
    assert(scan.sequence == sequence && builder.dropped_scans > 0);
    while (scan.sequence == sequence) { feed(a, 0, 2000); a += 1200; assert(a < limit + 72000); }
    for (int i = 0; i < 360; i++) assert(scan.distance_mm[i] == 2000);

    /* Clock discontinuity and a CRC/UART reset require a new full turn. */
    sequence = scan.sequence;
    assert(!feed(a, 300000, 3000)); a += 1200;
    ld20_scan_builder_reset(&builder);
    assert(!builder.synced && builder.previous_angle == -1);
    while (scan.sequence == sequence) { feed(a, 300000, 3000); a += 1200; }
    for (int i = 0; i < 360; i++) assert(scan.distance_mm[i] == 3000);
    uint8_t bad[47] = {0};
    assert(ld20_scan_builder_packet(&builder, bad, 3000000, 10, &scan) == -1);
    assert(!builder.synced);
    put16(bad + 2, 2160); put16(bad + 4, 36000); put16(bad + 42, 1000);
    assert(ld20_scan_builder_packet(&builder, bad, 3000000, 10, &scan) == -1);
    puts("LD20: complete revolutions, wrap timestamp, gaps, reset and recovery passed");
}
