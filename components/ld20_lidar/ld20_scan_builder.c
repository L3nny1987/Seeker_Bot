#include "ld20_scan_builder.h"
#include <string.h>

static uint16_t le16(const uint8_t *p) { return p[0] | ((uint16_t)p[1] << 8); }

void ld20_scan_builder_reset(ld20_scan_builder_t *b)
{
    if (b->synced) b->dropped_scans++;
    memset(&b->work, 0, sizeof(b->work));
    b->synced = false;
    b->previous_angle = -1;
    b->last_packet_us = b->revolution_us = 0;
}

int ld20_scan_builder_packet(ld20_scan_builder_t *b, const uint8_t p[47],
                            int64_t now, uint8_t min_intensity, ld20_scan_t *out)
{
    unsigned start = le16(p + 4), end = le16(p + 42), speed = le16(p + 2);
    unsigned span = (end + 36000 - start) % 36000;
    if (start >= 36000 || end >= 36000 || speed < 360 || speed > 7200 || !span || span > 4500) {
        ld20_scan_builder_reset(b);
        return -1;
    }
    int64_t period = 360000000LL / speed;
    if (b->last_packet_us && (now <= b->last_packet_us || now - b->last_packet_us > period / 4))
        ld20_scan_builder_reset(b);
    b->last_packet_us = now;
    int ready = 0;
    for (unsigned i = 0; i < LD20_POINTS_PER_PACKET; i++) {
        unsigned offset = span * i / (LD20_POINTS_PER_PACKET - 1);
        unsigned angle = (start + offset) % 36000;
        /* Subtract point acquisition age plus the 47-byte UART transfer.
           At a wrap in the middle of a packet, its later points belong to
           the next revolution and must not delay the gyro reference. */
        int64_t point_us = now - (int64_t)(span - offset) * 10000 / speed - 2040;
        if (b->previous_angle >= 0 && (angle + 36000 - b->previous_angle) % 36000 > 1500) {
            ld20_scan_builder_reset(b);           /* reordered or missing angular chunk */
            b->last_packet_us = now;
        }
        if (b->previous_angle > 30000 && angle < 6000) {
            int64_t boundary = point_us - (int64_t)angle * 10000 / speed;
            int64_t duration = boundary - b->revolution_us;
            if (b->synced && duration >= period / 2 && duration <= period * 3 / 2) {
                b->work.speed_rpm = speed / 6;
                b->work.timestamp_us = boundary;
                b->work.sequence = out->sequence + 1;
                *out = b->work;
                ready = 1;
            } else if (b->synced) b->dropped_scans++;
            memset(&b->work, 0, sizeof(b->work));
            b->synced = true;
            b->revolution_us = boundary;
        }
        b->previous_angle = (int)angle;
        unsigned sector = angle / 100;
        uint16_t distance = le16(p + 6 + i * 3);
        uint8_t intensity = p[8 + i * 3];
        if (distance >= 80 && distance <= 8000 && intensity >= min_intensity) {
            if (!b->work.distance_mm[sector]) b->work.valid_sectors++;
            if (!b->work.distance_mm[sector] || distance < b->work.distance_mm[sector]) {
                b->work.distance_mm[sector] = distance;
                b->work.intensity[sector] = intensity;
            }
        }
    }
    return ready;
}
