#pragma once
#include <stdbool.h>
#include "ld20_scan.h"

/* Pure assembly layer: an interrupted revolution must never become a scan. */
typedef struct {
    ld20_scan_t work;
    bool synced;
    int previous_angle;
    int64_t last_packet_us, revolution_us;
    uint32_t dropped_scans;
} ld20_scan_builder_t;

void ld20_scan_builder_reset(ld20_scan_builder_t *b);
/* CRC/header verified by the receiver. -1 invalid, 0 assembling, 1 published. */
int ld20_scan_builder_packet(ld20_scan_builder_t *b, const uint8_t packet[47],
                            int64_t packet_end_us, uint8_t min_intensity, ld20_scan_t *out);
