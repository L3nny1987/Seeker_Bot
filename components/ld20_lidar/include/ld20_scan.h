#pragma once
#include <stdint.h>

#define LD20_POINTS_PER_PACKET 12
#define LD20_SCAN_SECTORS 360

typedef struct {
    uint16_t distance_mm;
    uint8_t intensity;
    uint16_t angle_cdeg;
} ld20_point_t;

typedef struct {
    uint16_t distance_mm[LD20_SCAN_SECTORS];
    uint8_t intensity[LD20_SCAN_SECTORS];
    uint16_t speed_rpm;
    int64_t timestamp_us;               /* actual revolution boundary, not packet reception */
    uint32_t sequence;
    uint16_t valid_sectors;
} ld20_scan_t;
