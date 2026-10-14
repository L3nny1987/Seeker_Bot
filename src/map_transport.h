#pragma once
#include <stddef.h>
#include <stdint.h>
#include "slam.h"

static inline uint8_t map_transport_byte(const int8_t *grid, size_t cell)
{
    return slam_cell_class(grid[cell]) | slam_cell_class(grid[cell + 1]) << 2 |
           slam_cell_class(grid[cell + 2]) << 4 | slam_cell_class(grid[cell + 3]) << 6;
}

/* RLE over packed 2-bit map bytes: (repeat count 1..255, byte value).
   Uses the existing HTTP scratch buffer. Zero means use legacy raw encoding;
   noisy maps cannot consume extra heap or overflow the scratch buffer. */
static inline size_t map_transport_rle(const int8_t *grid, size_t cells,
                                      uint8_t *out, size_t capacity)
{
    if (!cells || cells % 4) return 0;
    size_t used = 0;
    uint8_t value = map_transport_byte(grid, 0), count = 1;
    for (size_t i = 4; i <= cells; i += 4) {
        uint8_t next = i < cells ? map_transport_byte(grid, i) : 0;
        if (i < cells && next == value && count < 255) { count++; continue; }
        if (used + 2 > capacity) return 0;
        out[used++] = count;
        out[used++] = value;
        value = next;
        count = 1;
    }
    return used < cells / 4 ? used : 0;
}
