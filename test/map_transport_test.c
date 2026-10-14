#include "../src/map_transport.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static int8_t grid[SLAM_SIZE * SLAM_SIZE];
static uint8_t scratch[7170];

static void verify(size_t encoded)
{
    assert(encoded && encoded % 2 == 0);
    size_t byte = 0;
    for (size_t i = 0; i < encoded; i += 2) {
        unsigned run = scratch[i + 1];
        assert(run);
        for (unsigned j = 0; j < run; j++, byte++) {
            assert(byte < sizeof(grid) / 4);
            assert(scratch[i + 2] == map_transport_byte(grid, byte * 4));
        }
    }
    assert(byte == sizeof(grid) / 4);
    assert(scratch[0] == 0xa5 && scratch[7169] == 0xa5);
}

static void put16(uint8_t *p, unsigned n) { p[0] = n; p[1] = n >> 8; }

int main(int argc, char **argv)
{
    memset(scratch, 0xa5, sizeof(scratch));
    size_t n = map_transport_rle(grid, sizeof(grid), scratch + 1, 7168);
    assert(n == 130);                          /* empty full-size map: 16 KB -> 130 bytes */
    verify(n);
    assert(map_transport_rle(grid, sizeof(grid), scratch + 1, 0) == 0);
    assert(map_transport_rle(grid, sizeof(grid), scratch + 1, 1) == 0);
    assert(map_transport_rle(grid, sizeof(grid), scratch + 1, 128) == 0);
    assert(map_transport_rle(grid, 7, scratch + 1, 7168) == 0);

    for (int y = 0; y < SLAM_SIZE; y++) for (int x = 0; x < SLAM_SIZE; x++) {
        int wx = (x - SLAM_ORIGIN) * SLAM_RES_MM, wy = (y - SLAM_ORIGIN) * SLAM_RES_MM;
        if (wx < -1000 || wx > 3000 || wy < -1500 || wy > 1500) continue;
        bool wall = wx < -950 || wx > 2950 || wy < -1450 || wy > 1450 ||
                    (wx > 1200 && wx < 1350 && wy > 0);
        grid[y * SLAM_SIZE + x] = wall ? 80 : -30;
    }
    n = map_transport_rle(grid, sizeof(grid), scratch + 1, 7168);
    assert(n < sizeof(grid) / 16);
    verify(n);
    printf("Map transport: room grid %zu -> %zu bytes, %.1f %% smaller, no extra heap\n",
           sizeof(grid) / 4, n, 100.0 * (1.0 - (double)n / (sizeof(grid) / 4)));

    /* The browser test decodes bytes produced by this actual C encoder. */
    FILE *fixture = fopen(argc > 1 ? argv[1] : ".pio/host-tests/map_rle_fixture.bin", "wb");
    assert(fixture);
    uint8_t header[40] = {'S','M','A','P',2,1};
    put16(header + 6, SLAM_SIZE); put16(header + 8, SLAM_RES_MM);
    put16(header + 10, 2); put16(header + 34, (unsigned)n);
    assert(fwrite(header, 1, sizeof(header), fixture) == sizeof(header));
    assert(fwrite(scratch + 1, 1, n, fixture) == n);
    uint8_t path[8] = {10, 0, 251, 255, 20, 0, 0, 0};
    assert(fwrite(path, 1, sizeof(path), fixture) == sizeof(path));
    fclose(fixture);
    FILE *raw = fopen(".pio/host-tests/map_rle_expected.bin", "wb");
    assert(raw);
    for (size_t i = 0; i < sizeof(grid); i += 4) {
        uint8_t b = map_transport_byte(grid, i);
        assert(fwrite(&b, 1, 1, raw) == 1);
    }
    fclose(raw);

    /* Worst-case noise falls back to v1 without writing past the buffer. */
    static const int8_t levels[] = {-30, 0, 15, 80};
    uint32_t random = 17;
    for (size_t i = 0; i < sizeof(grid); i++) {
        random = random * 1664525u + 1013904223u;
        grid[i] = levels[random >> 30];
    }
    assert(map_transport_rle(grid, sizeof(grid), scratch + 1, 7168) == 0);
    assert(scratch[0] == 0xa5 && scratch[7169] == 0xa5);
    puts("Map transport: exact round-trip, 255-byte runs, short buffers and noisy-map fallback passed");
}
