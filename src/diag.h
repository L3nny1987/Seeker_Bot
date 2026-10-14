#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint32_t failures, size, caps, uptime_ms;
} diag_memory_t;

/* Reset diagnostics: log lines and the last robot state live in RTC memory
   (survive panic, watchdog and brownout resets, not a power loss); reset
   reasons with that state are counted in NVS. Call after nvs_flash_init. */
void diag_init(void);
/* Latest robot state, cheap (called every manager period). */
void diag_snapshot(int mode, int drive, int turn, bool online);
/* Plain-text report parts for /api/diag. */
size_t diag_summary(char *out, size_t size);
const char *diag_previous_log(size_t *len);
size_t diag_current_log(char *out, size_t size);
void diag_clear(void);
diag_memory_t diag_memory_status(void);
