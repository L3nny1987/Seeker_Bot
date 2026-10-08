#pragma once
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t disconnects;        /* lost an established connection */
    uint32_t failed_attempts;    /* connection attempts that did not succeed */
    uint32_t connected_s;        /* duration of the current connection */
    uint32_t last_outage_ms;     /* duration of the last outage */
    uint8_t last_reason;         /* wifi_err_reason_t of the last disconnect */
} network_stats_t;

esp_err_t seeker_network_init(void);
network_stats_t seeker_network_stats(void);