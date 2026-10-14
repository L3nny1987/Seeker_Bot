#pragma once

/* SLAM shares internal RAM with Wi-Fi/lwIP. Defaults intended for throughput
   can queue more than the available heap on a weak radio link. */
#define SEEKER_WIFI_STATIC_RX_BUFFERS 6
#define SEEKER_WIFI_DYNAMIC_RX_BUFFERS 8
#define SEEKER_WIFI_DYNAMIC_TX_BUFFERS 8
#define SEEKER_TCP_WINDOW_BYTES 2880
#define SEEKER_HTTP_CLIENTS 4
#define SEEKER_BULK_HEAP_RESERVE 16384
#define SEEKER_BULK_MAX_US 750000
