#include "web_server.h"
#include "robot_manager.h"
#include "ld20_lidar.h"
#include "seeker_pins.h"
#include <math.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "cJSON.h"
#include "esp_http_server.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "dashboard_asset.h"
#include "lwip/sockets.h"
#include "lwip/tcp.h"
#include "esp_wifi.h"
#include "reset_diagnostics.h"
#include "diag.h"
#include "imu.h"
#include "network.h"
#include "network_limits.h"
#include "map_transport.h"
#include "esp_app_desc.h"
#include "esp_app_format.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"

#define OTA_CHUNK    4096
#define OTA_RETRIES  4                  /* recv timeouts (1 s each) per block */
#define OTA_IDLE_US  (30 * 1000000LL)   /* upload session expiry */

static const char *TAG = "web";
static httpd_handle_t s_server;
static uint32_t s_bulk_pauses, s_bulk_errors;

/* The HTTP server runs all handlers on one task, so a single static buffer
   is safe and avoids hundreds of heap allocations per telemetry frame. */
static char s_json[7168];
/* Resumable firmware upload; while it runs, control commands except estop
   are refused. A session expires after OTA_IDLE_US without blocks. */
static struct {
    esp_ota_handle_t handle;
    const esp_partition_t *slot;
    size_t total, written;
    int64_t last_us;
    bool begun, finished;
} s_ota;

static bool ota_running(void)
{
    return s_ota.finished || (s_ota.begun && esp_timer_get_time() - s_ota.last_us < OTA_IDLE_US);
}
typedef struct { size_t used; bool overflow; } writer_t;

static void put(writer_t *w, const char *format, ...)
{
    if (w->overflow) return;
    va_list args;
    va_start(args, format);
    int n = vsnprintf(s_json + w->used, sizeof(s_json) - w->used, format, args);
    va_end(args);
    if (n < 0 || (size_t)n >= sizeof(s_json) - w->used) w->overflow = true;
    else w->used += n;
}

static const char *json_bool(bool value) { return value ? "true" : "false"; }

static const struct { const char *name; size_t offset; } SETTINGS_FIELDS[] = {
    {"manualStopMm", offsetof(motion_settings_t, manual_stop_mm)},
    {"manualRotateMm", offsetof(motion_settings_t, manual_rotate_mm)},
    {"autoStopMm", offsetof(motion_settings_t, auto_stop_mm)},
    {"autoRotateMm", offsetof(motion_settings_t, auto_rotate_mm)},
    {"autoSlowMm", offsetof(motion_settings_t, auto_slow_mm)},
    {"autoDrivePercent", offsetof(motion_settings_t, auto_drive_percent)},
    {"autoTurnPercent", offsetof(motion_settings_t, auto_turn_percent)},
    {"autoSlowPercent", offsetof(motion_settings_t, auto_slow_percent)},
    {"rampMs", offsetof(motion_settings_t, ramp_ms)},
    {"reverseMs", offsetof(motion_settings_t, reverse_ms)},
    {"turnMs", offsetof(motion_settings_t, turn_ms)},
    {"resumeMarginMm", offsetof(motion_settings_t, resume_margin_mm)},
    {"robotWidthMm", offsetof(motion_settings_t, robot_width_mm)},
    {"autoLimitS", offsetof(motion_settings_t, auto_limit_s)},
    {"turnRampMs", offsetof(motion_settings_t, turn_ramp_ms)},
    {"steerMinPercent", offsetof(motion_settings_t, steer_min_percent)},
    {"wallDistanceMm", offsetof(motion_settings_t, wall_distance_mm)},
    {"driveMinPercent", offsetof(motion_settings_t, drive_min_percent)},
    {"turnMinPercent", offsetof(motion_settings_t, turn_min_percent)},
    {"driveKickMs", offsetof(motion_settings_t, drive_kick_ms)},
    {"turnKickMs", offsetof(motion_settings_t, turn_kick_ms)},
};
#define SETTINGS_COUNT (sizeof(SETTINGS_FIELDS) / sizeof(SETTINGS_FIELDS[0]))

static esp_err_t socket_open(httpd_handle_t server, int socket)
{
    (void)server;
    /* Small control replies must not wait for Nagle/delayed ACK batching. */
    int enabled = 1;
    return setsockopt(socket, IPPROTO_TCP, TCP_NODELAY, &enabled, sizeof(enabled)) == 0
           ? ESP_OK : ESP_FAIL;
}

_Static_assert(CONFIG_LWIP_TCP_SND_BUF_DEFAULT <= SEEKER_TCP_WINDOW_BYTES &&
               CONFIG_LWIP_TCP_WND_DEFAULT <= SEEKER_TCP_WINDOW_BYTES,
               "Apply the bounded TCP windows from sdkconfig.defaults to the existing sdkconfig");

static bool bulk_allowed(void)
{
    return heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) >= SEEKER_BULK_HEAP_RESERVE;
}

static esp_err_t bulk_unavailable(httpd_req_t *req)
{
    s_bulk_pauses++;
    httpd_resp_set_status(req, "503 Service Unavailable");
    httpd_resp_set_hdr(req, "Retry-After", "3");
    return httpd_resp_send(req, "Network memory busy; retry shortly", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t bulk_chunk(httpd_req_t *req, const void *data, size_t size, int64_t deadline)
{
    if (!bulk_allowed()) { s_bulk_pauses++; return ESP_ERR_TIMEOUT; }
    if (esp_timer_get_time() >= deadline) { s_bulk_errors++; return ESP_ERR_TIMEOUT; }
    esp_err_t err = httpd_resp_send_chunk(req, data, size);
    if (err != ESP_OK) s_bulk_errors++;
    return err;
}

static void headers(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "X-Content-Type-Options", "nosniff");
    httpd_resp_set_hdr(req, "X-Frame-Options", "DENY");
    httpd_resp_set_hdr(req, "Referrer-Policy", "no-referrer");
    httpd_resp_set_hdr(req, "Content-Security-Policy",
        "default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'; frame-ancestors 'none'; connect-src 'self'; object-src 'none'");
}

static esp_err_t json_reply(httpd_req_t *req, const writer_t *w)
{
    if (w->overflow) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "response_too_large");
    headers(req);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, s_json, w->used);
}

static esp_err_t text_reply(httpd_req_t *req, const char *status, const char *json)
{
    if (status) httpd_resp_set_status(req, status);
    headers(req);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t index_handler(httpd_req_t *req)
{
    headers(req);
    if (!bulk_allowed()) return bulk_unavailable(req);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_set_hdr(req, "Vary", "Accept-Encoding");
    /* The uncompressed CSS/HTML previously exceeded the small TCP send
       window on a weak Wi-Fi link. Stream a compact asset in bounded chunks. */
    int64_t deadline = esp_timer_get_time() + 3000000;
    for (size_t offset = 0; offset < sizeof(dashboard_gzip); offset += 1024) {
        size_t count = sizeof(dashboard_gzip) - offset;
        if (count > 1024) count = 1024;
        esp_err_t err = bulk_chunk(req, dashboard_gzip + offset, count, deadline);
        if (err != ESP_OK) return err;
    }
    return httpd_resp_send_chunk(req, NULL, 0);
}

static void scan_json(writer_t *w)
{
    static ld20_scan_t scan;
    bool fresh = ld20_lidar_get_scan(&scan, 0) == ESP_OK;
    put(w, "{\"fresh\":%s,\"rpm\":%u,\"sequence\":%lu,\"ageMs\":%lu,\"distance\":[",
        json_bool(fresh), (unsigned)scan.speed_rpm, (unsigned long)scan.sequence,
        (unsigned long)(scan.timestamp_us ? (esp_timer_get_time() - scan.timestamp_us) / 1000 : UINT32_MAX));
    for (int i = 0; i < 360; i++) {
        int raw = (SEEKER_LIDAR_FRONT_DEG + (SEEKER_LIDAR_CLOCKWISE ? i : -i) + 720) % 360;
        put(w, i ? ",%u" : "%u", (unsigned)scan.distance_mm[raw]);
    }
    put(w, "]}");
}

static esp_err_t status_handler(httpd_req_t *req)
{
    robot_status_t s = robot_manager_status();
    ld20_stats_t stats = ld20_lidar_get_stats();
    writer_t w = {0};
    char sample[sizeof(stats.sample) * 2 + 1] = {0};
    for (unsigned i = 0; i < stats.sample_len && i < sizeof(stats.sample); i++)
        snprintf(sample + i * 2, 3, "%02X", stats.sample[i]);
    const char *diagnostic = !stats.rx_bytes ? "no_uart_bytes" :
        !stats.headers ? "no_ld_header" : !stats.packets ? "no_valid_packets" :
        !s.scan_sequence ? "waiting_for_revolution" :
        !s.lidar_healthy ? "stale_or_incomplete_scan" : "healthy";
    put(&w, "{\"version\":\"%s\",\"mode\":\"%s\",\"reason\":\"%s\",\"network\":%s,"
        "\"controller\":%s,\"linkOk\":%s,\"heartbeatAgeMs\":%lu,\"commissioned\":%s,"
        "\"lidarHealthy\":%s,\"drive\":%d,\"turn\":%d,\"coverage\":%u,\"scanAgeMs\":%lu,"
        "\"scanSequence\":%lu,",
        SEEKER_BOT_VERSION, robot_mode_name(s.mode), s.reason, json_bool(s.network),
        json_bool(s.controller), json_bool(s.link_ok), (unsigned long)s.heartbeat_age_ms,
        json_bool(s.commissioned), json_bool(s.lidar_healthy), s.drive, s.turn,
        (unsigned)s.coverage, (unsigned long)s.scan_age_ms, (unsigned long)s.scan_sequence);
    put(&w, "\"frontMm\":%u,\"rearMm\":%u,\"leftMm\":%u,\"rightMm\":%u,\"nearestMm\":%u,"
        "\"frontKnown\":%s,\"rearKnown\":%s,\"rotationKnown\":%s,",
        (unsigned)s.view.front_mm, (unsigned)s.view.rear_mm, (unsigned)s.view.left_mm,
        (unsigned)s.view.right_mm, (unsigned)s.view.nearest_mm, json_bool(s.view.front_known),
        json_bool(s.view.rear_known), json_bool(s.view.rotation_known));
    put(&w, "\"autoPhase\":\"%s\",\"autoTrigger\":\"%s\",\"escapeAttempts\":%u,"
        "\"navHeading\":%d,\"navCenterMm\":%u,\"autoRemainingS\":%lu,",
        s.auto_phase, s.auto_trigger ? s.auto_trigger : "none", s.escape_attempts, s.nav_heading,
        (unsigned)s.view.free_mm[NAV_CENTER], (unsigned long)s.auto_remaining_s);
    put(&w, "\"clearRearMm\":%u,\"swingMm\":%u,\"tightFrontMm\":%u,\"tightRearMm\":%u,"
        "\"bodyFrontMm\":%d,\"bodyRearMm\":%d,\"bodyHalfWidthMm\":%d,",
        (unsigned)s.view.rear_free_mm, (unsigned)s.view.swing_mm, (unsigned)s.view.tight_front_mm,
        (unsigned)s.view.tight_rear_mm, SEEKER_BODY_FRONT_MM, SEEKER_BODY_REAR_MM,
        SEEKER_BODY_HALF_WIDTH_MM);
    put(&w, "\"exploreTarget\":%s,\"exploreFrontier\":%s,\"exploreX\":%.0f,\"exploreY\":%.0f,"
        "\"exploreWayX\":%.0f,\"exploreWayY\":%.0f,\"exploreAvoided\":%u,\"exploreSteps\":%u,",
        json_bool(s.map.target.valid), json_bool(s.map.target.frontier), s.map.target.x,
        s.map.target.y, s.map.target.way_x, s.map.target.way_y, (unsigned)s.map.avoided,
        (unsigned)s.map.target.steps);
    put(&w, "\"strategy\":\"%s\",\"navDetail\":\"%s\",\"goalSet\":%s,\"goalX\":%.0f,\"goalY\":%.0f,"
        "\"wallLeftMm\":%u,\"wallLeftDeg\":%d,\"wallRightMm\":%u,\"wallRightDeg\":%d,",
        s.strategy, s.nav_detail, json_bool(s.goal_set), s.goal_x, s.goal_y,
        s.view.wall_left.valid ? (unsigned)s.view.wall_left.distance_mm : 0u, s.view.wall_left.angle_deg,
        s.view.wall_right.valid ? (unsigned)s.view.wall_right.distance_mm : 0u, s.view.wall_right.angle_deg);
    put(&w, "\"mapGeneration\":%lu,\"routeRevision\":%lu,\"routeActive\":%s,\"routeConfigured\":%s,"
        "\"routeApproaching\":%s,\"routeAdaptive\":%s,\"routeLaps\":%lu,\"routeCompleted\":%lu,"
        "\"routeLearnedLaps\":%lu,\"routePoints\":%u,\"routeTarget\":%u,\"routeConfidence\":%u,\"routePower\":%d,"
        "\"routeCorridorMm\":%u,",
        (unsigned long)s.map.generation, (unsigned long)s.route_revision, json_bool(s.route_active),
        json_bool(s.route_configured), json_bool(s.route_approaching), json_bool(s.route_adaptive),
        (unsigned long)s.route_laps, (unsigned long)s.route_completed, (unsigned long)s.route_learned_laps,
        s.route_points, s.route_target, s.route_confidence, s.route_power, s.route_corridor_mm);
    put(&w, "\"poseValid\":%s,\"slamLost\":%s,\"poseX\":%.0f,\"poseY\":%.0f,\"poseDeg\":%.1f,"
        "\"slamQuality\":%.2f,\"slamScans\":%lu,\"slamIntegrated\":%lu,\"slamRejected\":%lu,"
        "\"slamUpdateMs\":%lu,\"mapAreaM2\":%.1f,\"pathPoints\":%u,",
        json_bool(s.map.valid), json_bool(s.map.lost), s.map.x, s.map.y, s.map.theta * 57.29578f,
        s.map.quality, (unsigned long)s.map.scans, (unsigned long)s.map.integrated,
        (unsigned long)s.map.rejected, (unsigned long)s.map.update_ms,
        s.map.known_cells * (SLAM_RES_MM * SLAM_RES_MM / 1e6f), (unsigned)s.map.path_count);
    put(&w, "\"packets\":%lu,\"lidarRxBytes\":%lu,\"lidarHeaders\":%lu,\"lidarRejected\":%lu,"
        "\"lidarRxPin\":%d,\"lidarBaud\":%d,\"lidarRawSample\":\"%s\",\"lidarDiagnostic\":\"%s\","
        "\"crcErrors\":%lu,",
        (unsigned long)stats.packets, (unsigned long)stats.rx_bytes, (unsigned long)stats.headers,
        (unsigned long)stats.rejected_packets, SEEKER_PIN_LIDAR_RX, SEEKER_LIDAR_BAUDRATE,
        sample, diagnostic, (unsigned long)stats.crc_errors);
    int64_t pose_age = esp_timer_get_time() - s.map.feedback.timestamp_us;
    put(&w, "\"slamWeak\":%s,\"slamDegenerate\":%s,\"poseAgeMs\":%lu,"
        "\"lidarDroppedScans\":%lu,\"lidarUartErrors\":%lu,",
        json_bool(s.map.weak), json_bool(s.map.degenerate),
        (unsigned long)(s.map.feedback.timestamp_us && pose_age >= 0 ? pose_age / 1000 : UINT32_MAX),
        (unsigned long)stats.dropped_scans, (unsigned long)stats.uart_errors);
    put(&w, "\"heap\":%lu,\"minHeap\":%lu,\"uptime\":%lld,\"resetReason\":%d,\"resetReasonText\":\"%s\",",
        (unsigned long)esp_get_free_heap_size(), (unsigned long)esp_get_minimum_free_heap_size(),
        (long long)(esp_timer_get_time() / 1000000), (int)esp_reset_reason(),
        seeker_reset_name(esp_reset_reason()));
    diag_memory_t memory = diag_memory_status();
    put(&w, "\"heapInternal\":%lu,\"heapLargest\":%lu,\"allocFailures\":%lu,"
        "\"lastAllocBytes\":%lu,\"lastAllocMs\":%lu,\"bulkPaused\":%s,"
        "\"bulkPauses\":%lu,\"bulkErrors\":%lu,",
        (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
        (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
        (unsigned long)memory.failures, (unsigned long)memory.size, (unsigned long)memory.uptime_ms,
        json_bool(!bulk_allowed()), (unsigned long)s_bulk_pauses, (unsigned long)s_bulk_errors);
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) put(&w, "\"wifiRssi\":%d,", ap.rssi);
    network_stats_t net = seeker_network_stats();
    size_t clients = CONFIG_LWIP_MAX_SOCKETS;
    int fds[CONFIG_LWIP_MAX_SOCKETS];
    if (httpd_get_client_list(s_server, &clients, fds) != ESP_OK) clients = 0;
    put(&w, "\"wifiDisconnects\":%lu,\"wifiFailedAttempts\":%lu,\"wifiLastReason\":%u,"
        "\"wifiConnectedS\":%lu,\"wifiLastOutageMs\":%lu,\"httpClients\":%u,",
        (unsigned long)net.disconnects, (unsigned long)net.failed_attempts, net.last_reason,
        (unsigned long)net.connected_s, (unsigned long)net.last_outage_ms, (unsigned)clients);
    imu_status_t imu = imu_status();
    put(&w, "\"imuPresent\":%s,\"imuReady\":%s,\"imuCalibrating\":%s,\"imuAddress\":%u,"
        "\"imuRateDps\":%.1f,\"imuBiasDps\":%.2f,\"imuYawDeg\":%.1f,\"imuTiltDeg\":%.1f,"
        "\"imuShockG\":%.2f,\"imuHz\":%.0f,\"imuErrors\":%lu,\"imuOverflows\":%lu,"
        "\"slamImu\":%s,\"headingSigmaDeg\":%.2f,\"gyroBiasDps\":%.3f,",
        json_bool(imu.present), json_bool(imu.ready), json_bool(imu.calibrating), imu.address,
        imu.rate_dps, imu.bias_dps, imu.yaw_deg, imu.tilt_deg, imu.shock_g, imu.sample_hz,
        (unsigned long)imu.errors, (unsigned long)imu.overflows, json_bool(s.map.imu),
        s.map.heading_sigma * 57.29578f, s.map.gyro_bias * 57.29578f);
    put(&w, "\"manualLimit\":%d,\"lidarFrontDeg\":%d,\"settings\":{",
        SEEKER_MANUAL_MAX_PERCENT, SEEKER_LIDAR_FRONT_DEG);
    for (unsigned i = 0; i < SETTINGS_COUNT; i++)
        put(&w, "%s\"%s\":%d", i ? "," : "", SETTINGS_FIELDS[i].name,
            *(const int *)((const char *)&s.settings + SETTINGS_FIELDS[i].offset));
    put(&w, "}");
    /* One bounded request carries radar + state, also during movement.
       Keep the lightweight status endpoint for diagnostics/other clients. */
    if (!strcmp(req->uri, "/api/telemetry")) {
        put(&w, ",\"scan\":");
        scan_json(&w);
    }
    put(&w, "}");
    return json_reply(req, &w);
}

static esp_err_t scan_handler(httpd_req_t *req)
{
    writer_t w = {0};
    scan_json(&w);
    return json_reply(req, &w);
}

static void put16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
static void put32(uint8_t *p, uint32_t v) { put16(p, v); put16(p + 2, v >> 16); }
static void putf(uint8_t *p, float v) { uint32_t u; memcpy(&u, &v, 4); put32(p, u); }

/* Binary occupancy map, little-endian:
   40 byte header "SMAP", then SIZE*SIZE 2-bit cells (4 per byte, index
   gy*SIZE+gx: 0 unknown, 1 free, 2 probably occupied, 3 occupied), then
   the path as int16 cm (x, y) pairs. Streamed in bounded chunks. */
static esp_err_t map_handler(httpd_req_t *req)
{
    headers(req);
    if (!bulk_allowed()) return bulk_unavailable(req);
    static int16_t path[SLAM_PATH_MAX][2];
    robot_status_t s = robot_manager_status();
    uint16_t count = mapper_path(path, SLAM_PATH_MAX);
    uint8_t chunk[1024] = {0};
    memcpy(chunk, "SMAP", 4);
    chunk[4] = 1;
    chunk[5] = (s.map.valid ? 1 : 0) | (s.map.lost ? 2 : 0) | (s.goal_set ? 4 : 0);
    put16(chunk + 6, SLAM_SIZE);
    put16(chunk + 8, SLAM_RES_MM);
    put16(chunk + 10, count);
    putf(chunk + 12, s.map.x);
    putf(chunk + 16, s.map.y);
    putf(chunk + 20, s.map.theta);
    putf(chunk + 24, s.goal_x);
    putf(chunk + 28, s.goal_y);
    float quality = s.map.quality * 100;
    chunk[32] = (uint8_t)(quality < 0 ? 0 : quality > 100 ? 100 : quality);
    chunk[33] = 0;
    put32(chunk + 36, s.map.integrated);
    /* New clients opt in; older dashboards still receive the v1 layout. */
    char query[32], encoding[8];
    size_t encoded = 0;
    const int8_t *grid = mapper_grid();
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "rle", encoding, sizeof(encoding)) == ESP_OK && !strcmp(encoding, "1")) {
        encoded = map_transport_rle(grid, SLAM_SIZE * SLAM_SIZE, (uint8_t *)s_json, sizeof(s_json));
        if (encoded) { chunk[4] = 2; put16(chunk + 34, (uint16_t)encoded); }
    }
    httpd_resp_set_type(req, "application/octet-stream");
    int64_t deadline = esp_timer_get_time() + SEEKER_BULK_MAX_US;
    size_t used = 40;
    size_t bytes = encoded ? encoded : SLAM_SIZE * SLAM_SIZE / 4;
    for (size_t i = 0; i < bytes; i++) {
        chunk[used++] = encoded ? (uint8_t)s_json[i] : map_transport_byte(grid, i * 4);
        if (used == sizeof(chunk)) {
            esp_err_t err = bulk_chunk(req, chunk, used, deadline);
            if (err != ESP_OK) return err;
            used = 0;
        }
    }
    for (uint16_t i = 0; i < count; i++) {
        /* RLE lengths are even, not necessarily a multiple of four. */
        if (used + 4 > sizeof(chunk)) {
            esp_err_t err = bulk_chunk(req, chunk, used, deadline);
            if (err != ESP_OK) return err;
            used = 0;
        }
        put16(chunk + used, (uint16_t)path[i][0]);
        put16(chunk + used + 2, (uint16_t)path[i][1]);
        used += 4;
        if (used == sizeof(chunk)) {
            esp_err_t err = bulk_chunk(req, chunk, used, deadline);
            if (err != ESP_OK) return err;
            used = 0;
        }
    }
    if (used) {
        esp_err_t err = bulk_chunk(req, chunk, used, deadline);
        if (err != ESP_OK) return err;
    }
    return httpd_resp_send_chunk(req, NULL, 0);
}

static bool coord_field(cJSON *j, const char *name, float *out)
{
    cJSON *v = cJSON_GetObjectItemCaseSensitive(j, name);
    if (!cJSON_IsNumber(v) || !isfinite(v->valuedouble) || v->valuedouble < -10000 ||
        v->valuedouble > 10000) return false;
    *out = (float)v->valuedouble;
    return true;
}

/* strategy: explore | wall_left | wall_right | goal | path (goalX/goalY mm)
   | home | path_home. goal/home: Bug2, path/path_home: SLAM map path. */
static bool strategy_field(cJSON *j, robot_auto_request_t *r)
{
    cJSON *v = cJSON_GetObjectItemCaseSensitive(j, "strategy");
    r->strategy = NAV_STRATEGY_EXPLORE; r->goal_x = 0; r->goal_y = 0;
    if (!v) return true;
    if (!cJSON_IsString(v)) return false;
    if (!strcmp(v->valuestring, "explore")) return true;
    if (!strcmp(v->valuestring, "wall_left")) { r->strategy = NAV_STRATEGY_WALL_LEFT; return true; }
    if (!strcmp(v->valuestring, "wall_right")) { r->strategy = NAV_STRATEGY_WALL_RIGHT; return true; }
    bool path = !strncmp(v->valuestring, "path", 4);
    r->strategy = path ? NAV_STRATEGY_PATH : NAV_STRATEGY_GOAL;
    if (!strcmp(v->valuestring, "home") || !strcmp(v->valuestring, "path_home")) return true;
    return (!strcmp(v->valuestring, "goal") || !strcmp(v->valuestring, "path")) &&
           coord_field(j, "goalX", &r->goal_x) && coord_field(j, "goalY", &r->goal_y);
}

static bool uint_field(cJSON *j, const char *name, uint32_t *out)
{
    cJSON *v = cJSON_GetObjectItemCaseSensitive(j, name);
    if (!cJSON_IsNumber(v) || !isfinite(v->valuedouble) || v->valuedouble < 0 ||
        v->valuedouble > UINT32_MAX || floor(v->valuedouble) != v->valuedouble) return false;
    *out = (uint32_t)v->valuedouble;
    return true;
}

static bool axis_field(cJSON *j, const char *name, int *out)
{
    cJSON *v = cJSON_GetObjectItemCaseSensitive(j, name);
    if (!cJSON_IsNumber(v) || !isfinite(v->valuedouble) || v->valuedouble < -100 ||
        v->valuedouble > 100 || floor(v->valuedouble) != v->valuedouble) return false;
    *out = v->valueint;
    return true;
}

/* Same-origin request with the custom header prevents browser CSRF.
   Returns the rejection reason or NULL. */
static const char *foreign_request(httpd_req_t *req)
{
    char origin[128], host[96], expected[128], marker[8];
    if (httpd_req_get_hdr_value_str(req, "Origin", origin, sizeof(origin)) != ESP_OK ||
        httpd_req_get_hdr_value_str(req, "Host", host, sizeof(host)) != ESP_OK ||
        httpd_req_get_hdr_value_str(req, "X-Seeker-Control", marker, sizeof(marker)) != ESP_OK ||
        strcmp(marker, "1"))
        return "same_origin_required";
    snprintf(expected, sizeof(expected), "http://%s", host);
    return strcmp(origin, expected) ? "origin_rejected" : NULL;
}

static esp_err_t control_handler(httpd_req_t *req)
{
    /* A controller lease arbitrates clients; it is NOT authentication
       against LAN peers. */
    char type[64];
    const char *foreign = foreign_request(req);
    if (foreign) return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, foreign);
    if (httpd_req_get_hdr_value_str(req, "Content-Type", type, sizeof(type)) != ESP_OK ||
        strcmp(type, "application/json"))
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "json_required");
    if (req->content_len <= 0 || req->content_len > 768)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid_body_size");
    char body[769];
    unsigned used = 0;
    while (used < req->content_len) {
        int n = httpd_req_recv(req, body + used, req->content_len - used);
        if (n <= 0) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "incomplete_body");
        used += n;
    }
    body[used] = 0;
    cJSON *j = cJSON_ParseWithOpts(body, NULL, true);
    if (!cJSON_IsObject(j)) { cJSON_Delete(j); return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid_json"); }
    cJSON *cmd = cJSON_GetObjectItemCaseSensitive(j, "command");
    if (!cJSON_IsString(cmd)) { cJSON_Delete(j); return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "command_required"); }
    if (ota_running() && strcmp(cmd->valuestring, "estop")) {
        cJSON_Delete(j);
        return text_reply(req, "409 Conflict", "{\"ok\":false,\"error\":\"ota_busy\"}");
    }
    uint32_t token = 0, seq = 0;
    int drive = 0, turn = 0;
    esp_err_t err;
    if (!strcmp(cmd->valuestring, "claim")) err = robot_manager_claim(&token);
    else {
        cJSON *mode = cJSON_GetObjectItemCaseSensitive(j, "mode");
        bool valid = !strcmp(cmd->valuestring, "estop") ||
                      (uint_field(j, "token", &token) && uint_field(j, "sequence", &seq));
        if (!strcmp(cmd->valuestring, "move"))
            valid = valid && axis_field(j, "drive", &drive) && axis_field(j, "turn", &turn);
        if (!strcmp(cmd->valuestring, "settings")) {
            motion_settings_t settings;
            for (unsigned i = 0; i < SETTINGS_COUNT; i++) {
                uint32_t value = 0;
                valid = uint_field(j, SETTINGS_FIELDS[i].name, &value) && value <= 5000 && valid;
                *(int *)((char *)&settings + SETTINGS_FIELDS[i].offset) = (int)value;
            }
            err = valid ? robot_manager_configure(token, seq, settings) : ESP_ERR_INVALID_ARG;
        } else {
            robot_auto_request_t request;
            valid = strategy_field(j, &request) && valid;
            err = valid ? robot_manager_command(token, seq, cmd->valuestring, drive, turn,
                         cJSON_IsString(mode) ? mode->valuestring : NULL, &request) : ESP_ERR_INVALID_ARG;
        }
    }
    bool claim = !strcmp(cmd->valuestring, "claim");
    cJSON_Delete(j);
    if (err != ESP_OK)
        return text_reply(req, err == ESP_ERR_INVALID_ARG ? "400 Bad Request" : "409 Conflict",
            err == ESP_ERR_INVALID_ARG ? "{\"ok\":false,\"error\":\"ESP_ERR_INVALID_ARG\"}"
                                       : "{\"ok\":false,\"error\":\"ESP_ERR_INVALID_STATE\"}");
    if (claim) {
        char reply[48];
        snprintf(reply, sizeof(reply), "{\"ok\":true,\"token\":%lu}", (unsigned long)token);
        return text_reply(req, NULL, reply);
    }
    return text_reply(req, NULL, "{\"ok\":true}");
}

static esp_err_t route_get_handler(httpd_req_t *req)
{
    route_runner_t route;
    robot_manager_route_snapshot(&route);
    writer_t w = {0};
    put(&w, "{\"configured\":%s,\"active\":%s,\"generation\":%lu,\"laps\":%lu,"
        "\"completed\":%lu,\"learnedLaps\":%lu,\"adaptive\":%s,\"target\":%u,\"approaching\":%s,\"points\":[",
        json_bool(route.configured), json_bool(route.active), (unsigned long)route.definition.generation,
        (unsigned long)route.definition.laps, (unsigned long)route.completed, (unsigned long)route.learned_laps,
        json_bool(route.definition.adaptive), route.target, json_bool(route.approaching));
    for (unsigned i = 0; i < route.definition.count; i++)
        put(&w, i ? ",[%d,%d]" : "[%d,%d]", route.definition.points[i].x, route.definition.points[i].y);
    put(&w, "],\"scores\":[");
    for (unsigned i = 0; i < route.definition.count; i++) put(&w, i ? ",%u" : "%u", route.score[i]);
    put(&w, "]}");
    return json_reply(req, &w);
}

static esp_err_t route_post_handler(httpd_req_t *req)
{
    const char *foreign = foreign_request(req);
    if (foreign) return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, foreign);
    char type[48];
    if (httpd_req_get_hdr_value_str(req, "Content-Type", type, sizeof(type)) != ESP_OK ||
        strcmp(type, "application/octet-stream") || req->content_len < ROUTE_PACKET_HEADER ||
        req->content_len > ROUTE_PACKET_MAX)
        return text_reply(req, "400 Bad Request", "{\"ok\":false,\"error\":\"route_invalid_packet\"}");
    if (ota_running()) return text_reply(req, "409 Conflict", "{\"ok\":false,\"error\":\"ota_busy\"}");
    if (!bulk_allowed()) return bulk_unavailable(req);
    uint8_t body[ROUTE_PACKET_MAX];
    unsigned used = 0;
    while (used < req->content_len) {
        int n = httpd_req_recv(req, (char *)body + used, req->content_len - used);
        if (n <= 0) return text_reply(req, "400 Bad Request", "{\"ok\":false,\"error\":\"route_incomplete\"}");
        used += n;
    }
    route_definition_t definition;
    uint32_t token, sequence;
    if (!route_packet_decode(body, used, &definition, &token, &sequence))
        return text_reply(req, "400 Bad Request", "{\"ok\":false,\"error\":\"route_invalid_packet\"}");
    const char *reason = NULL;
    esp_err_t error = robot_manager_start_route(token, sequence, &definition, &reason);
    if (error != ESP_OK) {
        char reply[128];
        snprintf(reply, sizeof(reply), "{\"ok\":false,\"error\":\"%s\"}", reason ? reason : "route_rejected");
        return text_reply(req, error == ESP_ERR_INVALID_ARG ? "400 Bad Request" : "409 Conflict", reply);
    }
    return text_reply(req, NULL, "{\"ok\":true}");
}

/* Only images of this project (app descriptor) are accepted. */
static bool own_image(const uint8_t *data, size_t size)
{
    const size_t offset = sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t);
    esp_app_desc_t desc;
    if (size < offset + sizeof(desc) || data[0] != ESP_IMAGE_HEADER_MAGIC) return false;
    memcpy(&desc, data + offset, sizeof(desc));
    return desc.magic_word == ESP_APP_DESC_MAGIC_WORD &&
           !strncmp(desc.project_name, esp_app_get_description()->project_name, sizeof(desc.project_name));
}

static void restart_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(800));      /* let the reply reach the client */
    esp_restart();
}

static esp_err_t ota_error(httpd_req_t *req, const char *status, const char *error)
{
    char reply[64];
    snprintf(reply, sizeof(reply), "{\"ok\":false,\"error\":\"%s\"}", error);
    return text_reply(req, status, reply);
}

static void ota_drop(const char *why)
{
    if (s_ota.begun) esp_ota_abort(s_ota.handle);
    if (s_ota.begun) ESP_LOGE(TAG, "OTA abgebrochen: %s (%u/%u)", why,
                              (unsigned)s_ota.written, (unsigned)s_ota.total);
    s_ota.begun = false;
    s_ota.written = 0;
}

static bool query_size(const char *query, const char *key, size_t *out)
{
    char value[16], *end;
    if (httpd_query_key_value(query, key, value, sizeof(value)) != ESP_OK || !value[0]) return false;
    unsigned long n = strtoul(value, &end, 10);
    *out = n;
    return *end == 0;
}

/* Firmware update (incl. the embedded dashboard), resumable: each request
   carries one block at ?offset=&total=. A Wi-Fi dropout only costs a retry
   of that block; a single ~1 MB stream never survived the access point's
   multi-second outages. The reply always reports the next expected offset,
   so repeated or lost replies resynchronise the client. The bootloader
   rolls back if the new image never brings the web server up. */
static esp_err_t ota_handler(httpd_req_t *req)
{
    char type[48], query[64];
    size_t offset, total, len = req->content_len;
    const char *foreign = foreign_request(req);
    if (foreign) return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, foreign);
    if (httpd_req_get_hdr_value_str(req, "Content-Type", type, sizeof(type)) != ESP_OK ||
        strcmp(type, "application/octet-stream"))
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "binary_required");
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        !query_size(query, "offset", &offset) || !query_size(query, "total", &total))
        return ota_error(req, "400 Bad Request", "offset_required");
    if (len == 0 || len > OTA_CHUNK || offset > total || len > total - offset)
        return ota_error(req, "400 Bad Request", "invalid_size");
    if (offset == 0 && !s_ota.finished) {
        robot_mode_t mode = robot_manager_status().mode;
        if (mode == ROBOT_MANUAL || mode == ROBOT_AUTO)
            return ota_error(req, "409 Conflict", "robot_busy");
        if (mapper_record_info().erasing) return ota_error(req, "409 Conflict", "ota_busy");
        mapper_record_stop();            /* the recording lives in the OTA slot */
        ota_drop("restart");
        s_ota.slot = esp_ota_get_next_update_partition(NULL);
        if (!s_ota.slot) return ota_error(req, "500 Internal Server Error", "no_ota_slot");
        if (total < 1024 || total > s_ota.slot->size) return ota_error(req, "400 Bad Request", "invalid_size");
        s_ota.total = total;
    } else if (!ota_running() || total != s_ota.total) {
        ota_drop("timeout");
        return ota_error(req, "409 Conflict", "no_session");
    }
    /* The server is single-threaded: the JSON buffer is free meanwhile. */
    uint8_t *buf = (uint8_t *)s_json;
    size_t fill = 0;
    int retries = 0;
    while (fill < len) {
        int n = httpd_req_recv(req, (char *)buf + fill, len - fill);
        if (n == HTTPD_SOCK_ERR_TIMEOUT && ++retries <= OTA_RETRIES) continue;
        if (n <= 0) return ota_error(req, "400 Bad Request", "receive_failed");
        fill += n;
    }
    s_ota.last_us = esp_timer_get_time();
    if (!s_ota.finished && offset == s_ota.written) {
        if (offset == 0) {
            if (!own_image(buf, len)) return ota_error(req, "400 Bad Request", "wrong_image");
            if (esp_ota_begin(s_ota.slot, OTA_WITH_SEQUENTIAL_WRITES, &s_ota.handle) != ESP_OK)
                return ota_error(req, "500 Internal Server Error", "begin_failed");
            s_ota.begun = true;
            ESP_LOGW(TAG, "OTA: %u Bytes nach %s", (unsigned)total, s_ota.slot->label);
        }
        if (esp_ota_write(s_ota.handle, buf, len) != ESP_OK) {
            ota_drop("write_failed");
            return ota_error(req, "500 Internal Server Error", "write_failed");
        }
        s_ota.written += len;
        if (s_ota.written == s_ota.total) {
            s_ota.begun = false;         /* esp_ota_end releases the handle */
            if (esp_ota_end(s_ota.handle) != ESP_OK) {
                ota_drop("image_invalid");
                return ota_error(req, "400 Bad Request", "image_invalid");
            }
            if (esp_ota_set_boot_partition(s_ota.slot) != ESP_OK) {
                ota_drop("set_boot_failed");
                return ota_error(req, "500 Internal Server Error", "set_boot_failed");
            }
            s_ota.finished = true;
            ESP_LOGW(TAG, "OTA fertig, Neustart");
            xTaskCreate(restart_task, "ota_restart", 2048, NULL, 5, NULL);
        }
    }
    char reply[64];
    snprintf(reply, sizeof(reply), "{\"ok\":true,\"written\":%u,\"done\":%s}",
             (unsigned)s_ota.written, s_ota.finished ? "true" : "false");
    return text_reply(req, NULL, reply);
}

/* SLAM recorder: GET status JSON, or ?offset=&len= raw records (<= 4 KB). */
static esp_err_t record_get_handler(httpd_req_t *req)
{
    char query[64];
    size_t offset, len;
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        query_size(query, "offset", &offset) && query_size(query, "len", &len)) {
        if (len == 0 || len > sizeof(s_json) || !mapper_record_read(offset, s_json, len))
            return ota_error(req, "400 Bad Request", "out_of_range");
        headers(req);
        httpd_resp_set_type(req, "application/octet-stream");
        return httpd_resp_send(req, s_json, len);
    }
    mapper_record_info_t info = mapper_record_info();
    char reply[128];
    snprintf(reply, sizeof(reply),
             "{\"active\":%s,\"erasing\":%s,\"records\":%lu,\"capacity\":%lu,\"recordSize\":%d}",
             info.active ? "true" : "false", info.erasing ? "true" : "false",
             (unsigned long)info.records, (unsigned long)info.capacity, MAPPER_RECORD_SIZE);
    return text_reply(req, NULL, reply);
}

/* POST ?cmd=start|stop. Start erases the inactive OTA slot: robot idle. */
static esp_err_t record_post_handler(httpd_req_t *req)
{
    char query[32], cmd[8];
    const char *foreign = foreign_request(req);
    if (foreign) return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, foreign);
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "cmd", cmd, sizeof(cmd)) != ESP_OK)
        return ota_error(req, "400 Bad Request", "cmd_required");
    if (!strcmp(cmd, "stop")) {
        mapper_record_stop();
        return text_reply(req, NULL, "{\"ok\":true}");
    }
    if (strcmp(cmd, "start")) return ota_error(req, "400 Bad Request", "cmd_required");
    robot_mode_t mode = robot_manager_status().mode;
    if (mode == ROBOT_MANUAL || mode == ROBOT_AUTO) return ota_error(req, "409 Conflict", "robot_busy");
    if (ota_running() || s_ota.begun) return ota_error(req, "409 Conflict", "ota_busy");
    if (mapper_record_start() != ESP_OK) return ota_error(req, "409 Conflict", "record_busy");
    return text_reply(req, NULL, "{\"ok\":true}");
}

/* Plain-text reset report: counters, last resets, log before the last
   reset and since this boot. */
static esp_err_t diag_get_handler(httpd_req_t *req)
{
    headers(req);
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    size_t len = diag_summary(s_json, sizeof(s_json));
    esp_err_t err = httpd_resp_send_chunk(req, s_json, len);
    const char *previous = diag_previous_log(&len);
    if (err == ESP_OK) err = httpd_resp_sendstr_chunk(req, "\n--- Log vor dem letzten Neustart ---\n");
    if (err == ESP_OK && len) err = httpd_resp_send_chunk(req, previous, len);
    if (err == ESP_OK) err = httpd_resp_sendstr_chunk(req, "\n--- Log seit diesem Start ---\n");
    len = diag_current_log(s_json, sizeof(s_json));
    if (err == ESP_OK && len) err = httpd_resp_send_chunk(req, s_json, len);
    if (err == ESP_OK) err = httpd_resp_send_chunk(req, NULL, 0);
    return err;
}

/* POST ?cmd=clear resets the counters and the history. */
static esp_err_t diag_post_handler(httpd_req_t *req)
{
    char query[32], cmd[8];
    const char *foreign = foreign_request(req);
    if (foreign) return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, foreign);
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "cmd", cmd, sizeof(cmd)) != ESP_OK || strcmp(cmd, "clear"))
        return ota_error(req, "400 Bad Request", "cmd_required");
    diag_clear();
    return text_reply(req, NULL, "{\"ok\":true}");
}

/* A freshly updated image proves itself by bringing the web server up. */
static void confirm_image(void)
{
    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(esp_ota_get_running_partition(), &state) == ESP_OK &&
        state == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_ota_mark_app_valid_cancel_rollback();
        ESP_LOGW(TAG, "Neue Firmware bestaetigt");
    }
}

esp_err_t seeker_web_start(void)
{
    if (s_server) return ESP_OK;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.max_open_sockets = SEEKER_HTTP_CLIENTS;
    config.max_uri_handlers = 14;
    config.lru_purge_enable = true;
    config.open_fn = socket_open;
    config.recv_wait_timeout = 1;
    /* The server is single-threaded: a stalled send must not block the
       heartbeats of the controller for long. Motor deadman stays 350 ms. */
    config.send_wait_timeout = 1;
    config.keep_alive_enable = true;
    config.keep_alive_idle = 5;
    config.keep_alive_interval = 2;
    config.keep_alive_count = 2;
    esp_err_t err = httpd_start(&s_server, &config);
    if (err != ESP_OK) return err;
    const httpd_uri_t routes[] = {
        { .uri = "/", .method = HTTP_GET, .handler = index_handler },
        { .uri = "/api/status", .method = HTTP_GET, .handler = status_handler },
        { .uri = "/api/telemetry", .method = HTTP_GET, .handler = status_handler },
        { .uri = "/api/scan", .method = HTTP_GET, .handler = scan_handler },
        { .uri = "/api/map", .method = HTTP_GET, .handler = map_handler },
        { .uri = "/api/control", .method = HTTP_POST, .handler = control_handler },
        { .uri = "/api/route", .method = HTTP_GET, .handler = route_get_handler },
        { .uri = "/api/route", .method = HTTP_POST, .handler = route_post_handler },
        { .uri = "/api/ota", .method = HTTP_POST, .handler = ota_handler },
        { .uri = "/api/record", .method = HTTP_GET, .handler = record_get_handler },
        { .uri = "/api/record", .method = HTTP_POST, .handler = record_post_handler },
        { .uri = "/api/diag", .method = HTTP_GET, .handler = diag_get_handler },
        { .uri = "/api/diag", .method = HTTP_POST, .handler = diag_post_handler },
    };
    for (unsigned i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
        err = httpd_register_uri_handler(s_server, &routes[i]);
        if (err != ESP_OK) { httpd_stop(s_server); s_server = NULL; return err; }
    }
    confirm_image();
    return ESP_OK;
}
