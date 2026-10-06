#include "web_server.h"
#include "robot_manager.h"
#include "ld20_lidar.h"
#include "seeker_pins.h"
#include <math.h>
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

static httpd_handle_t s_server;

static esp_err_t socket_open(httpd_handle_t server, int socket)
{
    (void)server;
    /* Small control replies must not wait for Nagle/delayed ACK batching. */
    int enabled = 1;
    return setsockopt(socket, IPPROTO_TCP, TCP_NODELAY, &enabled, sizeof(enabled)) == 0
           ? ESP_OK : ESP_FAIL;
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

static esp_err_t json_reply(httpd_req_t *req, cJSON *root)
{
    if (!root) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "allocation_failed");
    char *text = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!text) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "allocation_failed");
    headers(req);
    httpd_resp_set_type(req, "application/json");
    esp_err_t err = httpd_resp_send(req, text, HTTPD_RESP_USE_STRLEN);
    free(text);
    return err;
}

static esp_err_t index_handler(httpd_req_t *req)
{
    headers(req);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_set_hdr(req, "Vary", "Accept-Encoding");
    /* The uncompressed CSS/HTML previously exceeded the small TCP send
       window on a weak Wi-Fi link. Stream a compact asset in bounded chunks. */
    for (size_t offset = 0; offset < sizeof(dashboard_gzip); offset += 1024) {
        size_t count = sizeof(dashboard_gzip) - offset;
        if (count > 1024) count = 1024;
        esp_err_t err = httpd_resp_send_chunk(req, (const char *)dashboard_gzip + offset, count);
        if (err != ESP_OK) return err;
    }
    return httpd_resp_send_chunk(req, NULL, 0);
}

static esp_err_t status_handler(httpd_req_t *req)
{
    robot_status_t s = robot_manager_status();
    ld20_stats_t stats = ld20_lidar_get_stats();
    cJSON *j = cJSON_CreateObject();
    if (!j) return json_reply(req, NULL);
    cJSON_AddStringToObject(j, "version", SEEKER_BOT_VERSION);
    cJSON_AddStringToObject(j, "mode", robot_mode_name(s.mode));
    cJSON_AddStringToObject(j, "reason", s.reason);
    cJSON_AddBoolToObject(j, "network", s.network);
    cJSON_AddBoolToObject(j, "controller", s.controller);
    cJSON_AddBoolToObject(j, "commissioned", s.commissioned);
    cJSON_AddBoolToObject(j, "lidarHealthy", s.lidar_healthy);
    cJSON_AddNumberToObject(j, "drive", s.drive);
    cJSON_AddNumberToObject(j, "turn", s.turn);
    cJSON_AddNumberToObject(j, "coverage", s.coverage);
    cJSON_AddNumberToObject(j, "scanAgeMs", s.scan_age_ms);
    cJSON_AddNumberToObject(j, "scanSequence", s.scan_sequence);
    cJSON_AddNumberToObject(j, "frontMm", s.view.front_mm);
    cJSON_AddNumberToObject(j, "rearMm", s.view.rear_mm);
    cJSON_AddNumberToObject(j, "leftMm", s.view.left_mm);
    cJSON_AddNumberToObject(j, "rightMm", s.view.right_mm);
    cJSON_AddNumberToObject(j, "nearestMm", s.view.nearest_mm);
    cJSON_AddNumberToObject(j, "packets", stats.packets);
    cJSON_AddNumberToObject(j, "lidarRxBytes", stats.rx_bytes);
    cJSON_AddNumberToObject(j, "lidarHeaders", stats.headers);
    cJSON_AddNumberToObject(j, "lidarRejected", stats.rejected_packets);
    cJSON_AddNumberToObject(j, "lidarRxPin", SEEKER_PIN_LIDAR_RX);
    cJSON_AddNumberToObject(j, "lidarBaud", SEEKER_LIDAR_BAUDRATE);
    char sample[sizeof(stats.sample) * 2 + 1] = {0};
    for (unsigned i = 0; i < stats.sample_len; i++)
        snprintf(sample + i * 2, 3, "%02X", stats.sample[i]);
    cJSON_AddStringToObject(j, "lidarRawSample", sample);
    cJSON_AddStringToObject(j, "lidarDiagnostic", !stats.rx_bytes ? "no_uart_bytes" :
        !stats.headers ? "no_ld_header" : !stats.packets ? "no_valid_packets" :
        !s.scan_sequence ? "waiting_for_revolution" :
        !s.lidar_healthy ? "stale_or_incomplete_scan" : "healthy");
    cJSON_AddNumberToObject(j, "crcErrors", stats.crc_errors);
    cJSON_AddNumberToObject(j, "heap", esp_get_free_heap_size());
    cJSON_AddNumberToObject(j, "uptime", esp_timer_get_time() / 1000000);
    cJSON_AddNumberToObject(j, "resetReason", esp_reset_reason());
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK)
        cJSON_AddNumberToObject(j, "wifiRssi", ap.rssi);
    cJSON_AddNumberToObject(j, "manualLimit", SEEKER_MANUAL_MAX_PERCENT);
    return json_reply(req, j);
}

static esp_err_t scan_handler(httpd_req_t *req)
{
    ld20_scan_t scan = {0};
    bool fresh = ld20_lidar_get_scan(&scan, 0) == ESP_OK;
    cJSON *j = cJSON_CreateObject();
    if (!j) return json_reply(req, NULL);
    cJSON_AddBoolToObject(j, "fresh", fresh);
    cJSON_AddNumberToObject(j, "rpm", scan.speed_rpm);
    cJSON_AddNumberToObject(j, "sequence", scan.sequence);
    cJSON *points = cJSON_AddArrayToObject(j, "distance");
    if (!points) { cJSON_Delete(j); return json_reply(req, NULL); }
    for (int i = 0; i < 360; i++) {
        int raw = (SEEKER_LIDAR_FRONT_DEG + (SEEKER_LIDAR_CLOCKWISE ? i : -i) + 720) % 360;
        if (!cJSON_AddItemToArray(points, cJSON_CreateNumber(scan.distance_mm[raw]))) {
            cJSON_Delete(j); return json_reply(req, NULL);
        }
    }
    return json_reply(req, j);
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

static esp_err_t control_handler(httpd_req_t *req)
{
    /* Same-origin JSON + custom header prevents browser CSRF. A controller
       lease arbitrates clients; it is NOT authentication against LAN peers. */
    char origin[128], host[96], expected[128], marker[8], type[64];
    if (httpd_req_get_hdr_value_str(req, "Origin", origin, sizeof(origin)) != ESP_OK ||
        httpd_req_get_hdr_value_str(req, "Host", host, sizeof(host)) != ESP_OK ||
        httpd_req_get_hdr_value_str(req, "X-Seeker-Control", marker, sizeof(marker)) != ESP_OK ||
        strcmp(marker, "1"))
        return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "same_origin_required");
    snprintf(expected, sizeof(expected), "http://%s", host);
    if (strcmp(origin, expected))
        return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "origin_rejected");
    if (httpd_req_get_hdr_value_str(req, "Content-Type", type, sizeof(type)) != ESP_OK ||
        strcmp(type, "application/json"))
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "json_required");
    if (req->content_len <= 0 || req->content_len > 256)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid_body_size");
    char body[257];
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
        err = valid ? robot_manager_command(token, seq, cmd->valuestring, drive, turn,
                     cJSON_IsString(mode) ? mode->valuestring : NULL) : ESP_ERR_INVALID_ARG;
    }
    bool claim = !strcmp(cmd->valuestring, "claim");
    cJSON_Delete(j);
    if (err != ESP_OK) {
        httpd_resp_set_status(req, err == ESP_ERR_INVALID_ARG ? "400 Bad Request" : "409 Conflict");
        cJSON *reply = cJSON_CreateObject();
        if (reply) { cJSON_AddBoolToObject(reply, "ok", false); cJSON_AddStringToObject(reply, "error", esp_err_to_name(err)); }
        return json_reply(req, reply);
    }
    cJSON *reply = cJSON_CreateObject();
    if (reply) {
        cJSON_AddBoolToObject(reply, "ok", true);
        if (claim) cJSON_AddNumberToObject(reply, "token", token);
    }
    return json_reply(req, reply);
}

esp_err_t seeker_web_start(void)
{
    if (s_server) return ESP_OK;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.max_open_sockets = 4;
    config.lru_purge_enable = true;
    config.open_fn = socket_open;
    config.recv_wait_timeout = 1;
    /* Network delivery timeout only; motor deadman remains 350 ms. */
    config.send_wait_timeout = 5;
    esp_err_t err = httpd_start(&s_server, &config);
    if (err != ESP_OK) return err;
    const httpd_uri_t routes[] = {
        { .uri = "/", .method = HTTP_GET, .handler = index_handler },
        { .uri = "/api/status", .method = HTTP_GET, .handler = status_handler },
        { .uri = "/api/scan", .method = HTTP_GET, .handler = scan_handler },
        { .uri = "/api/control", .method = HTTP_POST, .handler = control_handler },
    };
    for (unsigned i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
        err = httpd_register_uri_handler(s_server, &routes[i]);
        if (err != ESP_OK) { httpd_stop(s_server); s_server = NULL; return err; }
    }
    return ESP_OK;
}