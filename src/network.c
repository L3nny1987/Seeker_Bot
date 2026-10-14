#include "network.h"
#include "credentials.h"
#include "robot_manager.h"
#include "web_server.h"
#include "network_limits.h"
#include <string.h>
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

static EventGroupHandle_t s_events;
static const char *TAG = "network";
#define CONNECTED_BIT BIT0
#define BACKOFF_MIN_MS   500
#define BACKOFF_MAX_MS   4000
#define ATTEMPT_TIMEOUT  (15 * 1000000LL)   /* association must finish */
#define DHCP_TIMEOUT     (15 * 1000000LL)   /* associated but no IP */

/* Written by the event loop, read by the supervisor task and HTTP. */
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static bool s_associated, s_connecting;
static int64_t s_attempt_us, s_assoc_us, s_next_us, s_lost_us, s_connected_us;
static uint32_t s_backoff_ms = BACKOFF_MIN_MS;
static network_stats_t s_stats;

static void event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    int64_t now = esp_timer_get_time();
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        taskENTER_CRITICAL(&s_mux);
        s_next_us = now;                 /* supervisor connects right away */
        taskEXIT_CRITICAL(&s_mux);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_CONNECTED) {
        taskENTER_CRITICAL(&s_mux);
        s_associated = true;
        s_connecting = false;
        s_assoc_us = now;
        taskEXIT_CRITICAL(&s_mux);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *disconnected = data;
        bool was_online = xEventGroupGetBits(s_events) & CONNECTED_BIT;
        xEventGroupClearBits(s_events, CONNECTED_BIT);
        robot_manager_network(false);
        taskENTER_CRITICAL(&s_mux);
        if (was_online) { s_stats.disconnects++; s_lost_us = now; s_backoff_ms = BACKOFF_MIN_MS; }
        else s_stats.failed_attempts++;
        if (!s_lost_us) s_lost_us = now;
        s_stats.last_reason = disconnected->reason;
        s_associated = false;
        s_connecting = false;
        /* Never interrupt an attempt: the next one starts after a short,
           growing pause instead of a blind fixed-interval connect. */
        s_next_us = now + s_backoff_ms * 1000LL;
        s_backoff_ms = s_backoff_ms * 2 > BACKOFF_MAX_MS ? BACKOFF_MAX_MS : s_backoff_ms * 2;
        taskEXIT_CRITICAL(&s_mux);
        ESP_LOGW(TAG, "WLAN getrennt; reason=%u; Antrieb gestoppt", disconnected->reason);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_LOST_IP) {
        xEventGroupClearBits(s_events, CONNECTED_BIT);
        robot_manager_network(false);
        ESP_LOGW(TAG, "IP-Adresse verloren");
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ip = data;
        esp_err_t err = seeker_web_start();
        if (err != ESP_OK) {
            robot_manager_network(false);
            ESP_LOGE(TAG, "HTTP start failed: %s", esp_err_to_name(err));
            esp_wifi_disconnect();
            return;
        }
        robot_manager_network(true);
        xEventGroupSetBits(s_events, CONNECTED_BIT);
        taskENTER_CRITICAL(&s_mux);
        s_backoff_ms = BACKOFF_MIN_MS;
        s_connected_us = now;
        if (s_lost_us) s_stats.last_outage_ms = (uint32_t)((now - s_lost_us) / 1000);
        s_lost_us = 0;
        taskEXIT_CRITICAL(&s_mux);
        ESP_LOGI(TAG, "Dashboard: http://" IPSTR, IP2STR(&ip->ip_info.ip));
    }
}

/* Starts attempts and breaks out of stuck states (no event, no DHCP). */
static void supervisor_task(void *arg)
{
    (void)arg;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(250));
        int64_t now = esp_timer_get_time();
        bool online = xEventGroupGetBits(s_events) & CONNECTED_BIT;
        bool start = false, abort_attempt = false;
        taskENTER_CRITICAL(&s_mux);
        if (!s_associated && !s_connecting && s_next_us && now >= s_next_us) {
            s_connecting = true;
            s_attempt_us = now;
            start = true;
        } else if (s_connecting && now - s_attempt_us > ATTEMPT_TIMEOUT) {
            s_connecting = false;
            abort_attempt = true;
        } else if (s_associated && !online && now - s_assoc_us > DHCP_TIMEOUT) {
            s_assoc_us = now;
            abort_attempt = true;
        }
        taskEXIT_CRITICAL(&s_mux);
        if (start && esp_wifi_connect() != ESP_OK) {
            taskENTER_CRITICAL(&s_mux);
            s_connecting = false;
            s_next_us = now + 1000000;
            taskEXIT_CRITICAL(&s_mux);
        }
        if (abort_attempt) {
            ESP_LOGW(TAG, "WLAN-Verbindung haengt; neuer Versuch");
            /* Raises STA_DISCONNECTED, which schedules the next attempt. */
            if (esp_wifi_disconnect() != ESP_OK) {
                taskENTER_CRITICAL(&s_mux);
                s_associated = false;
                s_next_us = now + 1000000;
                taskEXIT_CRITICAL(&s_mux);
            }
        }
    }
}

network_stats_t seeker_network_stats(void)
{
    network_stats_t stats;
    taskENTER_CRITICAL(&s_mux);
    stats = s_stats;
    int64_t since = s_connected_us;
    taskEXIT_CRITICAL(&s_mux);
    bool online = s_events && (xEventGroupGetBits(s_events) & CONNECTED_BIT);
    stats.connected_s = online ? (uint32_t)((esp_timer_get_time() - since) / 1000000) : 0;
    return stats;
}

esp_err_t seeker_network_init(void)
{
    /* Wi-Fi secrets are compiled locally, never logged or served by HTTP.
       A firmware binary still contains these strings: keep binaries private. */
    _Static_assert(sizeof(_WIFI_SSID) <= 33, "SSID too long");
    _Static_assert(sizeof(_WIFI_PASS) <= 64, "WPA passphrase too long");
    if (strlen(_WIFI_SSID) == 0 || strlen(_WIFI_PASS) < 8) return ESP_ERR_INVALID_ARG;
    /* NVS is initialised in app_main before the robot manager. */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    if (!netif) return ESP_ERR_NO_MEM;
    ESP_ERROR_CHECK(esp_netif_set_hostname(netif, "seeker-bot"));
    s_events = xEventGroupCreate();
    if (!s_events) return ESP_ERR_NO_MEM;
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    /* Apply at runtime too: existing sdkconfig files otherwise keep the old
       32/32 dynamic buffers when only sdkconfig.defaults is edited. */
    init.static_rx_buf_num = SEEKER_WIFI_STATIC_RX_BUFFERS;
    init.dynamic_rx_buf_num = SEEKER_WIFI_DYNAMIC_RX_BUFFERS;
    init.dynamic_tx_buf_num = SEEKER_WIFI_DYNAMIC_TX_BUFFERS;
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_LOST_IP, event, NULL));
    wifi_config_t config = {0};
    memcpy(config.sta.ssid, _WIFI_SSID, strlen(_WIFI_SSID));
    memcpy(config.sta.password, _WIFI_PASS, strlen(_WIFI_PASS));
    config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    config.sta.pmf_cfg.capable = true;
    /* Mesh/repeater: always pick the strongest access point. */
    config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    /* 13 dBm instead of 20 dBm: much lower TX current peaks (brownouts on
       weak supplies); plenty for an indoor robot near the access point. */
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(52));
    return xTaskCreate(supervisor_task, "wifi_supervisor", 3072, NULL, 3, NULL) == pdPASS
           ? ESP_OK : ESP_ERR_NO_MEM;
}
