#include "network.h"
#include "credentials.h"
#include "robot_manager.h"
#include "web_server.h"
#include <string.h>
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

static EventGroupHandle_t s_events;
static const char *TAG = "network";
#define CONNECTED_BIT BIT0

static void event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_events, CONNECTED_BIT);
        robot_manager_network(false);
        wifi_event_sta_disconnected_t *disconnected = data;
        ESP_LOGW(TAG, "WLAN getrennt; reason=%u; Antrieb gestoppt", disconnected->reason);
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
        ESP_LOGI(TAG, "Dashboard: http://" IPSTR, IP2STR(&ip->ip_info.ip));
    }
}

static void reconnect_task(void *arg)
{
    (void)arg;
    while (true) {
        if (!(xEventGroupGetBits(s_events) & CONNECTED_BIT)) esp_wifi_connect();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

esp_err_t seeker_network_init(void)
{
    /* Wi-Fi secrets are compiled locally, never logged or served by HTTP.
       A firmware binary still contains these strings: keep binaries private. */
    _Static_assert(sizeof(_WIFI_SSID) <= 33, "SSID too long");
    _Static_assert(sizeof(_WIFI_PASS) <= 64, "WPA passphrase too long");
    if (strlen(_WIFI_SSID) == 0 || strlen(_WIFI_PASS) < 8) return ESP_ERR_INVALID_ARG;
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) return err;
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    if (!netif) return ESP_ERR_NO_MEM;
    ESP_ERROR_CHECK(esp_netif_set_hostname(netif, "seeker-bot"));
    s_events = xEventGroupCreate();
    if (!s_events) return ESP_ERR_NO_MEM;
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, event, NULL));
    wifi_config_t config = {0};
    memcpy(config.sta.ssid, _WIFI_SSID, strlen(_WIFI_SSID));
    memcpy(config.sta.password, _WIFI_PASS, strlen(_WIFI_PASS));
    config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    config.sta.pmf_cfg.capable = true;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    return xTaskCreate(reconnect_task, "wifi_reconnect", 3072, NULL, 3, NULL) == pdPASS
           ? ESP_OK : ESP_ERR_NO_MEM;
}