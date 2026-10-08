/**
 * @file    main.c
 * @brief   Einstiegspunkt des Seeker_Bot.
 *
 * All movement is owned by the robot manager; boot never starts driving.
 */

#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ld20_lidar.h"
#include "seeker_pins.h"
#include "track_drive.h"
#include "robot_manager.h"
#include "mapper.h"
#include "imu.h"
#include "network.h"
#include "reset_diagnostics.h"

static const char *TAG = "seeker_bot";

void app_main(void)
{
    ESP_LOGI(TAG, "Seeker_Bot v%s startet", SEEKER_BOT_VERSION);
    ESP_LOGW(TAG, "Reset reason: %s (%d)", seeker_reset_name(esp_reset_reason()), esp_reset_reason());

    ESP_ERROR_CHECK(track_drive_init());
    ESP_ERROR_CHECK(ld20_lidar_init());

    /* NVS holds Wi-Fi calibration and the persisted motion settings. */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* Optional: without an MPU6050 the SLAM runs on LiDAR alone. */
    err = imu_init();
    if (err != ESP_OK) ESP_LOGW(TAG, "IMU nicht verfuegbar: %s", esp_err_to_name(err));
    ESP_ERROR_CHECK(mapper_init());
    ESP_ERROR_CHECK(robot_manager_init());
    ESP_ERROR_CHECK(seeker_network_init());
    ESP_LOGI(TAG, "Bereit. Antrieb aus; WLAN-Verbindung wird aufgebaut.");
}
