/**
 * @file    main.c
 * @brief   Einstiegspunkt des Seeker_Bot.
 *
 * All movement is owned by the robot manager; boot never starts driving.
 */

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ld20_lidar.h"
#include "seeker_pins.h"
#include "track_drive.h"
#include "robot_manager.h"
#include "network.h"

static const char *TAG = "seeker_bot";

void app_main(void)
{
    ESP_LOGI(TAG, "Seeker_Bot v%s startet", SEEKER_BOT_VERSION);

    ESP_ERROR_CHECK(track_drive_init());
    ESP_ERROR_CHECK(ld20_lidar_init());

    ESP_ERROR_CHECK(robot_manager_init());
    ESP_ERROR_CHECK(seeker_network_init());
    ESP_LOGI(TAG, "Bereit. Antrieb aus; WLAN-Verbindung wird aufgebaut.");
}
