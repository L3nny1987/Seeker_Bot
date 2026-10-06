#include "ld20_lidar.h"
#include <string.h>
#include "seeker_pins.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static SemaphoreHandle_t s_lock;
static TaskHandle_t s_task;
static ld20_scan_t s_scan, s_work;
static ld20_stats_t s_stats;
static bool s_synced;
static int s_previous_angle = -1;
static int64_t s_last_packet;

static uint16_t le16(const uint8_t *p) { return p[0] | ((uint16_t)p[1] << 8); }

static uint8_t crc8(const uint8_t *p, unsigned len)
{
    uint8_t crc = 0;
    while (len--) {
        crc ^= *p++;
        for (unsigned bit = 0; bit < 8; bit++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x4d) : (uint8_t)(crc << 1);
    }
    return crc;
}

static void packet(const uint8_t *p)
{
    unsigned start = le16(p + 4), end = le16(p + 42);
    unsigned speed = le16(p + 2); /* degrees/second, NOT RPM */
    unsigned span = (end + 36000 - start) % 36000;
    if (start >= 36000 || end >= 36000 || speed < 360 || speed > 7200 || span > 4500) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        s_stats.rejected_packets++;
        xSemaphoreGive(s_lock);
        return;
    }
    int64_t now = esp_timer_get_time();
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_last_packet && now - s_last_packet > SEEKER_LIDAR_STALE_MS * 1000LL) {
        memset(&s_work, 0, sizeof(s_work));
        s_synced = false;
        s_previous_angle = -1;
    }
    s_last_packet = now;
    s_stats.packets++;
    for (unsigned i = 0; i < LD20_POINTS_PER_PACKET; i++) {
        unsigned angle = (start + span * i / (LD20_POINTS_PER_PACKET - 1)) % 36000;
        if (s_previous_angle > 30000 && angle < 6000) {
            if (s_synced) {
                s_work.speed_rpm = speed / 6;
                s_work.timestamp_us = now;
                s_work.sequence = s_scan.sequence + 1;
                s_scan = s_work;
                s_stats.scans++;
            }
            memset(&s_work, 0, sizeof(s_work));
            s_synced = true;
        }
        s_previous_angle = angle;
        unsigned sector = angle / 100;
        uint16_t distance = le16(p + 6 + i * 3);
        uint8_t intensity = p[8 + i * 3];
        if (distance >= 80 && distance <= 8000 && intensity >= SEEKER_LIDAR_MIN_INTENSITY) {
            if (!s_work.distance_mm[sector]) s_work.valid_sectors++;
            if (!s_work.distance_mm[sector] || distance < s_work.distance_mm[sector]) {
                s_work.distance_mm[sector] = distance;
                s_work.intensity[sector] = intensity;
            }
        }
    }
    xSemaphoreGive(s_lock);
}

static void receive_task(void *arg)
{
    (void)arg;
    uint8_t bytes[256], frame[47];
    unsigned used = 0;
    while (true) {
        int n = uart_read_bytes(SEEKER_LIDAR_UART_NUM, bytes, sizeof(bytes), pdMS_TO_TICKS(20));
        if (n > 0) {
            xSemaphoreTake(s_lock, portMAX_DELAY);
            s_stats.rx_bytes += n;
            s_stats.sample_len = n < (int)sizeof(s_stats.sample) ? n : sizeof(s_stats.sample);
            memcpy(s_stats.sample, bytes, s_stats.sample_len);
            xSemaphoreGive(s_lock);
        }
        for (int i = 0; i < n; i++) {
            if (!used && bytes[i] != 0x54) continue;
            frame[used++] = bytes[i];
            if (used == 2 && frame[1] != 0x2c) {
                used = frame[1] == 0x54 ? 1 : 0;
                continue;
            }
            if (used == 2) {
                xSemaphoreTake(s_lock, portMAX_DELAY);
                s_stats.headers++;
                xSemaphoreGive(s_lock);
            }
            if (used != sizeof(frame)) continue;
            if (crc8(frame, 46) == frame[46]) { packet(frame); used = 0; }
            else {
                xSemaphoreTake(s_lock, portMAX_DELAY);
                s_stats.crc_errors++;
                xSemaphoreGive(s_lock);
                unsigned offset = 1;
                while (offset < used && !(frame[offset] == 0x54 &&
                       (offset + 1 == used || frame[offset + 1] == 0x2c))) offset++;
                used -= offset;
                memmove(frame, frame + offset, used);
            }
        }
    }
}

esp_err_t ld20_lidar_init(void)
{
    if (s_task) return ESP_ERR_INVALID_STATE;
    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) return ESP_ERR_NO_MEM;
    uart_config_t config = { .baud_rate = SEEKER_LIDAR_BAUDRATE,
        .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT };
    esp_err_t err = uart_param_config(SEEKER_LIDAR_UART_NUM, &config);
    if (err == ESP_OK) err = uart_set_pin(SEEKER_LIDAR_UART_NUM, SEEKER_PIN_LIDAR_TX,
                                         SEEKER_PIN_LIDAR_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err == ESP_OK) err = uart_driver_install(SEEKER_LIDAR_UART_NUM, 4096, 0, 0, NULL, 0);
    if (err != ESP_OK) return err;
    if (xTaskCreate(receive_task, "lidar_rx", 4096, NULL, 7, &s_task) != pdPASS) {
        uart_driver_delete(SEEKER_LIDAR_UART_NUM);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t ld20_lidar_deinit(void)
{
    /* Lifetime component: no unsafe deletion of a running receiver. */
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t ld20_lidar_get_scan(ld20_scan_t *out, uint32_t timeout_ms)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    if (!s_task) return ESP_ERR_INVALID_STATE;
    int64_t deadline = esp_timer_get_time() + timeout_ms * 1000LL;
    do {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        *out = s_scan;
        xSemaphoreGive(s_lock);
        if (out->timestamp_us && esp_timer_get_time() - out->timestamp_us < SEEKER_LIDAR_STALE_MS * 1000LL)
            return ESP_OK;
        if (!timeout_ms || esp_timer_get_time() >= deadline) return ESP_ERR_TIMEOUT;
        vTaskDelay(pdMS_TO_TICKS(10));
    } while (true);
}

esp_err_t ld20_lidar_min_distance(uint16_t start_deg, uint16_t end_deg, uint16_t *out_mm)
{
    if (!out_mm || start_deg >= 360 || end_deg >= 360) return ESP_ERR_INVALID_ARG;
    ld20_scan_t scan;
    esp_err_t err = ld20_lidar_get_scan(&scan, 0);
    if (err != ESP_OK) return err;
    uint16_t min = UINT16_MAX;
    for (unsigned i = start_deg;; i = (i + 1) % 360) {
        if (scan.distance_mm[i] && scan.distance_mm[i] < min) min = scan.distance_mm[i];
        if (i == end_deg) break;
    }
    if (min == UINT16_MAX) return ESP_ERR_NOT_FOUND;
    *out_mm = min;
    return ESP_OK;
}

bool ld20_lidar_is_healthy(void)
{
    ld20_scan_t scan;
    return ld20_lidar_get_scan(&scan, 0) == ESP_OK && scan.valid_sectors >= SEEKER_AUTO_MIN_SECTORS;
}

ld20_stats_t ld20_lidar_get_stats(void)
{
    ld20_stats_t stats = {0};
    if (!s_lock) return stats;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    stats = s_stats;
    xSemaphoreGive(s_lock);
    return stats;
}
