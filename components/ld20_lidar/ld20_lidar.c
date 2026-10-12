#include "ld20_lidar.h"
#include "ld20_scan_builder.h"
#include <string.h>
#include "seeker_pins.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static SemaphoreHandle_t s_lock;
static TaskHandle_t s_task;
static ld20_scan_t s_scan;
static ld20_scan_builder_t s_builder;
static ld20_stats_t s_stats;
static QueueHandle_t s_uart_events;

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

static void packet(const uint8_t *p, int64_t now)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    int result = ld20_scan_builder_packet(&s_builder, p, now, SEEKER_LIDAR_MIN_INTENSITY, &s_scan);
    if (result < 0) s_stats.rejected_packets++;
    else s_stats.packets++;
    if (result > 0) s_stats.scans++;
    s_stats.dropped_scans = s_builder.dropped_scans;
    xSemaphoreGive(s_lock);
}

static void receive_task(void *arg)
{
    (void)arg;
    uint8_t bytes[256], frame[47];
    unsigned used = 0;
    int64_t last_byte_us = 0;
    while (true) {
        uart_event_t event;
        while (xQueueReceive(s_uart_events, &event, 0) == pdTRUE) {
            if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL ||
                event.type == UART_FRAME_ERR || event.type == UART_PARITY_ERR) {
                uart_flush_input(SEEKER_LIDAR_UART_NUM);
                xQueueReset(s_uart_events);
                used = 0;
                xSemaphoreTake(s_lock, portMAX_DELAY);
                ld20_scan_builder_reset(&s_builder);
                s_stats.uart_errors++;
                s_stats.dropped_scans = s_builder.dropped_scans;
                xSemaphoreGive(s_lock);
            }
        }
        /* Wait for the first byte, then take what is buffered: the scan
           timestamp feeds the gyro fusion, waiting for a full 256-byte block
           delayed it by up to 15 ms. Each frame is back-dated to its last byte. */
        int n = uart_read_bytes(SEEKER_LIDAR_UART_NUM, bytes, 1, pdMS_TO_TICKS(20));
        if (n > 0) {
            int more = uart_read_bytes(SEEKER_LIDAR_UART_NUM, bytes + 1, sizeof(bytes) - 1, 0);
            if (more > 0) n += more;
        }
        int64_t received = esp_timer_get_time();
        const int64_t byte_us = 10000000LL / SEEKER_LIDAR_BAUDRATE;
        if (n > 0) {
            if (last_byte_us && received - (n - 1) * byte_us - last_byte_us > 20000) used = 0;
            last_byte_us = received;
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
            if (crc8(frame, 46) == frame[46]) { packet(frame, received - (n - 1 - i) * byte_us); used = 0; }
            else {
                xSemaphoreTake(s_lock, portMAX_DELAY);
                s_stats.crc_errors++;
                ld20_scan_builder_reset(&s_builder);
                s_stats.dropped_scans = s_builder.dropped_scans;
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
    if (err == ESP_OK) err = uart_driver_install(SEEKER_LIDAR_UART_NUM, 4096, 0, 16, &s_uart_events, 0);
    if (err != ESP_OK) return err;
    ld20_scan_builder_reset(&s_builder);
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
