/**
 * @file    ld20_lidar.h
 * @brief   Treiber fuer den FHL-LD20 2D-LiDAR (UART).
 *
 * Protokoll (LD-Serie, identisch zu LD06/LD19/LD20):
 *   - UART 230400 Baud, 8N1, 3.3 V TTL, nur Senderichtung LiDAR -> MCU
 *   - Paketlaenge 47 Byte:
 *       0x54 | VerLen | Speed(2) | StartAngle(2) | 12x {Dist(2), Intensity(1)}
 *       | EndAngle(2) | Timestamp(2) | CRC8(1)
 *   - Winkel in 0.01 deg, Distanz in mm
 *
 * This protocol must be verified against the actual sensor before driving.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "ld20_scan.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t rx_bytes;
    uint32_t headers;
    uint32_t rejected_packets;
    uint8_t sample[16];
    uint8_t sample_len;
    uint32_t packets;
    uint32_t crc_errors;
    uint32_t scans;
    uint32_t dropped_scans;
    uint32_t uart_errors;
} ld20_stats_t;

ld20_stats_t ld20_lidar_get_stats(void);

/**
 * @brief UART initialisieren und Empfangstask starten.
 */
esp_err_t ld20_lidar_init(void);

/**
 * @brief Empfangstask stoppen und UART freigeben.
 */
esp_err_t ld20_lidar_deinit(void);

/**
 * @brief Letzten vollstaendigen Scan kopieren (threadsicher).
 *
 * @param out        Zielpuffer
 * @param timeout_ms Wartezeit auf einen frischen Scan
 */
esp_err_t ld20_lidar_get_scan(ld20_scan_t *out, uint32_t timeout_ms);

/**
 * @brief Kleinste Entfernung in einem Winkelbereich ermitteln.
 *
 * @param start_deg   Startwinkel 0..359
 * @param end_deg     Endwinkel 0..359 (darf ueber 0 hinweg laufen)
 * @param out_mm      Ergebnis in mm
 */
esp_err_t ld20_lidar_min_distance(uint16_t start_deg, uint16_t end_deg, uint16_t *out_mm);

/**
 * @brief true, wenn seit kurzem gueltige Pakete empfangen wurden.
 */
bool ld20_lidar_is_healthy(void);

#ifdef __cplusplus
}
#endif
