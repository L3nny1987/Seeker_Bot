/**
 * @file    track_drive.h
 * @brief   Ansteuerung des Kettenantriebs ueber den L298N.
 *
 * Kinematik des Seeker_Bot:
 *   Der Antrieb ist KEIN Differentialantrieb. Es gibt zwei getrennte Achsen:
 *     - FAHRT   : ein Motor treibt beide Ketten vorwaerts/rueckwaerts
 *     - DREHUNG : ein zweiter Motor erzeugt die Rotation um die Hochachse
 *
 *   Daraus folgt: Fahrt und Drehung sind unabhaengig voneinander und koennen
 *   gleichzeitig gesetzt werden (Kurvenfahrt = beides != 0).
 *
 * Alle Geschwindigkeiten werden als signed Prozentwert angegeben:
 *   -100 .. 0 .. +100
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Drehrichtung der Rotationsachse. */
typedef enum {
    TRACK_TURN_LEFT  = -1,
    TRACK_TURN_NONE  =  0,
    TRACK_TURN_RIGHT =  1,
} track_turn_dir_t;

/** Aktueller Zustand des Antriebs. */
typedef struct {
    int8_t drive_percent;   /**< -100 (rueckwaerts) .. +100 (vorwaerts) */
    int8_t turn_percent;    /**< -100 (links)       .. +100 (rechts)    */
    bool   emergency_stop;  /**< true = Antrieb gesperrt                */
} track_drive_state_t;

/**
 * @brief GPIOs und LEDC-PWM initialisieren. Motoren stehen danach still.
 */
esp_err_t track_drive_init(void);

/**
 * @brief Fahrgeschwindigkeit setzen.
 * @param percent -100 (volle Fahrt rueckwaerts) .. +100 (volle Fahrt vorwaerts)
 */
esp_err_t track_drive_set_drive(int8_t percent);

/**
 * @brief Drehgeschwindigkeit setzen.
 * @param percent -100 (Drehung links) .. +100 (Drehung rechts)
 */
esp_err_t track_drive_set_turn(int8_t percent);

/**
 * @brief Fahrt und Drehung in einem Aufruf setzen.
 */
esp_err_t track_drive_set(int8_t drive_percent, int8_t turn_percent);

/**
 * @brief Weich ausrollen lassen (H-Bruecke auf Freilauf).
 */
esp_err_t track_drive_coast(void);

/**
 * @brief Sicherer PWM-Aus-Stopp (Freilauf, keine aktive Bremsung).
 */
esp_err_t track_drive_brake(void);

/**
 * @brief Not-Aus. Sperrt alle weiteren Fahrbefehle bis track_drive_clear_stop().
 */
esp_err_t track_drive_emergency_stop(void);

/**
 * @brief Not-Aus aufheben.
 */
esp_err_t track_drive_clear_stop(void);

/**
 * @brief Aktuellen Antriebszustand auslesen.
 */
track_drive_state_t track_drive_get_state(void);

#ifdef __cplusplus
}
#endif
