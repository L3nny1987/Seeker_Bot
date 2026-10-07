/**
 * @file    seeker_pins.h
 * @brief   Zentrale Hardware-/Pinbelegung des Seeker_Bot.
 *
 * ACHTUNG: Dies ist die EINZIGE Stelle, an der GPIO-Nummern definiert werden.
 *          Niemals GPIO-Nummern direkt im Treiber- oder Anwendungscode nutzen.
 *
 * Mechanik-Hinweis:
 *   Der Roboter faehrt auf Panzerketten, besitzt aber KEINEN klassischen
 *   Differentialantrieb mit zwei unabhaengigen Kettenmotoren.
 *     - Motor A (L298N Kanal A) -> FAHRT    (vorwaerts / rueckwaerts)
 *     - Motor B (L298N Kanal B) -> DREHUNG  (links / rechts)
 */

#pragma once

#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 *  L298N - Kanal A  =  FAHRMOTOR (vorwaerts / rueckwaerts)
 *  (in der Verkabelungsdoku als "Motor links" bezeichnet, OUT1 / OUT2)
 * ========================================================================= */
#define SEEKER_PIN_DRIVE_EN     GPIO_NUM_25   /**< ENA - PWM Geschwindigkeit  */
#define SEEKER_PIN_DRIVE_IN1    GPIO_NUM_26   /**< IN1 - Richtung             */
#define SEEKER_PIN_DRIVE_IN2    GPIO_NUM_27   /**< IN2 - Richtung             */

/* =========================================================================
 *  L298N - Kanal B  =  DREHMOTOR (Rotation links / rechts)
 *  (in der Verkabelungsdoku als "Motor rechts" bezeichnet, OUT3 / OUT4)
 * ========================================================================= */
#define SEEKER_PIN_TURN_EN      GPIO_NUM_14   /**< ENB - PWM Geschwindigkeit  */
#define SEEKER_PIN_TURN_IN3     GPIO_NUM_32   /**< IN3 - Richtung             */
#define SEEKER_PIN_TURN_IN4     GPIO_NUM_33   /**< IN4 - Richtung             */

/* Richtungsumkehr per Software, falls ein Motor verkehrt herum laeuft. */
#define SEEKER_DRIVE_INVERTED   0
/* API: negative = left. Verified sketch: left IN3 HIGH / IN4 LOW. */
#define SEEKER_TURN_INVERTED    1

/* =========================================================================
 *  FHL-LD20 LiDAR
 *
 *  Kabelbelegung laut aktueller Verkabelung:
 *    Schwarz -> +5 V
 *    Gruen   -> GND
 *    Weiss   -> GPIO 18   (Datenleitung des LiDAR -> RX des ESP32)
 *    Rot     -> OFFEN     (PWM-Eingang, nicht Datenleitung)
 *  Herstellerabbildung: https://wiki.youyeetoo.com/en/Lidar/LD20
 *  Abschnitt 3.1.1. Kabelchargen koennen abweichen: Steckerbelegung pruefen.
 *
 *  Der LD20 sendet seine Messdaten per UART (3.3 V TTL), typ. 230400 Baud, 8N1.
 *  Es wird nur empfangen -> TX bleibt unbelegt.
 * ========================================================================= */
#define SEEKER_PIN_LIDAR_RX     GPIO_NUM_18   /**< Weiss: LiDAR TX -> ESP32 RX */
#define SEEKER_PIN_LIDAR_TX     (-1)          /**< nicht verwendet            */
#define SEEKER_PIN_LIDAR_PWM    (-1)          /**< Rot: PWM-Eingang OFFEN      */

#define SEEKER_LIDAR_UART_NUM   UART_NUM_2
#define SEEKER_LIDAR_BAUDRATE   230400

/* =========================================================================
 *  PWM (LEDC) Einstellungen fuer den L298N
 * ========================================================================= */
#define SEEKER_PWM_TIMER        LEDC_TIMER_0
#define SEEKER_PWM_MODE         LEDC_LOW_SPEED_MODE
#define SEEKER_PWM_CH_DRIVE     LEDC_CHANNEL_0
#define SEEKER_PWM_CH_TURN      LEDC_CHANNEL_1
#define SEEKER_PWM_RESOLUTION   LEDC_TIMER_10_BIT   /**< 0 .. 1023           */
#define SEEKER_PWM_FREQ_HZ      20000               /**< 20 kHz: lautlos     */

/**
 * Der L298N benoetigt eine Mindest-Duty, damit sich der Motor ueberhaupt
 * dreht (Haftreibung + ca. 2 V Spannungsabfall ueber die H-Bruecke).
 */
#define SEEKER_PWM_MIN_PERCENT  25
#define SEEKER_PWM_MAX_PERCENT  100

/* Commission with wheels raised. This mounting places raw 180 at the front.
 * No odometry: these are geometric margins, not braking guarantees. */
#define SEEKER_LIDAR_FRONT_DEG       180
#define SEEKER_LIDAR_CLOCKWISE       1
#define SEEKER_LIDAR_MIN_INTENSITY   10
#define SEEKER_LIDAR_STALE_MS        400
/* Sensor-Gesundheit, keine Freiraum-Annahme: fehlende Rueckmeldungen
   (z. B. > 8 m oder schwarze Flaechen) werden im Planer als unsicher
   behandelt und nur langsam/kurz befahren. */
#define SEEKER_AUTO_MIN_SECTORS      180
#define SEEKER_MANUAL_STOP_MM        100
#define SEEKER_MANUAL_ROTATE_MM      100
#define SEEKER_STOP_DISTANCE_MM      200
#define SEEKER_ROTATE_CLEARANCE_MM   100
#define SEEKER_SLOW_DISTANCE_MM      400
#define SEEKER_MANUAL_MAX_PERCENT    100
#define SEEKER_AUTO_DRIVE_PERCENT    90
#define SEEKER_AUTO_TURN_PERCENT     100
#define SEEKER_AUTO_SLOW_PERCENT     85
#define SEEKER_RAMP_MS               1000
#define SEEKER_REVERSE_MS            800
#define SEEKER_TURN_MS               1600
#define SEEKER_RESUME_MARGIN_MM      50
/* Fahrkorridor-Breite fuer die Auto-Planung (LIDAR-Mitte +/- Breite/2). */
#define SEEKER_ROBOT_WIDTH_MM        250
#define SEEKER_AUTO_LIMIT_S          300
/* Steuerungs-Lease: ohne Heartbeat stoppt Auto nach LINK_PAUSE sofort und
   setzt nach kurzer WLAN-Stoerung fort; nach CONTROL_TIMEOUT ist die
   Steuerung verloren (IDLE, kein automatischer Neustart). */
#define SEEKER_LINK_PAUSE_MS         1500
#define SEEKER_CONTROL_TIMEOUT_MS    5000
#define SEEKER_MANUAL_TIMEOUT_MS     350
#define SEEKER_MANAGER_PERIOD_MS     20

#ifdef __cplusplus
}
#endif
