/**
 * @file    track_drive.c
 * @brief   Implementierung des Kettenantriebs (L298N).
 *
 * Serialized control of independent drive and turn axes.
 */

#include "track_drive.h"

#include "esp_log.h"
#include "seeker_pins.h"
#include "driver/ledc.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static SemaphoreHandle_t s_lock;
static track_drive_state_t s_state;
static bool s_ready;

static esp_err_t axis(ledc_channel_t channel, gpio_num_t a, gpio_num_t b,
                      int value, int previous, bool inverted)
{
    if (inverted) { value = -value; previous = -previous; }
    /* Avoid cutting the PWM output every manager cycle at constant duty. */
    if (value == previous) return ESP_OK;
    unsigned magnitude = value < 0 ? -value : value;
    esp_err_t err = ESP_OK;
    bool same_direction = (value > 0 && previous > 0) || (value < 0 && previous < 0);
    if (!same_direction) {
        /* Direction change or start/stop: cut power before touching the
           H-bridge inputs. Ramp steps in one direction only update duty,
           so the motor no longer sees a 0 % pulse every 20 ms. */
        err = ledc_set_duty(SEEKER_PWM_MODE, channel, 0);
        if (err == ESP_OK) err = ledc_update_duty(SEEKER_PWM_MODE, channel);
        if (err != ESP_OK) return err;
        if (value * previous < 0) esp_rom_delay_us(1000);
        err = gpio_set_level(a, value > 0);
        if (err == ESP_OK) err = gpio_set_level(b, value < 0);
        if (err != ESP_OK || !value) return err;
    }
    /* Classic ESP32 supports 2^resolution as constant HIGH. At 100% this
       matches the verified sketch's digitalWrite(ENA/ENB, HIGH). */
    err = ledc_set_duty(SEEKER_PWM_MODE, channel, magnitude * 1024 / 100);
    if (err == ESP_OK) err = ledc_update_duty(SEEKER_PWM_MODE, channel);
    return err;
}

static esp_err_t set_locked(int drive, int turn)
{
    if (s_state.emergency_stop && (drive || turn)) return ESP_ERR_INVALID_STATE;
    esp_err_t err = axis(SEEKER_PWM_CH_DRIVE, SEEKER_PIN_DRIVE_IN1, SEEKER_PIN_DRIVE_IN2,
                         drive, s_state.drive_percent, SEEKER_DRIVE_INVERTED);
    if (err == ESP_OK) err = axis(SEEKER_PWM_CH_TURN, SEEKER_PIN_TURN_IN3, SEEKER_PIN_TURN_IN4,
                                turn, s_state.turn_percent, SEEKER_TURN_INVERTED);
    if (err != ESP_OK) {
        ledc_set_duty(SEEKER_PWM_MODE, SEEKER_PWM_CH_DRIVE, 0);
        ledc_update_duty(SEEKER_PWM_MODE, SEEKER_PWM_CH_DRIVE);
        ledc_set_duty(SEEKER_PWM_MODE, SEEKER_PWM_CH_TURN, 0);
        ledc_update_duty(SEEKER_PWM_MODE, SEEKER_PWM_CH_TURN);
        s_state.emergency_stop = true;
        drive = turn = 0;
    }
    s_state.drive_percent = drive;
    s_state.turn_percent = turn;
    return err;
}

esp_err_t track_drive_init(void)
{
    if (s_ready) return ESP_ERR_INVALID_STATE;
    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) return ESP_ERR_NO_MEM;
    gpio_config_t pins = {
        .pin_bit_mask = (1ULL << SEEKER_PIN_DRIVE_IN1) | (1ULL << SEEKER_PIN_DRIVE_IN2)
                      | (1ULL << SEEKER_PIN_TURN_IN3) | (1ULL << SEEKER_PIN_TURN_IN4)
                      | (1ULL << SEEKER_PIN_DRIVE_EN) | (1ULL << SEEKER_PIN_TURN_EN),
        .mode = GPIO_MODE_OUTPUT,
    };
    esp_err_t err = gpio_config(&pins);
    if (err != ESP_OK) return err;
    gpio_set_level(SEEKER_PIN_DRIVE_EN, 0);
    gpio_set_level(SEEKER_PIN_TURN_EN, 0);
    gpio_set_level(SEEKER_PIN_DRIVE_IN1, 0);
    gpio_set_level(SEEKER_PIN_DRIVE_IN2, 0);
    gpio_set_level(SEEKER_PIN_TURN_IN3, 0);
    gpio_set_level(SEEKER_PIN_TURN_IN4, 0);
     /* gpio_config reserved these as GPIO outputs. Hand their output paths
         back before LEDC reserves them (ESP-IDF 6 GPIO ownership tracking). */
     err = gpio_reset_pin(SEEKER_PIN_DRIVE_EN);
     if (err == ESP_OK) err = gpio_reset_pin(SEEKER_PIN_TURN_EN);
     if (err != ESP_OK) return err;
    ledc_timer_config_t timer = { .speed_mode = SEEKER_PWM_MODE,
        .timer_num = SEEKER_PWM_TIMER, .duty_resolution = SEEKER_PWM_RESOLUTION,
        .freq_hz = SEEKER_PWM_FREQ_HZ, .clk_cfg = LEDC_AUTO_CLK };
    err = ledc_timer_config(&timer);
    if (err != ESP_OK) return err;
    ledc_channel_config_t ch = { .gpio_num = SEEKER_PIN_DRIVE_EN,
        .speed_mode = SEEKER_PWM_MODE, .channel = SEEKER_PWM_CH_DRIVE,
        .timer_sel = SEEKER_PWM_TIMER, .duty = 0, .hpoint = 0 };
    err = ledc_channel_config(&ch);
    if (err != ESP_OK) return err;
    ch.gpio_num = SEEKER_PIN_TURN_EN;
    ch.channel = SEEKER_PWM_CH_TURN;
    err = ledc_channel_config(&ch);
    s_ready = err == ESP_OK;
    return err;
}

esp_err_t track_drive_set(int8_t drive, int8_t turn)
{
    if (!s_ready) return ESP_ERR_INVALID_STATE;
    if (drive < -100 || drive > 100 || turn < -100 || turn > 100) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t err = set_locked(drive, turn);
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t track_drive_set_drive(int8_t value)
{
    if (!s_ready) return ESP_ERR_INVALID_STATE;
    if (value < -100 || value > 100) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t err = set_locked(value, s_state.turn_percent);
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t track_drive_set_turn(int8_t value)
{
    if (!s_ready) return ESP_ERR_INVALID_STATE;
    if (value < -100 || value > 100) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t err = set_locked(s_state.drive_percent, value);
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t track_drive_coast(void) { return track_drive_set(0, 0); }
esp_err_t track_drive_brake(void) { return track_drive_coast(); }

esp_err_t track_drive_emergency_stop(void)
{
    if (!s_ready) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t err = set_locked(0, 0);
    s_state.emergency_stop = true;
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t track_drive_clear_stop(void)
{
    if (!s_ready) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t err = set_locked(0, 0);
    if (err == ESP_OK) s_state.emergency_stop = false;
    xSemaphoreGive(s_lock);
    return err;
}

track_drive_state_t track_drive_get_state(void)
{
    track_drive_state_t state = {0};
    if (!s_ready) return state;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    state = s_state;
    xSemaphoreGive(s_lock);
    return state;
}
