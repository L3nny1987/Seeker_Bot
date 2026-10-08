#include "imu.h"
#include "seeker_pins.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_FIFO_EN      0x23
#define REG_USER_CTRL    0x6A
#define REG_PWR_MGMT_1   0x6B
#define REG_FIFO_COUNT   0x72
#define REG_FIFO_RW      0x74
#define REG_WHO_AM_I     0x75

#define RATE_HZ          200            /* 1 kHz / (1 + SMPLRT_DIV) with DLPF on */
#define FRAME            12             /* accel xyz + gyro xyz, big-endian */
#define FIFO_SIZE        1024
#define BURST_FRAMES     20             /* frames per I2C read */
#define GYRO_LSB         65.5f          /* per deg/s at +-500 deg/s */
#define ACCEL_LSB        8192.0f        /* per g at +-4 g */
#define CAL_SAMPLES      (2 * RATE_HZ)  /* 2 s standing still */
#define WINDOW           (RATE_HZ / 2)  /* statistics / bias tracking block */
#define STILL_SPREAD_DPS 3.0f           /* max gyro spread while standing still */
#define QUIET_US         500000         /* settle time after a motor command */
#define HISTORY          256            /* 1.28 s of yaw samples */
#define FAILURES_DETACH  5
#define DEG2RAD          0.0174532925f

static const char *TAG = "imu";
static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;
static SemaphoreHandle_t s_lock;
static imu_status_t s_status;           /* protected by s_lock */
static struct { int64_t t_us; double yaw; } s_hist[HISTORY];
static unsigned s_head, s_count;
static double s_yaw;
static float s_rate;                    /* rad/s */
static float s_bias[3], s_up[3];
static int64_t s_last_ts, s_stream_start;
static uint32_t s_stream_frames;
static float s_hz = RATE_HZ;
static volatile bool s_moving;
static volatile int64_t s_motion_us;

typedef struct {
    float g_sum[3], a_sum[3], g_min[3], g_max[3], a_dev_max;
    unsigned n;
    bool disturbed;
} block_t;
static block_t s_block;

static void block_clear(block_t *b)
{
    memset(b, 0, sizeof(*b));
    for (int i = 0; i < 3; i++) { b->g_min[i] = 1e9f; b->g_max[i] = -1e9f; }
}

static void block_add(block_t *b, const float g[3], const float a[3])
{
    for (int i = 0; i < 3; i++) {
        b->g_sum[i] += g[i];
        b->a_sum[i] += a[i];
        if (g[i] < b->g_min[i]) b->g_min[i] = g[i];
        if (g[i] > b->g_max[i]) b->g_max[i] = g[i];
    }
    b->n++;
}

static bool block_still(const block_t *b)
{
    if (!b->n || b->disturbed) return false;
    float norm = 0;
    for (int i = 0; i < 3; i++) {
        if (b->g_max[i] - b->g_min[i] > STILL_SPREAD_DPS) return false;
        float m = b->a_sum[i] / b->n;
        norm += m * m;
    }
    norm = sqrtf(norm);
    return norm > 0.85f && norm < 1.15f;
}

static void normalize(float v[3])
{
    float n = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (n < 1e-6f) return;
    for (int i = 0; i < 3; i++) v[i] /= n;
}

static esp_err_t wr(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return i2c_master_transmit(s_dev, buf, sizeof(buf), 20);
}

static esp_err_t rd(uint8_t reg, uint8_t *out, size_t n)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, out, n, 20);
}

/* Caller holds s_lock. Yaw continuity is broken: drop the history. */
static void new_epoch(void)
{
    s_status.epoch++;
    s_count = 0;
    s_head = 0;
    s_last_ts = 0;
}

static bool fifo_restart(void)
{
    bool ok = wr(REG_FIFO_EN, 0) == ESP_OK && wr(REG_USER_CTRL, 0x04) == ESP_OK &&
              wr(REG_USER_CTRL, 0x40) == ESP_OK && wr(REG_FIFO_EN, 0x78) == ESP_OK;
    s_stream_start = esp_timer_get_time();
    s_stream_frames = 0;
    return ok;
}

static void detach(const char *why)
{
    if (s_dev) i2c_master_bus_rm_device(s_dev);
    s_dev = NULL;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool was = s_status.present;
    s_status.present = s_status.ready = s_status.calibrating = false;
    new_epoch();
    xSemaphoreGive(s_lock);
    if (was) ESP_LOGW(TAG, "MPU6050 getrennt (%s); weiter nur mit LIDAR", why);
}

static bool attach(void)
{
    static const uint8_t addresses[] = {0x68, 0x69};
    uint8_t address = 0;
    for (unsigned i = 0; i < sizeof(addresses) && !address; i++)
        if (i2c_master_probe(s_bus, addresses[i], 20) == ESP_OK) address = addresses[i];
    if (!address) return false;
    i2c_device_config_t config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = SEEKER_IMU_I2C_HZ,
    };
    if (i2c_master_bus_add_device(s_bus, &config, &s_dev) != ESP_OK) { s_dev = NULL; return false; }
    uint8_t who = 0;
    bool ok = rd(REG_WHO_AM_I, &who, 1) == ESP_OK && wr(REG_PWR_MGMT_1, 0x80) == ESP_OK;
    vTaskDelay(pdMS_TO_TICKS(100));
    ok = ok && wr(REG_PWR_MGMT_1, 0x01) == ESP_OK            /* wake, PLL on gyro X */
            && wr(REG_SMPLRT_DIV, 1000 / RATE_HZ - 1) == ESP_OK
            && wr(REG_CONFIG, 0x03) == ESP_OK                 /* DLPF 44 Hz */
            && wr(REG_GYRO_CONFIG, 0x08) == ESP_OK            /* +-500 deg/s */
            && wr(REG_ACCEL_CONFIG, 0x08) == ESP_OK           /* +-4 g */
            && fifo_restart();
    if (!ok) { detach("Konfiguration fehlgeschlagen"); return false; }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_status.present = true;
    s_status.ready = false;
    s_status.calibrating = true;
    s_status.address = address;
    s_status.who_am_i = who;
    new_epoch();
    block_clear(&s_block);
    xSemaphoreGive(s_lock);
    ESP_LOGI(TAG, "MPU6050 gefunden: 0x%02X, WHO_AM_I 0x%02X; kalibriere (stillstehen)", address, who);
    return true;
}

/* Caller holds s_lock. g in deg/s, a in g. */
static void sample(const float g[3], const float a[3], int64_t ts, float dt)
{
    bool quiet = !s_moving && ts - s_motion_us > QUIET_US;
    if (!s_status.ready) {
        if (!quiet) { block_clear(&s_block); return; }
        block_add(&s_block, g, a);
        if (!block_still(&s_block)) { block_clear(&s_block); return; }
        if (s_block.n < CAL_SAMPLES) return;
        for (int i = 0; i < 3; i++) {
            s_bias[i] = s_block.g_sum[i] / s_block.n;
            s_up[i] = s_block.a_sum[i] / s_block.n;
        }
        normalize(s_up);
        s_yaw = 0;
        s_status.ready = true;
        s_status.calibrating = false;
        new_epoch();
        block_clear(&s_block);
        ESP_LOGI(TAG, "Kalibriert: Hochachse %.2f %.2f %.2f", s_up[0], s_up[1], s_up[2]);
        return;
    }
    float rate = 0, dev = 0;
    for (int i = 0; i < 3; i++) {
        rate += (g[i] - s_bias[i]) * s_up[i];
        dev += (a[i] - s_up[i]) * (a[i] - s_up[i]);
    }
    s_yaw += rate * DEG2RAD * dt;
    s_rate = rate * DEG2RAD;
    if (ts <= s_last_ts) ts = s_last_ts + 1000;
    s_last_ts = ts;
    s_hist[s_head].t_us = ts;
    s_hist[s_head].yaw = s_yaw;
    s_head = (s_head + 1) % HISTORY;
    if (s_count < HISTORY) s_count++;
    s_status.rate_dps = rate;

    block_add(&s_block, g, a);
    s_block.disturbed |= !quiet;
    dev = sqrtf(dev);
    if (dev > s_block.a_dev_max) s_block.a_dev_max = dev;
    if (s_block.n < WINDOW) return;
    float mean_a[3], mean_g[3];
    for (int i = 0; i < 3; i++) {
        mean_a[i] = s_block.a_sum[i] / s_block.n;
        mean_g[i] = s_block.g_sum[i] / s_block.n;
    }
    normalize(mean_a);
    float c = mean_a[0] * s_up[0] + mean_a[1] * s_up[1] + mean_a[2] * s_up[2];
    s_status.tilt_deg = acosf(c > 1 ? 1 : c < -1 ? -1 : c) / DEG2RAD;
    s_status.shock_g = s_block.a_dev_max;
    /* Zero-velocity update: track bias drift (temperature) while parked. */
    if (block_still(&s_block)) {
        for (int i = 0; i < 3; i++) {
            s_bias[i] += 0.1f * (mean_g[i] - s_bias[i]);
            s_up[i] += 0.1f * (mean_a[i] - s_up[i]);
        }
        normalize(s_up);
    }
    s_status.bias_dps = s_bias[0] * s_up[0] + s_bias[1] * s_up[1] + s_bias[2] * s_up[2];
    block_clear(&s_block);
}

static void process(const uint8_t *buf, unsigned frames, int64_t t_count)
{
    s_stream_frames += frames;
    int64_t elapsed = t_count - s_stream_start;
    if (elapsed > 3000000) {
        float hz = s_stream_frames * 1e6f / elapsed;
        s_hz = hz < RATE_HZ * 0.95f ? RATE_HZ * 0.95f : hz > RATE_HZ * 1.05f ? RATE_HZ * 1.05f : hz;
    }
    float dt = 1.0f / s_hz;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (unsigned k = 0; k < frames; k++) {
        const uint8_t *p = buf + k * FRAME;
        float a[3], g[3];
        for (int i = 0; i < 3; i++) {
            a[i] = (int16_t)(p[2 * i] << 8 | p[2 * i + 1]) / ACCEL_LSB;
            g[i] = (int16_t)(p[6 + 2 * i] << 8 | p[7 + 2 * i]) / GYRO_LSB;
        }
        sample(g, a, t_count - (int64_t)((frames - 1 - k) * dt * 1e6f), dt);
    }
    s_status.samples += frames;
    s_status.sample_hz = s_hz;
    s_status.yaw_deg = (float)fmod(s_yaw / DEG2RAD, 360.0);
    xSemaphoreGive(s_lock);
}

static void fault(const char *why, unsigned *failures)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_status.errors++;
    new_epoch();
    xSemaphoreGive(s_lock);
    if (++*failures >= FAILURES_DETACH) detach(why);
    else fifo_restart();
}

static void imu_task(void *arg)
{
    (void)arg;
    static uint8_t buf[FIFO_SIZE];
    unsigned failures = 0;
    while (true) {
        if (!s_dev) {
            /* Hot-plug: probe once per second until the sensor answers. */
            if (!attach()) { vTaskDelay(pdMS_TO_TICKS(1000)); continue; }
            failures = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        uint8_t c[2];
        if (rd(REG_FIFO_COUNT, c, 2) != ESP_OK) { fault("I2C-Fehler", &failures); continue; }
        int64_t t_count = esp_timer_get_time();
        unsigned count = (unsigned)c[0] << 8 | c[1];
        if (count > FIFO_SIZE - FRAME) {
            /* Overflow loses samples and frame alignment: restart cleanly. */
            xSemaphoreTake(s_lock, portMAX_DELAY);
            s_status.overflows++;
            new_epoch();
            xSemaphoreGive(s_lock);
            if (!fifo_restart()) fault("FIFO-Reset", &failures);
            continue;
        }
        unsigned frames = count / FRAME, got = 0;
        bool ok = true;
        while (ok && got < frames) {
            unsigned n = frames - got > BURST_FRAMES ? BURST_FRAMES : frames - got;
            ok = rd(REG_FIFO_RW, buf + got * FRAME, n * FRAME) == ESP_OK;
            got += n;
        }
        if (!ok) { fault("I2C-Fehler", &failures); continue; }
        failures = 0;
        if (frames) process(buf, frames, t_count);
    }
}

esp_err_t imu_init(void)
{
    if (s_lock) return ESP_ERR_INVALID_STATE;
    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) return ESP_ERR_NO_MEM;
    i2c_master_bus_config_t bus = {
        .i2c_port = SEEKER_IMU_I2C_PORT,
        .sda_io_num = SEEKER_PIN_IMU_SDA,
        .scl_io_num = SEEKER_PIN_IMU_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,   /* GY-521 has 4.7k on board too */
    };
    esp_err_t err = i2c_new_master_bus(&bus, &s_bus);
    if (err != ESP_OK) return err;
    /* Core 1 above the mapper (4) so FIFO reads are never starved. */
    return xTaskCreatePinnedToCore(imu_task, "imu", 3072, NULL, 5, NULL, 1) == pdPASS
           ? ESP_OK : ESP_ERR_NO_MEM;
}

void imu_set_motion(bool moving)
{
    if (moving) s_motion_us = esp_timer_get_time();
    s_moving = moving;
}

bool imu_yaw_at(int64_t t_us, double *yaw, uint32_t *epoch)
{
    if (!s_lock) return false;
    bool ok = false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_status.ready && s_count >= 2) {
        unsigned newest = (s_head + HISTORY - 1) % HISTORY;
        unsigned oldest = (s_head + HISTORY - s_count) % HISTORY;
        if (t_us >= s_hist[newest].t_us) {
            int64_t ahead = t_us - s_hist[newest].t_us;
            if (ahead <= 50000) { *yaw = s_hist[newest].yaw + s_rate * ahead * 1e-6; ok = true; }
        } else if (t_us >= s_hist[oldest].t_us) {
            for (unsigned k = 1; k < s_count; k++) {
                unsigned i = (newest + HISTORY - k) % HISTORY, j = (i + 1) % HISTORY;
                if (s_hist[i].t_us > t_us) continue;
                double f = (double)(t_us - s_hist[i].t_us) / (double)(s_hist[j].t_us - s_hist[i].t_us);
                *yaw = s_hist[i].yaw + f * (s_hist[j].yaw - s_hist[i].yaw);
                ok = true;
                break;
            }
        }
        *epoch = s_status.epoch;
    }
    xSemaphoreGive(s_lock);
    return ok;
}

imu_status_t imu_status(void)
{
    imu_status_t status = {0};
    if (!s_lock) return status;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    status = s_status;
    xSemaphoreGive(s_lock);
    return status;
}
