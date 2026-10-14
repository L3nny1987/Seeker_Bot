#include "diag.h"
#include "reset_diagnostics.h"
#include "robot_manager.h"
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"

#define DIAG_MAGIC   0x5342474Cu
#define ALLOC_MAGIC  0x5342414Cu
#define LOG_SIZE     3072
#define HISTORY      8
#define REASONS      16
#define NVS_NS       "diag"

/* State before a reset; valid = RTC memory survived. */
typedef struct {
    uint32_t boot, uptime_s, min_heap;
    uint8_t reason, valid, mode, online;
    int8_t drive, turn;
    uint8_t pad[2];
} diag_entry_t;

typedef struct {
    uint32_t magic;
    uint32_t head, wrapped;
    uint32_t uptime_ms, min_heap;
    int8_t mode, drive, turn, online;
    char log[LOG_SIZE];
    uint32_t magic_end;
    /* Appended: older firmware's RTC log layout remains readable. */
    uint32_t alloc_magic;
    diag_memory_t memory;
} rtc_diag_t;

static RTC_NOINIT_ATTR rtc_diag_t s_rtc;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static vprintf_like_t s_original;
static char s_prev_log[LOG_SIZE];
static size_t s_prev_len;
static uint32_t s_boots;
static uint16_t s_counts[REASONS];
static diag_entry_t s_history[HISTORY];      /* newest first */
static diag_memory_t s_previous_memory;

/* The allocator may fail in low-level code. Do not log, allocate, inspect
   heaps or dereference flash strings here. Record a small RTC snapshot only. */
static void IRAM_ATTR allocation_failed(size_t size, uint32_t caps, const char *function_name)
{
    (void)function_name;
    portENTER_CRITICAL_SAFE(&s_mux);
    s_rtc.memory.failures++;
    s_rtc.memory.size = (uint32_t)size;
    s_rtc.memory.caps = caps;
    s_rtc.memory.uptime_ms = s_rtc.uptime_ms;
    portEXIT_CRITICAL_SAFE(&s_mux);
}

diag_memory_t diag_memory_status(void)
{
    taskENTER_CRITICAL(&s_mux);
    diag_memory_t memory = s_rtc.memory;
    taskEXIT_CRITICAL(&s_mux);
    return memory;
}

static size_t linearize(char *out, size_t size)
{
    size_t n = 0;
    taskENTER_CRITICAL(&s_mux);
    if (s_rtc.wrapped) {
        size_t tail = LOG_SIZE - s_rtc.head;
        if (tail > size) tail = size;
        memcpy(out, s_rtc.log + s_rtc.head, tail);
        n = tail;
    }
    size_t head = s_rtc.head < size - n ? s_rtc.head : size - n;
    memcpy(out + n, s_rtc.log, head);
    n += head;
    taskEXIT_CRITICAL(&s_mux);
    return n;
}

/* Copies every ESP_LOG line into the RTC ring, then prints as before. */
static int log_hook(const char *format, va_list args)
{
    char line[160];
    va_list copy;
    va_copy(copy, args);
    int n = vsnprintf(line, sizeof(line), format, copy);
    va_end(copy);
    if (n > 0) {
        if (n >= (int)sizeof(line)) { n = sizeof(line) - 1; line[n - 1] = '\n'; }
        taskENTER_CRITICAL(&s_mux);
        for (int i = 0; i < n; i++) {
            s_rtc.log[s_rtc.head++] = line[i];
            if (s_rtc.head >= LOG_SIZE) { s_rtc.head = 0; s_rtc.wrapped = 1; }
        }
        taskEXIT_CRITICAL(&s_mux);
    }
    return s_original ? s_original(format, args) : n;
}

static void save(void)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NS, NVS_READWRITE, &handle) != ESP_OK) return;
    nvs_set_u32(handle, "boots", s_boots);
    nvs_set_blob(handle, "counts", s_counts, sizeof(s_counts));
    nvs_set_blob(handle, "history", s_history, sizeof(s_history));
    nvs_commit(handle);
    nvs_close(handle);
}

void diag_init(void)
{
    esp_reset_reason_t reason = esp_reset_reason();
    bool have = s_rtc.magic == DIAG_MAGIC && s_rtc.magic_end == DIAG_MAGIC &&
                s_rtc.head < LOG_SIZE && reason != ESP_RST_POWERON;
    diag_entry_t entry = { .reason = (uint8_t)reason };
    if (have) {
        if (s_rtc.alloc_magic == ALLOC_MAGIC) s_previous_memory = s_rtc.memory;
        s_prev_len = linearize(s_prev_log, sizeof(s_prev_log));
        entry.valid = 1;
        entry.uptime_s = s_rtc.uptime_ms / 1000;
        entry.min_heap = s_rtc.min_heap;
        entry.mode = (uint8_t)s_rtc.mode;
        entry.drive = s_rtc.drive;
        entry.turn = s_rtc.turn;
        entry.online = (uint8_t)s_rtc.online;
    }
    memset(&s_rtc, 0, sizeof(s_rtc));
    s_rtc.magic = s_rtc.magic_end = DIAG_MAGIC;
    s_rtc.alloc_magic = ALLOC_MAGIC;

    nvs_handle_t handle;
    if (nvs_open(NVS_NS, NVS_READONLY, &handle) == ESP_OK) {
        size_t size = sizeof(s_counts);
        nvs_get_u32(handle, "boots", &s_boots);
        if (nvs_get_blob(handle, "counts", s_counts, &size) != ESP_OK || size != sizeof(s_counts))
            memset(s_counts, 0, sizeof(s_counts));
        size = sizeof(s_history);
        if (nvs_get_blob(handle, "history", s_history, &size) != ESP_OK || size != sizeof(s_history))
            memset(s_history, 0, sizeof(s_history));
        nvs_close(handle);
    }
    entry.boot = ++s_boots;
    if (s_counts[reason < REASONS ? reason : 0] < UINT16_MAX) s_counts[reason < REASONS ? reason : 0]++;
    memmove(s_history + 1, s_history, sizeof(s_history) - sizeof(s_history[0]));
    s_history[0] = entry;
    save();
    s_original = esp_log_set_vprintf(log_hook);
    ESP_ERROR_CHECK(heap_caps_register_failed_alloc_callback(allocation_failed));
}

void diag_snapshot(int mode, int drive, int turn, bool online)
{
    s_rtc.uptime_ms = (uint32_t)(esp_timer_get_time() / 1000);
    s_rtc.min_heap = esp_get_minimum_free_heap_size();
    s_rtc.mode = (int8_t)mode;
    s_rtc.drive = (int8_t)drive;
    s_rtc.turn = (int8_t)turn;
    s_rtc.online = online;
}

size_t diag_summary(char *out, size_t size)
{
    size_t n = 0;
#define ADD(...) do { int r = snprintf(out + n, size - n, __VA_ARGS__); \
                      if (r > 0) n = (size_t)r < size - n ? n + r : size - 1; } while (0)
    ADD("Seeker_Bot v%s - Diagnose\nLaufzeit %lld s, Start Nr. %lu\n\nNeustart-Gruende (gesamt):",
        SEEKER_BOT_VERSION, esp_timer_get_time() / 1000000, (unsigned long)s_boots);
    for (int i = 0; i < REASONS; i++)
        if (s_counts[i]) ADD(" %s %u,", seeker_reset_name((esp_reset_reason_t)i), s_counts[i]);
    ADD("\n\nLetzte Neustarts (neueste zuerst):\n");
    for (int i = 0; i < HISTORY && s_history[i].boot; i++) {
        const diag_entry_t *e = &s_history[i];
        ADD("  Start %lu: %-16s ", (unsigned long)e->boot, seeker_reset_name((esp_reset_reason_t)e->reason));
        if (e->valid)
            ADD("vorher %lu s gelaufen, Modus %s, Fahrt %d, Drehung %d, WLAN %s, min. Heap %lu\n",
                (unsigned long)e->uptime_s, robot_mode_name((robot_mode_t)e->mode), e->drive, e->turn,
                e->online ? "ja" : "nein", (unsigned long)e->min_heap);
        else
            ADD("kein Zustand (Strom war ganz weg oder Start ohne Log)\n");
    }
    diag_memory_t memory = diag_memory_status();
    ADD("\nSpeicherreservierungen fehlgeschlagen: aktuell %lu, vor letztem Reset %lu\n",
        (unsigned long)memory.failures, (unsigned long)s_previous_memory.failures);
    if (memory.failures)
        ADD("  Letzter Fehler: %lu Bytes, caps 0x%08lX, bei %lu ms\n",
            (unsigned long)memory.size, (unsigned long)memory.caps, (unsigned long)memory.uptime_ms);
    if (s_previous_memory.failures)
        ADD("  Vor Reset: %lu Bytes, caps 0x%08lX, bei %lu ms\n",
            (unsigned long)s_previous_memory.size, (unsigned long)s_previous_memory.caps,
            (unsigned long)s_previous_memory.uptime_ms);
#undef ADD
    return n;
}

const char *diag_previous_log(size_t *len)
{
    *len = s_prev_len;
    return s_prev_log;
}

size_t diag_current_log(char *out, size_t size) { return linearize(out, size); }

void diag_clear(void)
{
    s_boots = 0;
    memset(s_counts, 0, sizeof(s_counts));
    memset(s_history, 0, sizeof(s_history));
    save();
}
