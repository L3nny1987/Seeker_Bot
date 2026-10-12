#include "mapper.h"
#include "imu.h"
#include "ld20_lidar.h"
#include "seeker_pins.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

#define TARGET_EVERY       5        /* scans between target updates (~0.5 s) */
#define TARGET_TIMEOUT_MS  20000    /* driving that long without reaching it: give up */
#define TARGET_CLOSE_MM    600.0f   /* frontier still open when this close: give up */
#define TARGET_PULL        2500     /* heading bonus toward the waypoint */
#define TARGET_PULL_DEG    60.0f
#define WEAK_INTEGRATED    5        /* map younger than this: weak */
#define WEAK_BAD_MATCHES   1        /* an unverified position must not guide motion */
#define RECOVERY_MATCHES   3        /* avoid stop/start on alternating good/bad scans */

static const char *TAG = "mapper";
static slam_t s_slam;                  /* ~43 KB, written only by the mapper task */
static SemaphoreHandle_t s_lock;
static mapper_info_t s_info;
static pose_feedback_t s_feedback;
static uint32_t s_pose_sequence;
static unsigned s_good_matches;
static volatile bool s_moving, s_reset;
static uint8_t s_avoid[SLAM_PLAN_SIZE * SLAM_PLAN_SIZE];
static slam_target_t s_target;
static uint16_t s_avoided;
static volatile bool s_goal_active;
static volatile float s_goal_x, s_goal_y;
static volatile int s_drive, s_turn;
static const esp_partition_t *s_rec_part;
static volatile bool s_rec_active, s_rec_erasing, s_rec_start;
static volatile uint32_t s_rec_count;

static void publish(uint32_t update_ms)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_info.valid = s_slam.initialized && !s_slam.lost && !s_slam.bad_matches && !s_slam.degenerate;
    s_info.lost = s_slam.lost;
    s_info.degenerate = s_slam.degenerate;
    /* Remain stopped until the map has matured and matches are healthy. */
    s_info.weak = !s_slam.initialized || s_slam.degenerate ||
                  s_slam.integrated < WEAK_INTEGRATED || s_slam.bad_matches >= WEAK_BAD_MATCHES ||
                  s_good_matches < RECOVERY_MATCHES;
    s_info.x = s_slam.pose.x;
    s_info.y = s_slam.pose.y;
    s_info.theta = s_slam.pose.theta;
    s_info.quality = s_slam.quality;
    s_info.scans = s_slam.scans;
    s_info.integrated = s_slam.integrated;
    s_info.rejected = s_slam.rejected;
    s_info.known_cells = s_slam.known_cells;
    s_info.path_count = s_slam.path_count;
    s_info.update_ms = update_ms;
    s_info.feedback = s_feedback;
    s_info.scan_sequence = s_pose_sequence;
    s_info.imu = s_slam.imu_used;
    s_info.heading_sigma = slam_heading_sigma(&s_slam);
    s_info.gyro_bias = s_slam.gyro_bias;
    s_info.target = s_target;
    s_info.avoided = s_avoided;
    xSemaphoreGive(s_lock);
}

static void avoid(uint16_t cell)
{
    int cx = cell % SLAM_PLAN_SIZE, cy = cell / SLAM_PLAN_SIZE;
    for (int y = cy - 1; y <= cy + 1; y++)
        for (int x = cx - 1; x <= cx + 1; x++)
            if (x >= 0 && y >= 0 && x < SLAM_PLAN_SIZE && y < SLAM_PLAN_SIZE && !s_avoid[y * SLAM_PLAN_SIZE + x]) {
                s_avoid[y * SLAM_PLAN_SIZE + x] = 1;
                s_avoided++;
            }
}

/* Frontier exploration: commit to the nearest reachable unknown space and
   follow the map path to it until it is reached, explored or given up;
   re-choosing every update made the robot weave between targets. Targets
   that cannot be reached or resolved are blacklisted. */
static void update_target(uint64_t now_ms, bool reset)
{
    static slam_target_t current;
    static uint64_t last_ms, driven_ms;
    if (reset) { current.valid = false; last_ms = now_ms; driven_ms = 0; }
    if (!s_slam.initialized || s_slam.lost) { s_target.valid = false; return; }
    if (s_goal_active) {
        /* Go-to-point: shortest path over the map instead of frontiers. */
        slam_target_t path;
        if (!slam_plan_path(&s_slam, s_slam.pose, s_goal_x, s_goal_y, &path)) path.valid = false;
        current.valid = false;
        s_target = path;
        return;
    }
    if (current.valid && s_moving) driven_ms += now_ms - last_ms;
    last_ms = now_ms;
    slam_target_t route;
    bool keep = current.valid;
    if (keep && driven_ms > TARGET_TIMEOUT_MS) {
        ESP_LOGI(TAG, "Ziel %.0f/%.0f nicht erreicht: gesperrt", current.x, current.y);
        avoid(current.cell);
        keep = false;
    } else if (keep && hypotf(current.x - s_slam.pose.x, current.y - s_slam.pose.y) < TARGET_CLOSE_MM) {
        /* Reached (or nearly): never return to this frontier as a target. */
        if (current.frontier) avoid(current.cell);
        keep = false;
    } else if (keep && current.frontier && !slam_frontier_open(&s_slam, current.cell)) {
        keep = false;                    /* explored on the way */
    }
    if (keep && !slam_plan_path(&s_slam, s_slam.pose, current.x, current.y, &route)) {
        avoid(current.cell);             /* blocked meanwhile */
        keep = false;
    }
    if (!keep) {
        slam_target_t next;
        driven_ms = 0;
        if (!slam_explore_target(&s_slam, s_slam.pose, s_avoid, &next) || !next.valid ||
            !slam_plan_path(&s_slam, s_slam.pose, next.x, next.y, &route)) {
            current.valid = false;
            s_target.valid = false;
            return;
        }
        current = next;
    }
    route.frontier = current.frontier;
    route.cell = current.cell;
    s_target = route;
}

/* Gyro yaw at the scan end (LD20 timestamp = end of revolution) and the
   rate during the revolution; deltas only within one IMU epoch. */
typedef struct { bool ok; double end, start; uint32_t epoch; } imu_raw_t;

static imu_raw_t imu_raw(const ld20_scan_t *scan, float period_ms)
{
    imu_raw_t r = {0};
    uint32_t epoch_start = 0;
    r.ok = imu_yaw_at(scan->timestamp_us, &r.end, &r.epoch) &&
           imu_yaw_at(scan->timestamp_us - (int64_t)(period_ms * 1000), &r.start, &epoch_start) &&
           r.epoch == epoch_start;
    return r;
}

static slam_imu_t imu_input(imu_raw_t r, float period_ms)
{
    static double last_yaw;
    static uint32_t last_epoch;
    static bool have_last;
    slam_imu_t in = {0};
    if (r.ok && have_last && last_epoch == r.epoch) {
        in.valid = true;
        in.dtheta = (float)(r.end - last_yaw);
        in.omega = (float)((r.end - r.start) / (period_ms * 0.001f));
    }
    have_last = r.ok;
    last_yaw = r.end;
    last_epoch = r.epoch;
    return in;
}

static void put32(uint8_t *p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }
static void putf(uint8_t *p, float v) { uint32_t u; memcpy(&u, &v, 4); put32(p, u); }
static int8_t clamp8(int v) { return (int8_t)(v > 127 ? 127 : v < -127 ? -127 : v); }

/* Record layout (little-endian, MAPPER_RECORD_SIZE bytes):
   0 "SR" magic, 2 u8 version 1, 3 u8 flags (1 imu ok, 2 moving),
   4 u32 scan sequence, 8 u32 scan time ms, 12 f32 period ms,
   16 f32 yaw end rad, 20 f32 yaw start rad, 24 u32 imu epoch,
   28 i8 drive, 29 i8 turn, 30 u16 valid sectors,
   32 f32 x mm, 36 f32 y mm, 40 f32 theta rad (on-device pose after the
   update), 44 f32 quality,
   48 u16 distance mm[360] (raw LiDAR angle order). */
static void record(const ld20_scan_t *scan, float period_ms, imu_raw_t r, bool moving)
{
    static uint8_t rec[MAPPER_RECORD_SIZE];
    uint32_t offset = s_rec_count * MAPPER_RECORD_SIZE;
    if (offset + MAPPER_RECORD_SIZE > s_rec_part->size) {
        s_rec_active = false;
        ESP_LOGW(TAG, "Aufzeichnung voll (%lu Scans)", (unsigned long)s_rec_count);
        return;
    }
    memset(rec, 0, sizeof(rec));
    rec[0] = 'S'; rec[1] = 'R'; rec[2] = 1;
    rec[3] = (r.ok ? 1 : 0) | (moving ? 2 : 0);
    put32(rec + 4, scan->sequence);
    put32(rec + 8, (uint32_t)(scan->timestamp_us / 1000));
    putf(rec + 12, period_ms);
    putf(rec + 16, (float)r.end);
    putf(rec + 20, (float)r.start);
    put32(rec + 24, r.epoch);
    rec[28] = (uint8_t)clamp8(s_drive);
    rec[29] = (uint8_t)clamp8(s_turn);
    rec[30] = scan->valid_sectors & 0xff; rec[31] = scan->valid_sectors >> 8;
    putf(rec + 32, s_slam.pose.x);
    putf(rec + 36, s_slam.pose.y);
    putf(rec + 40, s_slam.pose.theta);
    putf(rec + 44, s_slam.quality);
    for (int i = 0; i < 360; i++) {
        rec[48 + 2 * i] = scan->distance_mm[i] & 0xff;
        rec[49 + 2 * i] = scan->distance_mm[i] >> 8;
    }
    if (esp_partition_write(s_rec_part, offset, rec, sizeof(rec)) != ESP_OK) {
        s_rec_active = false;
        ESP_LOGE(TAG, "Aufzeichnung: Schreibfehler");
        return;
    }
    s_rec_count++;
}

/* Erasing stalls flash access for seconds: done here, robot idle. */
static void record_begin(void)
{
    s_rec_start = false;
    s_rec_part = esp_ota_get_next_update_partition(NULL);
    if (!s_rec_part) { s_rec_erasing = false; return; }
    s_rec_count = 0;
    if (esp_partition_erase_range(s_rec_part, 0, s_rec_part->size) != ESP_OK) {
        s_rec_erasing = false;
        ESP_LOGE(TAG, "Aufzeichnung: Loeschen fehlgeschlagen");
        return;
    }
    s_rec_erasing = false;
    s_rec_active = true;
    ESP_LOGW(TAG, "Aufzeichnung gestartet (%s, max %lu Scans)", s_rec_part->label,
             (unsigned long)(s_rec_part->size / MAPPER_RECORD_SIZE));
}

static void mapper_task(void *arg)
{
    (void)arg;
    static ld20_scan_t scan;
    uint32_t last_sequence = 0;
    bool goal_active = false;
    float goal_x = 0, goal_y = 0;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(20));
        if (s_goal_active != goal_active || goal_x != s_goal_x || goal_y != s_goal_y) {
            /* New goal or mode change: never publish a stale target. */
            goal_active = s_goal_active;
            goal_x = s_goal_x;
            goal_y = s_goal_y;
            s_target.valid = false;
            update_target((uint64_t)(esp_timer_get_time() / 1000), false);
            publish(0);
        }
        if (s_reset) {
            /* Readers only see a cleared map; no allocation, no lock needed. */
            slam_reset(&s_slam);
            memset(s_avoid, 0, sizeof(s_avoid));
            s_avoided = 0;
            s_target.valid = false;
            s_feedback = (pose_feedback_t){0};
            s_pose_sequence = 0;
            s_good_matches = 0;
            update_target((uint64_t)(esp_timer_get_time() / 1000), true);
            s_reset = false;
            publish(0);
            ESP_LOGI(TAG, "Karte zurueckgesetzt");
        }
        if (s_rec_start) record_begin();
        if (ld20_lidar_get_scan(&scan, 0) != ESP_OK || scan.sequence == last_sequence) continue;
        last_sequence = scan.sequence;
        if (scan.valid_sectors < SEEKER_AUTO_MIN_SECTORS) continue;
        float period_ms = scan.speed_rpm ? 60000.0f / scan.speed_rpm : 100.0f;
        imu_raw_t raw = imu_raw(&scan, period_ms);
        slam_imu_t imu = imu_input(raw, period_ms);
        bool moving = s_moving;
        int64_t start = esp_timer_get_time();
        bool matched = slam_update(&s_slam, scan.distance_mm, SEEKER_LIDAR_FRONT_DEG,
            SEEKER_LIDAR_CLOCKWISE, period_ms, (uint64_t)(scan.timestamp_us / 1000), moving, &imu);
        if (!matched) s_good_matches = 0;
        else if (s_good_matches < RECOVERY_MATCHES) s_good_matches++;
        s_feedback = (pose_feedback_t){scan.timestamp_us, raw.end, raw.epoch, raw.ok};
        s_pose_sequence = scan.sequence;
        if (s_rec_active) record(&scan, period_ms, raw, moving);
        if (s_slam.scans % TARGET_EVERY == 0) update_target((uint64_t)(scan.timestamp_us / 1000), false);
        publish((uint32_t)((esp_timer_get_time() - start) / 1000));
    }
}

esp_err_t mapper_init(void)
{
    if (s_lock) return ESP_ERR_INVALID_STATE;
    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) return ESP_ERR_NO_MEM;
    slam_reset(&s_slam);
    /* Core 1, below the manager (8) so motor control always wins. */
    return xTaskCreatePinnedToCore(mapper_task, "mapper", 4096, NULL, 4, NULL, 1) == pdPASS
           ? ESP_OK : ESP_ERR_NO_MEM;
}

void mapper_set_motion(int drive, int turn)
{
    s_drive = drive;
    s_turn = turn;
    s_moving = drive || turn;
    imu_set_motion(s_moving);
}

void mapper_set_goal(bool active, float x, float y)
{
    s_goal_x = x;
    s_goal_y = y;
    s_goal_active = active;
}

mapper_info_t mapper_info(void)
{
    mapper_info_t info = {0};
    if (!s_lock) return info;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    info = s_info;
    xSemaphoreGive(s_lock);
    return info;
}

void mapper_explore_bonus(int16_t out[SLAM_HEADINGS])
{
    mapper_info_t info = mapper_info();
    if (!info.valid) { memset(out, 0, SLAM_HEADINGS * sizeof(out[0])); return; }
    if (s_goal_active) memset(out, 0, SLAM_HEADINGS * sizeof(out[0]));   /* path only */
    else slam_explore_bonus(&s_slam, (slam_pose_t){info.x, info.y, info.theta}, out);
    if (!info.target.valid) return;
    /* Robot-frame bearing of the waypoint (+ = right); behind the robot
       counts as the nearer side so it turns toward it. */
    float a = atan2f(info.target.way_y - info.y, info.target.way_x - info.x) - info.theta;
    float error = -atan2f(sinf(a), cosf(a)) * 57.29578f;
    if (error > 90) error = 90;
    if (error < -90) error = -90;
    for (int k = 0; k < SLAM_HEADINGS; k++) {
        float diff = fabsf(k * 10 - 90 - error);
        if (diff < TARGET_PULL_DEG) out[k] += (int16_t)(TARGET_PULL * (1 - diff / TARGET_PULL_DEG));
    }
}

void mapper_request_reset(void) { s_reset = true; }

const int8_t *mapper_grid(void) { return s_slam.grid; }

uint16_t mapper_path(int16_t (*out)[2], uint16_t max) { return slam_path_copy(&s_slam, out, max); }

esp_err_t mapper_record_start(void)
{
    if (s_rec_erasing || s_rec_start) return ESP_ERR_INVALID_STATE;
    s_rec_active = false;
    s_rec_erasing = true;
    s_rec_start = true;
    return ESP_OK;
}

void mapper_record_stop(void)
{
    if (s_rec_active) ESP_LOGW(TAG, "Aufzeichnung beendet (%lu Scans)", (unsigned long)s_rec_count);
    s_rec_active = false;
}

mapper_record_info_t mapper_record_info(void)
{
    const esp_partition_t *part = s_rec_part ? s_rec_part : esp_ota_get_next_update_partition(NULL);
    return (mapper_record_info_t){
        .active = s_rec_active,
        .erasing = s_rec_erasing,
        .records = s_rec_count,
        .capacity = part ? part->size / MAPPER_RECORD_SIZE : 0,
    };
}

bool mapper_record_read(uint32_t offset, void *out, size_t len)
{
    if (!s_rec_part || s_rec_erasing || offset + len > s_rec_count * MAPPER_RECORD_SIZE) return false;
    return esp_partition_read(s_rec_part, offset, out, len) == ESP_OK;
}
