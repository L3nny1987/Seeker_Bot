#include "mapper.h"
#include "imu.h"
#include "ld20_lidar.h"
#include "seeker_pins.h"
#include "esp_log.h"
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
#define WEAK_BAD_MATCHES   2        /* consecutive rejected matches: weak */

static const char *TAG = "mapper";
static slam_t s_slam;                  /* ~43 KB, written only by the mapper task */
static SemaphoreHandle_t s_lock;
static mapper_info_t s_info;
static volatile bool s_moving, s_reset;
static uint8_t s_avoid[SLAM_PLAN_SIZE * SLAM_PLAN_SIZE];
static slam_target_t s_target;
static uint16_t s_avoided;
static volatile bool s_goal_active;
static volatile float s_goal_x, s_goal_y;

static void publish(uint32_t update_ms)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_info.valid = s_slam.initialized && !s_slam.lost;
    s_info.lost = s_slam.lost;
    /* Young map or failing matches: navigation stands still briefly. */
    s_info.weak = s_slam.initialized && !s_slam.lost &&
                  (s_slam.integrated < WEAK_INTEGRATED || s_slam.bad_matches >= WEAK_BAD_MATCHES);
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
static void update_target(uint64_t now_ms)
{
    static slam_target_t current;
    static uint64_t last_ms, driven_ms;
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
static slam_imu_t imu_input(const ld20_scan_t *scan, float period_ms)
{
    static double last_yaw;
    static uint32_t last_epoch;
    static bool have_last;
    slam_imu_t in = {0};
    double end, start;
    uint32_t epoch_end = 0, epoch_start = 0;
    bool ok = imu_yaw_at(scan->timestamp_us, &end, &epoch_end) &&
              imu_yaw_at(scan->timestamp_us - (int64_t)(period_ms * 1000), &start, &epoch_start) &&
              epoch_end == epoch_start;
    if (ok && have_last && last_epoch == epoch_end) {
        in.valid = true;
        in.dtheta = (float)(end - last_yaw);
        in.omega = (float)((end - start) / (period_ms * 0.001f));
    }
    have_last = ok;
    last_yaw = end;
    last_epoch = epoch_end;
    return in;
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
            update_target((uint64_t)(esp_timer_get_time() / 1000));
            publish(0);
        }
        if (s_reset) {
            /* Readers only see a cleared map; no allocation, no lock needed. */
            slam_reset(&s_slam);
            memset(s_avoid, 0, sizeof(s_avoid));
            s_avoided = 0;
            s_target.valid = false;
            s_reset = false;
            publish(0);
            ESP_LOGI(TAG, "Karte zurueckgesetzt");
        }
        if (ld20_lidar_get_scan(&scan, 0) != ESP_OK || scan.sequence == last_sequence) continue;
        last_sequence = scan.sequence;
        if (scan.valid_sectors < SEEKER_AUTO_MIN_SECTORS) continue;
        float period_ms = scan.speed_rpm ? 60000.0f / scan.speed_rpm : 100.0f;
        slam_imu_t imu = imu_input(&scan, period_ms);
        int64_t start = esp_timer_get_time();
        slam_update(&s_slam, scan.distance_mm, SEEKER_LIDAR_FRONT_DEG, SEEKER_LIDAR_CLOCKWISE,
                    period_ms, (uint64_t)(scan.timestamp_us / 1000), s_moving, &imu);
        if (s_slam.scans % TARGET_EVERY == 0) update_target((uint64_t)(scan.timestamp_us / 1000));
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
