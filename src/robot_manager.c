#include "robot_manager.h"
#include "ld20_lidar.h"
#include "track_drive.h"
#include "seeker_pins.h"
#include <string.h>
#include "esp_timer.h"
#include "esp_random.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static SemaphoreHandle_t s_lock;
static robot_status_t s_status = { .mode = ROBOT_IDLE, .reason = "boot" };
static uint32_t s_token, s_sequence;
static int64_t s_heartbeat, s_manual_time, s_auto_started, s_avoid_started;
static int s_requested_drive, s_requested_turn, s_preferred_turn;

const char *robot_mode_name(robot_mode_t mode)
{
    static const char *names[] = { "idle", "manual", "auto", "estop", "fault" };
    return mode <= ROBOT_FAULT ? names[mode] : "fault";
}

static void stop_locked(robot_mode_t mode, const char *reason)
{
    s_requested_drive = s_requested_turn = 0;
    s_status.drive = s_status.turn = 0;
    s_status.mode = mode;
    s_status.reason = reason;
    if (track_drive_coast() != ESP_OK) {
        track_drive_emergency_stop();
        s_status.mode = ROBOT_FAULT;
        s_status.reason = "motor_fault";
    }
}

static bool lease_valid(int64_t now)
{
    return s_token && now - s_heartbeat < SEEKER_CONTROL_TIMEOUT_MS * 1000LL;
}

static int ramp(int current, int target)
{
    if (!target || (current > 0 && target < 0) || (current < 0 && target > 0)) return 0;
    if (current < target) return current + 2 > target ? target : current + 2;
    if (current > target) return current - 2 < target ? target : current - 2;
    return current;
}

static void manager_task(void *arg)
{
    (void)arg;
    /* A stalled manager must reboot rather than leave a persistent PWM duty.
       External EN pulldowns + a physical power cutoff remain necessary. */
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    TickType_t wake = xTaskGetTickCount();
    ld20_scan_t scan = {0};
    while (true) {
        bool fresh = ld20_lidar_get_scan(&scan, 0) == ESP_OK;
        navigation_view_t view = navigation_analyze(scan.distance_mm,
                                   SEEKER_LIDAR_FRONT_DEG, SEEKER_LIDAR_CLOCKWISE);
        int64_t now = esp_timer_get_time();
        xSemaphoreTake(s_lock, portMAX_DELAY);
        s_status.view = view;
        s_status.coverage = scan.valid_sectors;
        s_status.scan_sequence = scan.sequence;
        s_status.scan_age_ms = scan.timestamp_us ? (now - scan.timestamp_us) / 1000 : UINT32_MAX;
        s_status.lidar_healthy = fresh && scan.valid_sectors >= SEEKER_AUTO_MIN_SECTORS;
        if (!lease_valid(now)) {
            s_token = 0;
            if (s_status.mode == ROBOT_MANUAL || s_status.mode == ROBOT_AUTO)
                stop_locked(ROBOT_IDLE, "controller_timeout");
        }
        s_status.controller = s_token != 0;
        int drive = 0, turn = 0;
        if (s_status.mode == ROBOT_MANUAL) {
            if (now - s_manual_time > SEEKER_MANUAL_TIMEOUT_MS * 1000LL) {
                s_requested_drive = s_requested_turn = 0;
                s_status.reason = "deadman_timeout";
            }
            drive = s_requested_drive;
            turn = s_requested_turn;
            /* Manual commissioning is possible without LiDAR. When valid,
               measured close obstacles block travel toward them. */
            if (fresh && drive > 0 && view.front_mm && view.front_mm <= SEEKER_STOP_DISTANCE_MM) {
                drive = 0; s_status.reason = "front_obstacle";
            }
            if (fresh && drive < 0 && view.rear_mm && view.rear_mm <= SEEKER_STOP_DISTANCE_MM) {
                drive = 0; s_status.reason = "rear_obstacle";
            }
            if (fresh && turn && view.nearest_mm && view.nearest_mm <= SEEKER_ROTATE_CLEARANCE_MM) {
                turn = 0; s_status.reason = "rotation_obstacle";
            }
        } else if (s_status.mode == ROBOT_AUTO) {
            if (!s_status.lidar_healthy) stop_locked(ROBOT_FAULT, "lidar_stale_or_incomplete");
            else if (now - s_auto_started > 60000000LL) stop_locked(ROBOT_IDLE, "auto_time_limit");
            else {
                navigation_plan_t p = navigation_plan(view, s_preferred_turn,
                    SEEKER_STOP_DISTANCE_MM, SEEKER_ROTATE_CLEARANCE_MM, SEEKER_SLOW_DISTANCE_MM,
                    SEEKER_AUTO_DRIVE_PERCENT, SEEKER_AUTO_TURN_PERCENT);
                drive = p.drive; turn = p.turn;
                s_status.reason = p.reason;
                if (turn) {
                    if (!s_avoid_started) s_avoid_started = now;
                    s_preferred_turn = turn < 0 ? -1 : 1;
                    if (now - s_avoid_started > 3000000LL) {
                        drive = turn = 0;
                        stop_locked(ROBOT_IDLE, "avoidance_timeout");
                    }
                } else { s_avoid_started = 0; s_preferred_turn = 0; }
            }
        }
        int actual_drive = ramp(s_status.drive, drive);
        int actual_turn = ramp(s_status.turn, turn);
        if (s_status.mode != ROBOT_ESTOP && s_status.mode != ROBOT_FAULT) {
            if (track_drive_set(actual_drive, actual_turn) != ESP_OK) {
                track_drive_emergency_stop();
                stop_locked(ROBOT_FAULT, "motor_fault");
            } else { s_status.drive = actual_drive; s_status.turn = actual_turn; }
        }
        xSemaphoreGive(s_lock);
        ESP_ERROR_CHECK(esp_task_wdt_reset());
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(SEEKER_MANAGER_PERIOD_MS));
    }
}

esp_err_t robot_manager_init(void)
{
    if (s_lock) return ESP_ERR_INVALID_STATE;
    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) return ESP_ERR_NO_MEM;
    return xTaskCreate(manager_task, "robot_manager", 6144, NULL, 8, NULL) == pdPASS
           ? ESP_OK : ESP_ERR_NO_MEM;
}

void robot_manager_network(bool connected)
{
    if (!s_lock) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_status.network = connected;
    if (!connected) {
        s_token = 0;
        s_status.controller = false;
        if (s_status.mode != ROBOT_ESTOP && s_status.mode != ROBOT_FAULT)
            stop_locked(ROBOT_IDLE, "wifi_disconnected");
    }
    xSemaphoreGive(s_lock);
}

esp_err_t robot_manager_claim(uint32_t *token)
{
    if (!token || !s_lock) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t err = ESP_ERR_INVALID_STATE;
    int64_t now = esp_timer_get_time();
    if (s_status.network && !lease_valid(now)) {
        if (s_status.mode == ROBOT_MANUAL || s_status.mode == ROBOT_AUTO)
            stop_locked(ROBOT_IDLE, "new_controller");
        do { s_token = esp_random(); } while (!s_token);
        s_heartbeat = now;
        s_sequence = 0;
        s_status.controller = true;
        *token = s_token;
        err = ESP_OK;
    }
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t robot_manager_command(uint32_t token, uint32_t sequence, const char *cmd,
                                int drive, int turn, const char *mode)
{
    if (!cmd || !s_lock) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    int64_t now = esp_timer_get_time();
    esp_err_t err = ESP_OK;
    /* Emergency shutdown does not require ownership; restart always does. */
    if (!strcmp(cmd, "estop")) {
        track_drive_emergency_stop();
        stop_locked(ROBOT_ESTOP, "operator_estop");
    } else if (!token || token != s_token || !lease_valid(now) || !s_status.network ||
               sequence <= s_sequence) err = ESP_ERR_INVALID_STATE;
    else {
        s_sequence = sequence;
        s_heartbeat = now;
        if (!strcmp(cmd, "heartbeat")) { /* Lease only: never extends movement. */ }
        else if (!strcmp(cmd, "release")) {
            if (s_status.mode != ROBOT_ESTOP && s_status.mode != ROBOT_FAULT)
                stop_locked(ROBOT_IDLE, "released");
            s_token = 0;
            s_status.controller = false;
        } else if (!strcmp(cmd, "stop")) {
            if (s_status.mode != ROBOT_ESTOP && s_status.mode != ROBOT_FAULT)
                stop_locked(ROBOT_IDLE, "operator_stop");
        } else if (!strcmp(cmd, "reset")) {
            err = track_drive_clear_stop();
            if (err == ESP_OK) stop_locked(ROBOT_IDLE, "reset");
        } else if (!strcmp(cmd, "commission")) {
            if (s_status.mode != ROBOT_IDLE) err = ESP_ERR_INVALID_STATE;
            else s_status.commissioned = true;
        } else if (!strcmp(cmd, "mode") && mode) {
            if (s_status.mode == ROBOT_ESTOP || s_status.mode == ROBOT_FAULT) err = ESP_ERR_INVALID_STATE;
            else if (!strcmp(mode, "manual")) stop_locked(ROBOT_MANUAL, "manual_ready");
            else if (!strcmp(mode, "auto")) {
                if (!s_status.commissioned || !s_status.lidar_healthy || !s_status.view.front_known)
                    err = ESP_ERR_INVALID_STATE;
                else {
                    stop_locked(ROBOT_AUTO, "auto_ready");
                    s_auto_started = now;
                    s_preferred_turn = 0; s_avoid_started = 0;
                }
            } else if (!strcmp(mode, "idle")) stop_locked(ROBOT_IDLE, "operator_stop");
            else err = ESP_ERR_INVALID_ARG;
        } else if (!strcmp(cmd, "move")) {
            if (s_status.mode != ROBOT_MANUAL) err = ESP_ERR_INVALID_STATE;
            else if (drive < -SEEKER_MANUAL_MAX_PERCENT || drive > SEEKER_MANUAL_MAX_PERCENT ||
                     turn < -SEEKER_MANUAL_MAX_PERCENT || turn > SEEKER_MANUAL_MAX_PERCENT)
                err = ESP_ERR_INVALID_ARG;
            else {
                s_requested_drive = drive; s_requested_turn = turn;
                s_manual_time = now;
                s_status.reason = "manual";
            }
        } else err = ESP_ERR_INVALID_ARG;
    }
    xSemaphoreGive(s_lock);
    return err;
}

robot_status_t robot_manager_status(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    robot_status_t status = s_status;
    xSemaphoreGive(s_lock);
    return status;
}