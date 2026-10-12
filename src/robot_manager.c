#include "robot_manager.h"
#include "diag.h"
#include "imu.h"
#include "ld20_lidar.h"
#include "track_drive.h"
#include "seeker_pins.h"
#include <string.h>
#include "esp_timer.h"
#include "esp_random.h"
#include "esp_task_wdt.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define SETTINGS_NAMESPACE "seeker"
/* A short LiDAR dropout (e.g. motor inrush on a shared 5 V rail) pauses
   auto; only a persistent loss is a fault. */
#define LIDAR_FAULT_MS 2000
/* Wiggle pulses are short: a full 1 s kick would overshoot them. */
#define TIGHT_KICK_MS 250

static const navigation_body_t BODY = {
    SEEKER_BODY_FRONT_MM, SEEKER_BODY_REAR_MM, SEEKER_BODY_HALF_WIDTH_MM
};

static const char *TAG = "robot_manager";
static SemaphoreHandle_t s_lock;
static robot_status_t s_status = {
    .mode = ROBOT_IDLE, .reason = "boot",
    .settings = {
        .manual_stop_mm = SEEKER_MANUAL_STOP_MM,
        .manual_rotate_mm = SEEKER_MANUAL_ROTATE_MM,
        .auto_stop_mm = SEEKER_STOP_DISTANCE_MM,
        .auto_rotate_mm = SEEKER_ROTATE_CLEARANCE_MM,
        .auto_slow_mm = SEEKER_SLOW_DISTANCE_MM,
        .auto_drive_percent = SEEKER_AUTO_DRIVE_PERCENT,
        .auto_turn_percent = SEEKER_AUTO_TURN_PERCENT,
        .auto_slow_percent = SEEKER_AUTO_SLOW_PERCENT,
        .ramp_ms = SEEKER_RAMP_MS,
        .reverse_ms = SEEKER_REVERSE_MS,
        .turn_ms = SEEKER_TURN_MS,
        .resume_margin_mm = SEEKER_RESUME_MARGIN_MM,
        .robot_width_mm = SEEKER_ROBOT_WIDTH_MM,
        .auto_limit_s = SEEKER_AUTO_LIMIT_S,
        .turn_ramp_ms = SEEKER_TURN_RAMP_MS,
        .steer_min_percent = SEEKER_STEER_MIN_PERCENT,
        .wall_distance_mm = SEEKER_WALL_DISTANCE_MM,
        .drive_min_percent = SEEKER_DRIVE_MIN_PERCENT,
        .turn_min_percent = SEEKER_TURN_MIN_PERCENT,
        .drive_kick_ms = SEEKER_DRIVE_KICK_MS,
        .turn_kick_ms = SEEKER_TURN_KICK_MS
    }
};
static uint32_t s_token, s_sequence;
static int64_t s_heartbeat, s_manual_time, s_auto_started, s_lidar_bad_since;
static int s_requested_drive, s_requested_turn;
static navigation_auto_t s_navigation;
static motion_axis_t s_drive_axis, s_turn_axis;

const char *robot_mode_name(robot_mode_t mode)
{
    static const char *names[] = { "idle", "manual", "auto", "estop", "fault" };
    return mode <= ROBOT_FAULT ? names[mode] : "fault";
}

static void stop_locked(robot_mode_t mode, const char *reason)
{
    if (s_status.mode == ROBOT_AUTO || mode == ROBOT_FAULT)
        ESP_LOGW(TAG, "Stop mode=%s reason=%s", robot_mode_name(mode), reason);
    s_requested_drive = s_requested_turn = 0;
    s_status.drive = s_status.turn = 0;
    s_drive_axis = s_turn_axis = (motion_axis_t){0};
    s_status.mode = mode;
    s_status.reason = reason;
    navigation_auto_reset(&s_navigation);
    if (track_drive_coast() != ESP_OK) {
        if (track_drive_emergency_stop() != ESP_OK) esp_restart();
        s_status.mode = ROBOT_FAULT;
        s_status.reason = "motor_fault";
    }
}

static bool lease_valid(int64_t now)
{
    return s_token && now - s_heartbeat < SEEKER_CONTROL_TIMEOUT_MS * 1000LL;
}

static int axis(motion_axis_t *a, int target, int min_percent, int kick_ms, int ramp_ms)
{
    return motion_axis_step(a, target, min_percent, kick_ms, ramp_ms, SEEKER_MANAGER_PERIOD_MS);
}

static void settings_load(void)
{
    nvs_handle_t handle;
    if (nvs_open(SETTINGS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) return;
    uint32_t version = 0;
    motion_settings_t stored;
    size_t size = sizeof(stored);
    if (nvs_get_u32(handle, "motion_ver", &version) == ESP_OK && version == MOTION_SETTINGS_VERSION &&
        nvs_get_blob(handle, "motion", &stored, &size) == ESP_OK && size == sizeof(stored) &&
        motion_settings_valid(stored)) {
        s_status.settings = stored;
        ESP_LOGI(TAG, "Gespeicherte Bewegungsparameter geladen");
    }
    nvs_close(handle);
}

static esp_err_t settings_save(motion_settings_t settings)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(SETTINGS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    err = nvs_set_blob(handle, "motion", &settings, sizeof(settings));
    if (err == ESP_OK) err = nvs_set_u32(handle, "motion_ver", MOTION_SETTINGS_VERSION);
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

static void manager_task(void *arg)
{
    (void)arg;
    /* A stalled manager must reboot rather than leave a persistent PWM duty.
       External EN pulldowns + a physical power cutoff remain necessary. */
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    TickType_t wake = xTaskGetTickCount();
    static ld20_scan_t scan;
    static uint16_t previous[360], fused[360];
    uint32_t previous_sequence = 0, view_sequence = 0;
    bool previous_stationary = false;
    int view_width = 0;
    navigation_view_t view = {0};
    while (true) {
        bool fresh = ld20_lidar_get_scan(&scan, 0) == ESP_OK;
        xSemaphoreTake(s_lock, portMAX_DELAY);
        int width = s_status.settings.robot_width_mm;
        bool stationary = !s_status.drive && !s_status.turn;
        xSemaphoreGive(s_lock);
        if (scan.sequence != view_sequence || width != view_width) {
            /* Analyse each revolution once, outside the lock. Single
               dropouts are filled from the directly preceding revolution. */
            bool consecutive = stationary && previous_stationary && previous_sequence &&
                               scan.sequence == previous_sequence + 1;
            navigation_fuse(scan.distance_mm, consecutive ? previous : NULL, fused);
            if (scan.sequence != previous_sequence) {
                memcpy(previous, scan.distance_mm, sizeof(previous));
                previous_sequence = scan.sequence;
                previous_stationary = stationary;
            }
            view = navigation_analyze(fused, SEEKER_LIDAR_FRONT_DEG, SEEKER_LIDAR_CLOCKWISE, width, BODY);
            mapper_explore_bonus(view.explore_bonus);
            view_sequence = scan.sequence;
            view_width = width;
        }
        /* SLAM pose of the latest matched scan (mapper task, own lock). */
        mapper_info_t map = mapper_info();
        int64_t now = esp_timer_get_time();
        bool pose_fresh = pose_feedback_fresh(map.feedback, now);
        view.pose_valid = map.valid && pose_fresh;
        view.pose_x = map.x;
        view.pose_y = map.y;
        view.pose_theta = map.theta;
        view.pose_sequence = map.scan_sequence;
        view.heading_live = false;
        view.yaw_rate = 0;
        double yaw, prior_yaw;
        uint32_t epoch, prior_epoch;
        if (imu_yaw_at(now, &yaw, &epoch) &&
            pose_feedback_heading(map.feedback, now, yaw, epoch, map.theta, map.gyro_bias, &view.pose_theta) &&
            imu_yaw_at(now - 20000, &prior_yaw, &prior_epoch) && prior_epoch == epoch) {
            view.heading_live = true;
            view.yaw_rate = (float)((yaw - prior_yaw) / 0.02) - map.gyro_bias;
        }
        view.route_valid = view.pose_valid && map.target.valid;
        view.route_way_x = map.target.way_x;
        view.route_way_y = map.target.way_y;
        view.route_goal_x = map.target.x;
        view.route_goal_y = map.target.y;
        view.map_lost = map.lost;
        view.map_weak = map.weak || !pose_fresh;
        xSemaphoreTake(s_lock, portMAX_DELAY);
        s_status.view = view;
        s_status.coverage = scan.valid_sectors;
        s_status.scan_sequence = scan.sequence;
        s_status.scan_age_ms = scan.timestamp_us ? (now - scan.timestamp_us) / 1000 : UINT32_MAX;
        s_status.lidar_healthy = fresh && scan.valid_sectors >= SEEKER_AUTO_MIN_SECTORS;
        if (s_status.lidar_healthy) s_lidar_bad_since = 0;
        else if (!s_lidar_bad_since) s_lidar_bad_since = now;
        if (s_token && !lease_valid(now)) {
            s_token = 0;
            if (s_status.mode == ROBOT_MANUAL || s_status.mode == ROBOT_AUTO)
                stop_locked(ROBOT_IDLE, "controller_timeout");
        }
        s_status.controller = s_token != 0;
        /* Short Wi-Fi drops pause motion; only the lease timeout ends it. */
        bool link_ok = s_token && now - s_heartbeat < SEEKER_LINK_PAUSE_MS * 1000LL;
        s_status.link_ok = link_ok;
        int drive = 0, turn = 0;
        bool tight = false;
        bool precision = false;
        if (s_status.mode == ROBOT_MANUAL) {
            if (now - s_manual_time > SEEKER_MANUAL_TIMEOUT_MS * 1000LL) {
                s_requested_drive = s_requested_turn = 0;
                s_status.reason = "deadman_timeout";
            }
            drive = s_requested_drive;
            turn = s_requested_turn;
            /* Manual commissioning is possible without LiDAR. When valid,
               measured close obstacles block travel toward them. Clearances
               are measured from the body, not from the LiDAR. */
            if (fresh && drive > 0 && view.front_mm &&
                view.free_mm[NAV_CENTER] <= s_status.settings.manual_stop_mm) {
                drive = 0; s_status.reason = "front_obstacle";
            }
            if (fresh && drive < 0 && view.rear_mm &&
                view.rear_free_mm <= s_status.settings.manual_stop_mm) {
                drive = 0; s_status.reason = "rear_obstacle";
            }
            if (fresh && turn && view.nearest_mm && view.swing_mm <= s_status.settings.manual_rotate_mm) {
                turn = 0; s_status.reason = "rotation_obstacle";
            }
        } else if (s_status.mode == ROBOT_AUTO) {
            if (!s_status.lidar_healthy && now - s_lidar_bad_since >= LIDAR_FAULT_MS * 1000LL)
                stop_locked(ROBOT_FAULT, "lidar_stale_or_incomplete");
            else if (now - s_auto_started >= s_status.settings.auto_limit_s * 1000000LL)
                stop_locked(ROBOT_IDLE, "auto_time_limit");
            else if (!s_status.lidar_healthy || !link_ok) {
                /* Hold still and re-plan from scratch when data returns. */
                navigation_auto_reset(&s_navigation);
                s_status.reason = !link_ok ? "link_wait" : "lidar_wait";
            } else {
                navigation_plan_t p = navigation_auto_step(&s_navigation, &view,
                    s_status.settings, now / 1000, scan.sequence);
                if (p.done) stop_locked(ROBOT_IDLE, p.reason);
                else {
                    drive = p.drive; turn = p.turn; tight = p.tight; precision = p.precision;
                    s_status.reason = p.reason;
                }
            }
        }
        /* Output stage: minimum PWM, start kick, ramps (steering faster). */
        const motion_settings_t *cfg = &s_status.settings;
        int drive_kick = cfg->drive_kick_ms, turn_kick = cfg->turn_kick_ms;
        if (tight && drive_kick > TIGHT_KICK_MS) drive_kick = TIGHT_KICK_MS;
        if (tight && turn_kick > TIGHT_KICK_MS) turn_kick = TIGHT_KICK_MS;
        /* A course pulse is at most 80 ms. A 300 ms start kick and a slow
           down-ramp would turn a small correction into a large oscillation. */
        if (precision && turn_kick > SEEKER_MANAGER_PERIOD_MS) turn_kick = SEEKER_MANAGER_PERIOD_MS;
        int actual_drive = axis(&s_drive_axis, drive, cfg->drive_min_percent, drive_kick, cfg->ramp_ms);
        int actual_turn = axis(&s_turn_axis, turn, cfg->turn_min_percent, turn_kick,
                              precision ? 0 : cfg->turn_ramp_ms);
        if (s_status.mode != ROBOT_ESTOP && s_status.mode != ROBOT_FAULT) {
            if (track_drive_set(actual_drive, actual_turn) != ESP_OK) {
                track_drive_emergency_stop();
                stop_locked(ROBOT_FAULT, "motor_fault");
            } else { s_status.drive = actual_drive; s_status.turn = actual_turn; }
        }
        mapper_set_motion(s_status.drive, s_status.turn);
        diag_snapshot(s_status.mode, s_status.drive, s_status.turn, s_status.network);
        mapper_set_goal(s_status.mode == ROBOT_AUTO && s_navigation.strategy == NAV_STRATEGY_PATH,
                        s_navigation.goal_x, s_navigation.goal_y);
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
    navigation_auto_reset(&s_navigation);
    settings_load();   /* NVS must be initialised before this call. */
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
                                int drive, int turn, const char *mode,
                                const robot_auto_request_t *auto_request)
{
    if (!cmd || !s_lock) return ESP_ERR_INVALID_ARG;
    /* Read the SLAM state before taking the manager lock (no nesting). */
    mapper_info_t map = mapper_info();
    xSemaphoreTake(s_lock, portMAX_DELAY);
    int64_t now = esp_timer_get_time();
    esp_err_t err = ESP_OK;
    /* Emergency shutdown does not require ownership; restart always does. */
    if (!strcmp(cmd, "estop")) {
        if (track_drive_emergency_stop() != ESP_OK) esp_restart();
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
                navigation_strategy_t strategy = auto_request ? auto_request->strategy : NAV_STRATEGY_EXPLORE;
                /* Goal navigation needs a localized robot. */
                if (!s_status.commissioned || !s_status.lidar_healthy ||
                    ((strategy == NAV_STRATEGY_GOAL || strategy == NAV_STRATEGY_PATH) && !map.valid))
                    err = ESP_ERR_INVALID_STATE;
                else {
                    navigation_auto_configure(&s_navigation, strategy,
                        auto_request ? auto_request->goal_x : 0, auto_request ? auto_request->goal_y : 0);
                    ESP_LOGI(TAG, "AUTO start: strategy=%s goal=%.0f,%.0f drive=%d slow=%d turn=%d ramp=%d/%dms stop=%d rotate=%d slowZone=%dmm width=%dmm wall=%dmm limit=%ds heap=%lu",
                        navigation_strategy_name(strategy), s_navigation.goal_x, s_navigation.goal_y,
                        s_status.settings.auto_drive_percent, s_status.settings.auto_slow_percent,
                        s_status.settings.auto_turn_percent, s_status.settings.ramp_ms,
                        s_status.settings.turn_ramp_ms,
                        s_status.settings.auto_stop_mm, s_status.settings.auto_rotate_mm,
                        s_status.settings.auto_slow_mm, s_status.settings.robot_width_mm,
                        s_status.settings.wall_distance_mm,
                        s_status.settings.auto_limit_s, (unsigned long)esp_get_free_heap_size());
                    stop_locked(ROBOT_AUTO, "auto_ready");
                    s_auto_started = now;
                }
            } else if (!strcmp(mode, "idle")) stop_locked(ROBOT_IDLE, "operator_stop");
            else err = ESP_ERR_INVALID_ARG;
        } else if (!strcmp(cmd, "map_reset")) {
            if (s_status.mode == ROBOT_AUTO) err = ESP_ERR_INVALID_STATE;
            else { mapper_request_reset(); s_status.reason = "map_reset"; }
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
    mapper_info_t map = mapper_info();
    xSemaphoreTake(s_lock, portMAX_DELAY);
    robot_status_t status = s_status;
    int64_t now = esp_timer_get_time();
    status.map = map;
    if (!pose_feedback_fresh(map.feedback, now)) {
        status.map.valid = false;
        status.map.weak = true;
    }
    status.strategy = navigation_strategy_name(s_navigation.strategy);
    status.nav_detail = s_navigation.detail ? s_navigation.detail : "idle";
    status.goal_set = s_navigation.strategy == NAV_STRATEGY_GOAL ||
                      s_navigation.strategy == NAV_STRATEGY_PATH;
    status.goal_x = s_navigation.goal_x;
    status.goal_y = s_navigation.goal_y;
    status.auto_phase = navigation_phase_name(s_navigation.phase);
    status.auto_trigger = s_navigation.trigger;
    status.nav_heading = s_navigation.heading_deg;
    status.escape_attempts = s_navigation.attempts;
    status.heartbeat_age_ms = s_token ? (uint32_t)((now - s_heartbeat) / 1000) : UINT32_MAX;
    int64_t remaining = s_status.settings.auto_limit_s - (now - s_auto_started) / 1000000;
    status.auto_remaining_s = s_status.mode == ROBOT_AUTO && remaining > 0 ? (uint32_t)remaining : 0;
    xSemaphoreGive(s_lock);
    return status;
}

esp_err_t robot_manager_configure(uint32_t token, uint32_t sequence, motion_settings_t settings)
{
    if (!s_lock || !motion_settings_valid(settings)) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t err = ESP_ERR_INVALID_STATE;
    int64_t now = esp_timer_get_time();
    /* Only the controller may configure, and only while explicitly stopped.
       Changing margins must never silently resume an autonomous movement. */
    if (token && token == s_token && lease_valid(now) && s_status.network &&
        sequence > s_sequence && s_status.mode == ROBOT_IDLE &&
        !s_status.drive && !s_status.turn) {
        s_sequence = sequence;
        s_heartbeat = now;
        s_status.settings = settings;
        s_status.reason = "settings_updated";
        err = ESP_OK;
    }
    xSemaphoreGive(s_lock);
    /* Persist outside the lock: flash writes must not delay the manager. */
    if (err == ESP_OK) {
        esp_err_t saved = settings_save(settings);
        if (saved != ESP_OK) ESP_LOGW(TAG, "Parameter nicht gespeichert: %s", esp_err_to_name(saved));
    }
    return err;
}
