#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Scan-matching SLAM without wheel odometry (Hector-style).
 * World frame: x = initial forward, y = left, theta CCW (rad), units mm.
 * Occupancy grid in int8 log-odds, cell (gx, gy) at index gy * SIZE + gx,
 * world (0, 0) = cell (ORIGIN, ORIGIN). Pure C: host-testable. */
#define SLAM_SIZE        192
#define SLAM_RES_MM      50
#define SLAM_ORIGIN      (SLAM_SIZE / 2)
#define SLAM_VISIT_CELLS 8                       /* 8 cells = 40 cm */
#define SLAM_VISIT_SIZE  (SLAM_SIZE / SLAM_VISIT_CELLS)
#define SLAM_PATH_MAX    1024
#define SLAM_OCCUPIED    30
#define SLAM_FREE        (-10)
#define SLAM_HEADINGS    19                      /* matches NAV_HEADINGS */
#define SLAM_PLAN_CELLS  4                       /* 4 cells = 20 cm planning grid */
#define SLAM_PLAN_SIZE   (SLAM_SIZE / SLAM_PLAN_CELLS)

typedef struct { float x, y, theta; } slam_pose_t;

/* Exploration target on the planning grid (world mm). */
typedef struct {
    bool valid, frontier;                        /* frontier: next to unknown space */
    float x, y;                                  /* target cell centre */
    float way_x, way_y;                          /* next waypoint on the path (~1 m) */
    uint16_t cell, steps;                        /* planning-grid index, path length */
} slam_target_t;

/* Optional gyro input per scan (MPU6050). dtheta: yaw change (rad, CCW)
   since the previous processed scan; omega: mean yaw rate during this scan. */
typedef struct { bool valid; float dtheta, omega; } slam_imu_t;

typedef struct {
    int8_t grid[SLAM_SIZE * SLAM_SIZE];
    uint8_t visits[SLAM_VISIT_SIZE * SLAM_VISIT_SIZE];
    slam_pose_t pose, integrated_pose;
    float vx, vy, omega;                         /* mm/s, mm/s, rad/s */
    float last_rate;                             /* rad/s used for the previous scan */
    float path_x, path_y;
    uint64_t last_ms;
    uint32_t scans, integrated, rejected, since_integrate;
    float quality;                               /* inlier fraction of the last match */
    /* Heading Kalman filter (state: theta, residual gyro bias in rad/s). */
    float kf_p00, kf_p01, kf_p11, gyro_bias;
    bool kf_ready, imu_used;
    unsigned bad_matches, known_cells;
    bool initialized, lost;
    int16_t path[SLAM_PATH_MAX][2];              /* cm, ring buffer */
    uint16_t path_count, path_head;
} slam_t;

void slam_reset(slam_t *s);
/* raw: LD20 bins (index = raw angle, 0 = no return). period_ms: duration of
   one revolution (for rotation deskew). moving: a motor command is active.
   imu: optional gyro data (NULL or !valid = LiDAR only).
   Returns true if the pose was corrected by a successful match. */
bool slam_update(slam_t *s, const uint16_t raw[360], int front_deg, bool clockwise,
                 float period_ms, uint64_t now_ms, bool moving, const slam_imu_t *imu);
/* 1-sigma heading uncertainty (rad) of the gyro filter, 0 without gyro. */
float slam_heading_sigma(const slam_t *s);
/* Bonus per robot heading (-90..+90 deg, 10 deg, + = right) for directions
   leading into unknown or unvisited space. */
void slam_explore_bonus(const slam_t *s, slam_pose_t pose, int16_t out[SLAM_HEADINGS]);
/* Breadth-first search over free 20 cm cells from the pose: nearest
   reachable frontier, otherwise the least-visited reachable cell. avoid
   (optional, SLAM_PLAN_SIZE^2, non-zero = skip) holds failed targets.
   Not reentrant (static work buffers). */
bool slam_explore_target(const slam_t *s, slam_pose_t pose, const uint8_t *avoid, slam_target_t *out);
/* Shortest path over the 20 cm planning grid from the pose to the goal:
   known free cells first, unknown cells only if needed (out->frontier).
   out->x/y: goal (moved to an open neighbour cell if the goal cell is
   blocked), out->way_x/y: farthest path point in line of sight (<= 1.6 m).
   Not reentrant (shares the static work buffers). */
bool slam_plan_path(const slam_t *s, slam_pose_t pose, float goal_x, float goal_y, slam_target_t *out);
/* Ordered copy of the path (oldest first); returns the number of points. */
uint16_t slam_path_copy(const slam_t *s, int16_t (*out)[2], uint16_t max);

/* 0 unknown, 1 free, 2 probably occupied, 3 occupied */
static inline uint8_t slam_cell_class(int8_t l)
{
    return l >= SLAM_OCCUPIED ? 3 : l >= 12 ? 2 : l <= SLAM_FREE ? 1 : 0;
}
