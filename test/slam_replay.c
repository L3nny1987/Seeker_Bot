/* Offline replay of SLAM recordings (tools/slam_record.ps1 download).
 * Build: gcc -std=c11 -O2 -I src test/slam_replay.c src/slam.c -lm -o slam_replay
 * Usage: slam_replay rec.bin [out.csv] [map.pgm]
 * Emulates mapper.c (imu_input) exactly and compares the replayed pose with
 * the on-device pose and the integrated gyro yaw. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "slam.h"

#define RECORD_SIZE 768
#define FRONT_DEG   180                  /* SEEKER_LIDAR_FRONT_DEG */
#define CLOCKWISE   1                    /* SEEKER_LIDAR_CLOCKWISE */
#define DEG         57.29577951308232

typedef struct {
    uint8_t flags;
    uint32_t seq, t_ms, epoch;
    float period, yaw_end, yaw_start, x, y, theta, quality;
    int8_t drive, turn;
    uint16_t valid, dist[360];
} rec_t;

static uint32_t u32(const uint8_t *p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }
static float f32(const uint8_t *p) { uint32_t u = u32(p); float f; memcpy(&f, &u, 4); return f; }

static bool parse(const uint8_t *b, rec_t *r)
{
    if (b[0] != 'S' || b[1] != 'R' || b[2] != 1) return false;
    r->flags = b[3];
    r->seq = u32(b + 4); r->t_ms = u32(b + 8); r->period = f32(b + 12);
    r->yaw_end = f32(b + 16); r->yaw_start = f32(b + 20); r->epoch = u32(b + 24);
    r->drive = (int8_t)b[28]; r->turn = (int8_t)b[29]; r->valid = b[30] | b[31] << 8;
    r->x = f32(b + 32); r->y = f32(b + 36); r->theta = f32(b + 40); r->quality = f32(b + 44);
    for (int i = 0; i < 360; i++) r->dist[i] = b[48 + 2 * i] | b[49 + 2 * i] << 8;
    return true;
}

static float wrap(float a) { return atan2f(sinf(a), cosf(a)); }

static slam_t slam;

int main(int argc, char **argv)
{
    if (argc < 2) { fprintf(stderr, "usage: %s rec.bin [out.csv] [map.pgm]\n", argv[0]); return 2; }
    FILE *in = fopen(argv[1], "rb");
    if (!in) { perror(argv[1]); return 1; }
    FILE *csv = fopen(argc > 2 ? argv[2] : "replay.csv", "w");
    if (!csv) { perror("csv"); return 1; }
    fprintf(csv, "i,t_ms,period,imu,moving,drive,turn,gyro_deg,dev_x,dev_y,dev_deg,dev_q,"
                 "x,y,deg,q,dgyro_deg,step_mm,step_deg,lost,rejected,integrated\n");

    slam_reset(&slam);
    uint8_t buf[RECORD_SIZE];
    rec_t r;
    double last_yaw = 0, gyro = 0;
    uint32_t last_epoch = 0;
    bool have_last = false;
    slam_pose_t prev = slam.pose;
    unsigned n = 0, jumps = 0, rot_jumps = 0, gyro_valid = 0;
    float max_dgyro = 0, sum_dgyro = 0;
    while (fread(buf, 1, RECORD_SIZE, in) == RECORD_SIZE && parse(buf, &r)) {
        bool ok = r.flags & 1, moving = r.flags & 2;
        slam_imu_t imu = {0};
        if (ok && have_last && last_epoch == r.epoch) {
            imu.valid = true;
            imu.dtheta = (float)((double)r.yaw_end - last_yaw);
            imu.omega = (r.yaw_end - r.yaw_start) / (r.period * 0.001f);
            gyro += imu.dtheta;
            gyro_valid++;
        }
        have_last = ok;
        last_yaw = r.yaw_end;
        last_epoch = r.epoch;

        slam_update(&slam, r.dist, FRONT_DEG, CLOCKWISE, r.period, r.t_ms, moving, &imu);
        slam_pose_t p = slam.pose;
        float step = hypotf(p.x - prev.x, p.y - prev.y);
        float dstep = wrap(p.theta - prev.theta);
        float dgyro = wrap(p.theta - (float)gyro);
        if (n > 0 && step > 150) jumps++;
        if (n > 0 && fabsf(dstep) > 0.25f && !imu.valid) rot_jumps++;
        if (n > 0 && imu.valid && fabsf(dstep - imu.dtheta) > 0.15f) rot_jumps++;
        if (fabsf(dgyro) > max_dgyro) max_dgyro = fabsf(dgyro);
        sum_dgyro += fabsf(dgyro);
        fprintf(csv, "%u,%u,%.1f,%d,%d,%d,%d,%.2f,%.0f,%.0f,%.2f,%.2f,%.0f,%.0f,%.2f,%.2f,%.2f,%.0f,%.2f,%d,%u,%u\n",
                n, r.t_ms, r.period, imu.valid, moving, r.drive, r.turn, gyro * DEG,
                r.x, r.y, r.theta * DEG, r.quality, p.x, p.y, p.theta * DEG, slam.quality,
                dgyro * DEG, step, dstep * DEG, slam.lost, slam.rejected, slam.integrated);
        prev = p;
        n++;
    }
    fclose(in);
    fclose(csv);
    printf("scans %u (gyro %u), integrated %u, rejected %u, lost %d\n",
           n, gyro_valid, slam.integrated, slam.rejected, slam.lost);
    printf("jumps >150mm %u, heading steps off gyro %u\n", jumps, rot_jumps);
    printf("heading - gyro: mean %.1f deg, max %.1f deg, final %.1f deg\n",
           n ? sum_dgyro / n * DEG : 0, max_dgyro * DEG, wrap(slam.pose.theta - (float)gyro) * DEG);

    if (argc > 3) {
        FILE *pgm = fopen(argv[3], "wb");
        if (!pgm) { perror(argv[3]); return 1; }
        static uint8_t img[SLAM_SIZE * SLAM_SIZE];
        for (int gy = 0; gy < SLAM_SIZE; gy++)
            for (int gx = 0; gx < SLAM_SIZE; gx++) {
                static const uint8_t shade[4] = {128, 255, 90, 0};
                /* flip y: image row 0 = top = +y */
                img[(SLAM_SIZE - 1 - gy) * SLAM_SIZE + gx] = shade[slam_cell_class(slam.grid[gy * SLAM_SIZE + gx])];
            }
        static int16_t path[SLAM_PATH_MAX][2];
        uint16_t count = slam_path_copy(&slam, path, SLAM_PATH_MAX);
        for (uint16_t i = 0; i < count; i++) {
            int gx = SLAM_ORIGIN + (int)lroundf(path[i][0] * 10.0f / SLAM_RES_MM);
            int gy = SLAM_ORIGIN + (int)lroundf(path[i][1] * 10.0f / SLAM_RES_MM);
            if (gx >= 0 && gy >= 0 && gx < SLAM_SIZE && gy < SLAM_SIZE)
                img[(SLAM_SIZE - 1 - gy) * SLAM_SIZE + gx] = 200;
        }
        fprintf(pgm, "P5\n%d %d\n255\n", SLAM_SIZE, SLAM_SIZE);
        fwrite(img, 1, sizeof(img), pgm);
        fclose(pgm);
    }
    return 0;
}
