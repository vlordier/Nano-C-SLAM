/* Replay captured Gazebo poses and four-ray ranges through the owned SLAM core.
 */
#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "point_cloud.h"

#define MAX_REPLAY_POSES 170
#define MAX_LIDAR_RANGE_MM 3490
#define MAPPING_RANGE_LIMIT_MM 1400

uint8_t verbose = 0;
static slam_job_t last_slam_job;
static icp_points_t previous_scan;
static icp_points_t current_scan;
static int previous_scan_start = -1;
static int icp_constraint_attempt_count;

static int fail(const char *message) {
    fprintf(stderr, "mapping replay: %s\n", message);
    return EXIT_FAILURE;
}

static void process_scan_window(int start_id) {
    pcl_extract_scan((int16_t)start_id, (int16_t)(start_id + 20), &current_scan,
                     MAPPING_RANGE_LIMIT_MM);
    if (current_scan.num == 0) return;

    if (previous_scan_start >= 0 && previous_scan.num > 0) {
        float correction[3] = {0};
        icp_job_t icp_job = {0};
        icp(&previous_scan, &current_scan, 25, 0.4f, correction, &icp_job);
        if (icp_job.has_nan == 0 && icp_job.error < 0.01f) {
            pcl_add_constraint((int16_t)start_id, (int16_t)previous_scan_start,
                               correction);
            icp_constraint_attempt_count++;
        }
    }

    add_scan_pose_id((uint16_t)start_id);
    previous_scan = current_scan;
    previous_scan_start = start_id;
}

static int emit_map(void) {
    const int16_t pose_count = pcl_get_pose_count();
    if (pose_count <= 0) return fail("no poses were accepted");

    icp_points_t map = {0};
    pcl_extract_scan(0, pose_count, &map, MAPPING_RANGE_LIMIT_MM);
    if (map.num == 0) return fail("map contains no projected lidar points");

    float *poses = calloc((size_t)pose_count * 3, sizeof(*poses));
    if (poses == NULL) return fail("could not allocate pose output");
    if (!pcl_to_poses(poses, 0, pose_count)) {
        free(poses);
        return fail("could not read optimized poses");
    }
    for (int16_t i = 0; i < pose_count * 3; i++) {
        if (!isfinite(poses[i])) {
            free(poses);
            return fail("optimized pose contains a non-finite value");
        }
    }

    const int32_t stored_constraint_count = pcl_get_constraint_count();
    printf("{\"pose_count\":%d,\"scan_count\":%u,\"map_point_count\":%zu,"
           "\"icp_constraint_attempt_count\":%d,"
           "\"stored_constraint_count\":%d,"
           "\"optimized_constraint_count\":%d,"
           "\"icp_constraint_count\":%d,\"optimized_pose_count\":%d,"
           "\"main_graph_size\":%d,"
           "\"has_nan\":%d,\"poses\":[",
           pose_count, pcl_get_scan_pose_count(), map.num,
           icp_constraint_attempt_count, (int)stored_constraint_count,
           (int)stored_constraint_count, (int)stored_constraint_count,
           last_slam_job.number_of_poses, last_slam_job.main_graph_size,
           last_slam_job.has_nan);
    for (int16_t i = 0; i < pose_count; i++) {
        printf("%s[%.7g,%.7g,%.7g]", i == 0 ? "" : ",", poses[3 * i],
               poses[3 * i + 1], poses[3 * i + 2]);
    }
    printf("],\"points\":[");
    for (size_t i = 0; i < map.num; i++) {
        printf("%s[%.7g,%.7g]", i == 0 ? "" : ",", map.items[i].x,
               map.items[i].y);
    }
    puts("]}");
    free(poses);
    return EXIT_SUCCESS;
}

int main(void) {
    int id;
    int timestamp_ms;
    float x, y, yaw;
    int ranges[4];
    int sample_count = 0;
    int last_scan_start = -1;
    int last_timestamp_ms = -1;

    verbose = 0;
    while (1) {
        int parsed =
            scanf("%d %d %f %f %f %d %d %d %d", &id, &timestamp_ms, &x, &y,
                  &yaw, &ranges[0], &ranges[1], &ranges[2], &ranges[3]);
        if (parsed == EOF) break;
        if (parsed != 9) return fail("malformed replay sample");
        if (sample_count >= MAX_REPLAY_POSES)
            return fail(
                "replay exceeds the supported pose/map buffer capacity");
        if (id != sample_count || id > INT16_MAX || timestamp_ms < 0 ||
            timestamp_ms <= last_timestamp_ms || !isfinite(x) || !isfinite(y) ||
            !isfinite(yaw))
            return fail("sample IDs, timestamp, or pose are invalid");

        pcl_pose_t pose = {0};
        pose.id = (int16_t)id;
        pose.timestamp = timestamp_ms;
        pose.pos[0] = x;
        pose.pos[1] = y;
        pose.pos[2] = yaw;
        for (int direction = 0; direction < 4; direction++) {
            if (ranges[direction] < -1 ||
                ranges[direction] > MAX_LIDAR_RANGE_MM)
                return fail("range is outside the simulated lidar bounds");
            for (int column = 0; column < 8; column++)
                pose.tof[direction][column] = -1;
            /* The four-ray Gazebo sensor has no angular bins; place each ray
             * in the closest-to-forward column consumed by the real mapper. */
            pose.tof[direction][3] = (int16_t)ranges[direction];
        }
        if (!pcl_add_augmented_pose(pose))
            return fail("Nano-C-SLAM rejected an augmented pose");
        last_timestamp_ms = timestamp_ms;
        sample_count++;

        if (sample_count % 20 == 0) {
            last_scan_start = sample_count - 20;
            process_scan_window(last_scan_start);
            pcl_optimize(&last_slam_job);
        }
    }
    if (ferror(stdin)) {
        perror("mapping replay: stdin");
        return EXIT_FAILURE;
    }
    if (sample_count >= 20 && last_scan_start != sample_count - 20) {
        last_scan_start = sample_count - 20;
        process_scan_window(last_scan_start);
        pcl_optimize(&last_slam_job);
    }
    return emit_map();
}
