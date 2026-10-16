#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(condition) \
    do { \
        if (!(condition)) { \
            fprintf(stderr, "check failed at line %d: %s\n", __LINE__, #condition); \
            abort(); \
        } \
    } while (0)

#include "../cslam-gap9-app/point-cloud/external_point_cloud.h"
#include "../cslam-gap9-app/point-cloud/point_cloud.h"
#include "../cslam-gap9-app/slam/graph-based-slam.h"

extern constraint constraints[];

uint8_t verbose;
static int main_graph_calls;
static int fail_slam;

uint32_t pi_time_get_us(void) {
    return 1000;
}
uint16_t external_constraints_count(void) {
    return 0;
}
constraint get_ext_constraint(int16_t idx) {
    (void)idx;
    constraint c = {0};
    return c;
}

static int compare_pose_ids(const void *a, const void *b) {
    int16_t left = *(const int16_t *)a;
    int16_t right = *(const int16_t *)b;
    return (left > right) - (left < right);
}

void delete_duplicates_and_sort(int16_t *ids, int16_t count, int16_t *unique) {
    qsort(ids, count, sizeof(*ids), compare_pose_ids);
    *unique = 0;
    for (int16_t i = 0; i < count; i++) {
        if (*unique == 0 || ids[i] != ids[*unique - 1])
            ids[(*unique)++] = ids[i];
    }
}

void measurement_from_poses(float *from, float *to, float *z) {
    for (int i = 0; i < 3; i++)
        z[i] = to[i] - from[i];
}

bool ls_slam(float *poses, constraint *edges, int16_t pose_count, int16_t edge_count) {
    (void)poses;
    CHECK(edge_count == 1);
    if (!fail_slam && pose_count == 3 && edges[0].fromIdx == 1 && edges[0].toIdx == 2)
        main_graph_calls++;
    return !fail_slam;
}

int main(void) {
    const float positions[] = {0, 0.1f, 1, 1.1f, 2};
    for (int i = 0; i < 5; i++) {
        pcl_pose_t pose = {.id = i, .pos = {positions[i], 0, 0}};
        for (int dir = 0; dir < 4; dir++)
            for (int col = 0; col < 8; col++)
                pose.tof[dir][col] = 100;
        if (i == 0)
            pose.tof[DIR_FRONT][0] = -1;
        CHECK(pcl_add_augmented_pose(pose));
    }

    pcl_pose_t invalid_pose = {0};
    invalid_pose.pos[1] = NAN;
    CHECK(!pcl_add_augmented_pose(invalid_pose));
    CHECK(pcl_get_pose_count() == 5);
    invalid_pose.pos[1] = 0;
    invalid_pose.tof[DIR_LEFT][3] = -2;
    CHECK(!pcl_add_augmented_pose(invalid_pose));
    CHECK(pcl_get_pose_count() == 5);

    float requested_poses[6] = {-1, -1, -1, -1, -1, -1};
    CHECK(pcl_to_poses(requested_poses, 1, 2));
    CHECK(requested_poses[0] == positions[1]);
    CHECK(requested_poses[3] == positions[2]);

    float rejected_poses[3] = {-1, -1, -1};
    CHECK(!pcl_to_poses(rejected_poses, -1, 1));
    CHECK(rejected_poses[0] == -1);
    CHECK(!pcl_to_poses(rejected_poses, 4, 2));
    CHECK(rejected_poses[0] == -1);
    CHECK(!pcl_to_poses(rejected_poses, 0, -1));
    CHECK(rejected_poses[0] == -1);
    CHECK(!pcl_to_poses(NULL, 0, 1));

    float correction[3] = {0};
    pcl_add_constraint(2, 4, correction);
    CHECK(constraints[0].fromIdx == 2 && constraints[0].toIdx == 4);
    slam_job_t job;
    pcl_optimize(&job);
    pcl_optimize(&job);
    CHECK(main_graph_calls == 2);
    CHECK(constraints[0].fromIdx == 2 && constraints[0].toIdx == 4);

    fail_slam = 1;
    pcl_optimize(&job);
    CHECK(job.has_nan);
    CHECK(main_graph_calls == 2);
    float after_failed_optimization[3];
    CHECK(pcl_to_poses(after_failed_optimization, 2, 1));
    CHECK(after_failed_optimization[0] == positions[2]);
    fail_slam = 0;

    pcl_pose_t pose = {.tof = {{100}}};
    for (int i = 5; i < 30; i++) {
        for (int dir = 0; dir < 4; dir++)
            for (int col = 0; col < 8; col++)
                pose.tof[dir][col] = 100;
        CHECK(pcl_add_augmented_pose(pose));
    }
    icp_points_t points;
    pcl_extract_scan(0, 30, &points, 1400);
    CHECK(points.num == SIZE_POINTS);
    pcl_extract_scan(-1, 30, &points, 1400);
    CHECK(points.num == 0);
    pcl_extract_scan(0, 31, &points, 1400);
    CHECK(points.num == 0);
    pcl_extract_scan(3, 3, &points, 1400);
    CHECK(points.num == 0);
    pcl_extract_scan(0, 30, &points, -1);
    CHECK(points.num == 0);
    pcl_extract_scan(0, 30, NULL, 1400);

    for (int i = 0; i < 60; i++)
        pcl_add_constraint(2, 4, correction);
    for (int i = 30; i < 3010; i++)
        CHECK(pcl_add_augmented_pose(pose) == (i < 3000));
    CHECK(pcl_get_pose_count() == 3000);
    return 0;
}
