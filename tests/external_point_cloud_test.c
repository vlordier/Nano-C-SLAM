#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "../cslam-gap9-app/point-cloud/external_point_cloud.h"

#define CHECK(condition) \
    do { \
        if (!(condition)) { \
            fprintf(stderr, "check failed at line %d: %s\n", __LINE__, #condition); \
            abort(); \
        } \
    } while (0)

extern ext_scan_t ext_scan_table[20];

int main(void) {
    ext_aug_pose_t pose = {.drone_id = 7};
    pose.id = 0;
    pose.pos[0] = NAN;
    add_ext_aug_pose_to_buf(pose);
    CHECK(!ext_scan_table[0].taken);
    pose.pos[0] = 0.0f;
    pose.tof[DIR_FRONT][0] = -2;
    add_ext_aug_pose_to_buf(pose);
    CHECK(!ext_scan_table[0].taken);
    pose.tof[DIR_FRONT][0] = 0;

    add_ext_aug_pose_to_buf(pose);
    pose.drone_id = 8;
    for (int sample = 0; sample < 19; sample++)
        add_ext_aug_pose_to_buf(pose);
    CHECK(!ext_scan_table[0].taken);

    pose.drone_id = 7;
    for (int sample = 1; sample < 20; sample++) {
        pose.id = sample;
        add_ext_aug_pose_to_buf(pose);
    }
    CHECK(ext_scan_table[0].taken);
    for (int sample = 0; sample < 20; sample++)
        CHECK(ext_scan_table[0].ext_aug_poses[sample].drone_id == 7);

    for (int scan = 1; scan < 20; scan++) {
        pose.id = scan;
        for (int sample = 0; sample < 20; sample++)
            add_ext_aug_pose_to_buf(pose);
        CHECK(ext_scan_table[scan].taken);
        CHECK(ext_scan_table[scan].pose_id == scan);
    }

    pose.id = 100;
    add_ext_aug_pose_to_buf(pose);
    CHECK(ext_scan_table[19].ext_aug_poses[0].id == 19);

    ext_scan_table[5].taken = 0;
    for (int sample = 1; sample < 20; sample++)
        add_ext_aug_pose_to_buf(pose);
    CHECK(!ext_scan_table[5].taken);

    for (int sample = 0; sample < 20; sample++)
        add_ext_aug_pose_to_buf(pose);
    CHECK(ext_scan_table[5].taken);
    CHECK(ext_scan_table[5].pose_id == 100);
    CHECK(ext_scan_table[19].pose_id == 19);

    icp_points_t points;
    get_external_scan(-1, &points, 1400);
    CHECK(points.num == 0);
    get_external_scan(20, &points, 1400);
    CHECK(points.num == 0);
    get_external_scan(5, &points, -1);
    CHECK(points.num == 0);
    get_external_scan(5, NULL, 1400);
    get_external_scan(5, &points, 1400);
    CHECK(points.num == 640);

    float z[3] = {1, 2, 3};
    for (int i = 0; i < 60; i++)
        force_external_constraint(i, z);
    CHECK(external_constraints_count() == 50);
    CHECK(get_ext_constraint(49).fromIdx == 49);
    CHECK(get_ext_constraint(-1).fromIdx == 0);
    CHECK(get_ext_constraint(50).fromIdx == 0);

    ext_pose_upd_t update = {0};
    for (int i = 0; i < 250; i++)
        add_pose_upd_to_buf(update);
    return 0;
}
