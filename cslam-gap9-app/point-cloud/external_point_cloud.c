/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#include "external_point_cloud.h"
#include "graph-based-slam.h"
#include "icp.h"
#include "pmsis.h"
#include "point_cloud.h"
#include "transformations.h"
#include "utils_array.h"
#include "utils_math.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define MAX_SCANS (20)
#define MAX_POSE_UPDATES (200)
#define MAX_EXTERNAL_CONSTRAINTS (50)

PI_L2 icp_points_t ref_ext_scan, cur_self_scan;

// Table storing the external scans (as batches of 20 external augmented poses)
ext_scan_t ext_scan_table[MAX_SCANS];
static int16_t ext_scan_table_idx = -1;
static int16_t ext_aug_pose_idx = 0;
static int16_t ext_drop_remaining = 0;

// Table storing the pose updates
static ext_pose_upd_t ext_pose_upd_table[MAX_POSE_UPDATES];
static int16_t ext_pose_upd_cnt = 0;

// Table storing the validated external constraints (inter-drone loop closures)
static ext_const_info_t ext_const_table[MAX_EXTERNAL_CONSTRAINTS];
static uint16_t ext_const_cnt = 0;

static icp_job_t icp_job = {0};

extern uint8_t verbose;

static int finite_pose(const float pose[3]) {
    return isfinite(pose[0]) && isfinite(pose[1]) && isfinite(pose[2]);
}

void add_pose_upd_to_buf(ext_pose_upd_t pose_upd) {
    if (ext_pose_upd_cnt >= MAX_POSE_UPDATES || pose_upd.pose_id < 0 ||
        pose_upd.drone_id < 0 || !finite_pose(pose_upd.pos))
        return;
    ext_pose_upd_table[ext_pose_upd_cnt++] = pose_upd;
}

void reset_pose_upd_buf() { ext_pose_upd_cnt = 0; }

void add_ext_aug_pose_to_buf(ext_aug_pose_t ext_pose) {
    if (ext_pose.id < 0 || ext_pose.drone_id < 0 || !finite_pose(ext_pose.pos))
        return;
    for (int direction = 0; direction < 4; ++direction) {
        for (int column = 0; column < 8; ++column) {
            if (ext_pose.tof[direction][column] < -1) return;
        }
    }

    if (ext_drop_remaining > 0) {
        ext_drop_remaining--;
        return;
    }
    if (ext_aug_pose_idx == 0) {
        ext_scan_table_idx = -1;
        for (int16_t i = 0; i < MAX_SCANS; i++) {
            if (ext_scan_table[i].taken == 0) {
                ext_scan_table_idx = i;
                ext_scan_table[i].pose_id = ext_pose.id;
                ext_scan_table[i].from_drone = ext_pose.drone_id;
                break;
            }
        }
        if (ext_scan_table_idx < 0) {
            // Keep the next batch aligned if a slot opens mid-scan.
            ext_drop_remaining = EXT_SCAN_POSE_COUNT - 1;
            return;
        }
    }

    ext_scan_table[ext_scan_table_idx].ext_aug_poses[ext_aug_pose_idx] =
        ext_pose;
    ext_aug_pose_idx++;
    if (ext_aug_pose_idx == EXT_SCAN_POSE_COUNT) {
        ext_aug_pose_idx = 0;
        ext_scan_table[ext_scan_table_idx].taken = 1;
        // printf("Added scan %d to table\n", ext_scan_table_idx);
    }
}

uint8_t has_external_constraint(int16_t id) {
    for (uint16_t i = 0; i < ext_const_cnt; i++) {
        if (ext_const_table[i].cur_pose_id == id) return 1;
    }
    return 0;
}

uint16_t external_constraints_count() { return ext_const_cnt; }

constraint get_ext_constraint(int16_t idx) {
    constraint c = {0};
    if (idx < 0 || idx >= ext_const_cnt) return c;
    c.fromIdx = ext_const_table[idx].cur_pose_id;
    c.toIdx = 0;
    memcpy(c.z, ext_const_table[idx].z, 3 * sizeof(float));
    return c;
}

void add_external_constraint(ext_scan_t *ext_scan, int16_t cur_scan_id,
                             float *icp_res) {
    if (ext_const_cnt >= MAX_EXTERNAL_CONSTRAINTS || ext_scan == NULL ||
        icp_res == NULL || !ext_scan->taken || ext_scan->from_drone < 0 ||
        ext_scan->pose_id < 0 || cur_scan_id < 0 ||
        !finite_pose(ext_scan->ext_aug_poses[0].pos) || !finite_pose(icp_res))
        return;
    ext_const_info_t c = {0};
    c.from_drone = ext_scan->from_drone;
    c.ref_pose_id = ext_scan->pose_id;
    c.cur_pose_id = cur_scan_id;

    float REF_T[3][3];
    float REF_INV_T[3][3];
    float CUR_T[3][3];
    float ICP_T[3][3];
    float T[3][3];
    float cur_pose[3];
    float ref_pose[3];

    memcpy(ref_pose, ext_scan->ext_aug_poses[0].pos, 3 * sizeof(float));
    if (!pcl_to_poses(cur_pose, cur_scan_id, 1) || !finite_pose(cur_pose))
        return;

    v2t(ref_pose, REF_T);
    v2t(cur_pose, CUR_T);
    v2t(icp_res, ICP_T);

    matinv_3x3(REF_T, REF_INV_T);
    matmul_3x3(REF_INV_T, ICP_T, T);
    matmul_3x3(T, CUR_T, T);

    t2v(T, c.t);

    // if (verbose) printf("Ref: %.3f  %.3f  %.3f\n", ref_pose[0], ref_pose[1],
    // ref_pose[2]); if (verbose) printf("Cur: %.3f  %.3f  %.3f\n", cur_pose[0],
    // cur_pose[1], cur_pose[2]);

    // Compute z
    float CUR_ROT_T[3][3];
    float pose0[3];
    float POSE0_T[3][3];

    if (!pcl_to_poses(pose0, 0, 1) || !finite_pose(pose0)) return;

    v2t(pose0, POSE0_T);

    matmul_3x3(ICP_T, CUR_T, CUR_ROT_T);
    matinv_3x3(CUR_ROT_T, CUR_ROT_T);
    matmul_3x3(CUR_ROT_T, POSE0_T, CUR_ROT_T);

    t2v(CUR_ROT_T, c.z);

    if (!finite_pose(c.t) || !finite_pose(c.z)) return;

    ext_const_table[ext_const_cnt++] = c;

    if (verbose) printf("z: %.2f, %.2f, %.2f\n", c.z[0], c.z[1], c.z[2]);
}

void force_external_constraint(int16_t pose_id, float *z) {
    if (ext_const_cnt >= MAX_EXTERNAL_CONSTRAINTS || pose_id < 0 || z == NULL ||
        !finite_pose(z))
        return;
    ext_const_info_t c = {0};
    c.cur_pose_id = pose_id;
    c.z[0] = z[0];
    c.z[1] = z[1];
    c.z[2] = z[2];
    ext_const_table[ext_const_cnt++] = c;
}

uint8_t is_matched_with_external(uint16_t int_pose_id) {
    for (int16_t i = 0; i < ext_const_cnt; i++) {
        if (int_pose_id == ext_const_table[i].cur_pose_id) {
            return 1;
        }
    }
    return 0;
}

void check_for_external_constraints() {
    if (ext_const_cnt >= MAX_EXTERNAL_CONSTRAINTS) return;
    for (int16_t i = 0; i < MAX_SCANS;
         i++) { // Loop through the external scan register
        if (!ext_scan_table[i].taken) continue;

        uint16_t int_scan_count = pcl_get_scan_pose_count();
        for (int16_t j = 0; j < int_scan_count;
             j++) { // Loop through the self scan poses
            int16_t cur_scan_id = pcl_get_scan_pose_id(j);

            if (is_matched_with_external(cur_scan_id)) continue;

            float ref_pose[3];
            float cur_pose[3];
            pcl_to_poses(cur_pose, cur_scan_id, 1);
            memcpy(ref_pose, ext_scan_table[i].ext_aug_poses[0].pos,
                   sizeof(ref_pose));

            // Check if the self and external scans are taken in the same
            // location
            float distance = euclidean_dist(ref_pose, cur_pose);
            if (distance < 1.2f) {
                // Extract scans
                get_external_scan(i, &ref_ext_scan, 1400);
                pcl_extract_scan(cur_scan_id, cur_scan_id + 20, &cur_self_scan,
                                 1400);
                if (ref_ext_scan.num == 0 || cur_self_scan.num == 0) continue;

                float icp_res[3];
                icp(&ref_ext_scan, &cur_self_scan, 25, 0.4f, icp_res, &icp_job);

                // Add new external constraint
                if ((icp_job.has_nan == 0) && (icp_job.error < 0.01)) {
                    add_external_constraint(&ext_scan_table[i], cur_scan_id,
                                            icp_res);
                    if (verbose)
                        printf("Created Ext constraint %d -> %d\n", cur_scan_id,
                               ext_scan_table[i].pose_id);
                    ext_scan_table[i].taken = 0;
                    if (ext_const_cnt >= MAX_EXTERNAL_CONSTRAINTS) return;
                    break;
                }
            }
        }
    }
}

void update_external_constraints() {
    for (int16_t i = 0; i < ext_pose_upd_cnt; i++) {
        ext_pose_upd_t pose_upd = ext_pose_upd_table[i];
        for (int16_t i = 0; i < ext_const_cnt; i++) {
            if ((ext_const_table[i].ref_pose_id == pose_upd.pose_id) &&
                (ext_const_table[i].from_drone == pose_upd.drone_id)) {
                float pose0[3];
                float POSE0_T[3][3];
                float REF_NEW_T[3][3];
                float T[3][3];
                if (!finite_pose(pose_upd.pos) || !pcl_to_poses(pose0, 0, 1) ||
                    !finite_pose(pose0))
                    break;

                v2t(pose0, POSE0_T);
                v2t(ext_const_table[i].t, T);
                v2t(pose_upd.pos, REF_NEW_T); // ref_pos_new
                matmul_3x3(REF_NEW_T, T, T);
                matinv_3x3(T, T);
                matmul_3x3(T, POSE0_T, T);

                float updated_z[3];
                t2v(T, updated_z);
                if (finite_pose(updated_z))
                    memcpy(ext_const_table[i].z, updated_z, sizeof(updated_z));
                // if (verbose) printf("Adjusted ext pose %d to %.2f %.2f
                // %.2f\n", ext_const_table[i].ref_pose_id,
                // ext_const_table[i].z[0], ext_const_table[i].z[1],
                // ext_const_table[i].z[2]);
                break;
            }
        }
    }
    reset_pose_upd_buf();
}

void get_external_scan(int16_t ext_scan_idx, icp_points_t *points,
                       int16_t range_limit) {

    const float offsets[4] = {0.025f, -0.025f, 0.02f, -0.02f};
    const float step = -((float)M_PI_4) / 8;

    if (points == NULL) return;
    points->num = 0;
    if (ext_scan_idx < 0 || ext_scan_idx >= MAX_SCANS || range_limit < 0 ||
        !ext_scan_table[ext_scan_idx].taken)
        return;

    for (int16_t pose_id = 0; pose_id < EXT_SCAN_POSE_COUNT; pose_id++) {

        float frame_x =
            ext_scan_table[ext_scan_idx].ext_aug_poses[pose_id].pos[0];
        float frame_y =
            ext_scan_table[ext_scan_idx].ext_aug_poses[pose_id].pos[1];
        float frame_yaw =
            ext_scan_table[ext_scan_idx].ext_aug_poses[pose_id].pos[2];
        if (!isfinite(frame_x) || !isfinite(frame_y) || !isfinite(frame_yaw))
            continue;

        for (int dir = 0; dir < 4; dir++) {

            float dir_yaw = frame_yaw;
            if (dir == DIR_BACK)
                dir_yaw += (float)M_PI;
            else if (dir == DIR_LEFT)
                dir_yaw += (float)M_PI_2;
            else if (dir == DIR_RIGHT)
                dir_yaw -= (float)M_PI_2;
            float angle = -4 * step + step / 2;
            for (int col = 0; col < 8; angle += step, col++) {

                // Check if measurement is valid
                int16_t measurement = ext_scan_table[ext_scan_idx]
                                          .ext_aug_poses[pose_id]
                                          .tof[dir][col];
                if (measurement < 0 || measurement > range_limit) continue;

                // Apply rotation and add point
                float dist_x = (float)measurement / 1000.0f;
                float dist_y = tanf(angle) * dist_x;
                dist_x += offsets[dir];
                icp_point_t p = {
                    .x = frame_x + dist_x * cosf(dir_yaw) -
                         dist_y * sinf(dir_yaw),
                    .y = frame_y + dist_x * sinf(dir_yaw) +
                         dist_y * cosf(dir_yaw),
                };
                if (!isfinite(p.x) || !isfinite(p.y)) continue;
                if (points->num >= SIZE_POINTS) return;
                points->items[points->num++] = p;
            }
        }
    }
}
