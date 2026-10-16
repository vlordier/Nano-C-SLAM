/* Run deterministic multi-scan input through the owned GAP9 SLAM pipeline. */
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "point_cloud.h"
#include "pmsis.h"

struct pi_device cluster_dev;
struct pi_cluster_conf cl_conf;
uint8_t verbose = 0;

static int run_mapping_fixture(void) {
    slam_job_t job = {0};
    const int16_t ranges_mm[4] = {1000, 1200, 900, 1100};

    for (int16_t id = 0; id < 60; id++) {
        const float angle = (float)id * 0.02f;
        pcl_pose_t pose = {0};
        pose.id = id;
        pose.timestamp = (int32_t)id * 100;
        pose.pos[0] = 0.15f * cosf(angle);
        pose.pos[1] = 0.15f * sinf(angle);
        pose.pos[2] = angle + (float)M_PI_2;
        for (int direction = 0; direction < 4; direction++) {
            for (int column = 0; column < 8; column++) {
                pose.tof[direction][column] = -1;
            }
            pose.tof[direction][3] = ranges_mm[direction];
        }
        if (!pcl_add_augmented_pose(pose)) {
            printf("NANO_C_SLAM_GVSOC:mapping_fail rejected_pose=%d\n", id);
            return -1;
        }

        if ((id + 1) % 20 == 0) {
            add_scan_pose_id((uint16_t)(id - 19));
            pcl_optimize(&job);
        }
    }

    const int16_t pose_count = pcl_get_pose_count();
    const uint16_t scan_count = pcl_get_scan_pose_count();
    icp_points_t map = {0};
    pcl_extract_scan(0, pose_count, &map, 1400);
    const int passed = pose_count == 60 && scan_count == 3 && map.num == 240 &&
                       job.number_of_poses == 60 && job.main_graph_size == 3 &&
                       job.has_nan == 0;
    if (!passed) {
        printf("NANO_C_SLAM_GVSOC:mapping_fail poses=%d scans=%u points=%zu "
               "optimized=%d graph=%d has_nan=%d\n",
               pose_count, scan_count, map.num, job.number_of_poses,
               job.main_graph_size, job.has_nan);
        return -1;
    }

    printf("NANO_C_SLAM_GVSOC:mapping_ok poses=%d scans=%u map_points=%zu "
           "optimized=%d graph=%d has_nan=%d\n",
           pose_count, scan_count, map.num, job.number_of_poses,
           job.main_graph_size, job.has_nan);
    return 0;
}

int main(void) {
#ifdef NANO_C_SLAM_TARGET_GVSOC
    pi_cluster_conf_init(&cl_conf);
    cl_conf.id = 0;
    pi_open_from_conf(&cluster_dev, &cl_conf);
    if (pi_cluster_open(&cluster_dev)) {
        printf("NANO_C_SLAM_GVSOC:cluster_open_fail\n");
        pmsis_exit(-1);
        return -1;
    }
#endif

    const int result = run_mapping_fixture();
#ifdef NANO_C_SLAM_TARGET_GVSOC
    pmsis_exit(result == 0 ? 0 : -1);
#endif
    return result == 0 ? 0 : 1;
}
