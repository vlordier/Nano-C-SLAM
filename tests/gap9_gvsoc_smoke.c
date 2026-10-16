#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../cslam-gap9-app/icp/icp.h"
#include "stubs/pmsis.h"

struct pi_device cluster_dev;

uint32_t pi_time_get_us(void) {
    return 0;
}
void pi_time_wait_us(uint32_t us) {
    (void)us;
}
int pi_core_id(void) {
    return 0;
}
void pi_cl_team_barrier(void) {}
void pi_cl_team_fork(int cores, void (*entry)(void *), void *arg) {
    (void)cores;
    entry(arg);
}
void pi_evt_sig_init(pi_evt_t *evt) {
    (void)evt;
}
void pi_evt_wait(pi_evt_t *evt) {
    (void)evt;
}
struct pi_cluster_task *pi_cluster_task(struct pi_cluster_task *task,
                                        void (*entry)(void *), void *arg) {
    task->entry = entry;
    task->arg = arg;
    return task;
}
void pi_cluster_send_task_to_cl_async(struct pi_device *device,
                                      struct pi_cluster_task *task,
                                      pi_evt_t *completion) {
    (void)device;
    task->entry(task->arg);
    (void)completion;
}

void cl_memcpy_from_l2(void *dest, const void *src, int32_t size) {
    (void)size;
    memcpy(dest, src, (size_t)size);
}

void cl_memcpy_to_l2(void *dest, void *src, int32_t size) {
    memcpy(dest, src, (size_t)size);
}

int main(void) {
    icp_points_t reference = {0};
    icp_points_t current = {0};
    icp_job_t job = {0};
    float result[3] = {0};

    reference.num = 4;
    current.num = 4;
    reference.items[0] = (icp_point_t){0.0f, 0.0f};
    reference.items[1] = (icp_point_t){1.0f, 0.0f};
    reference.items[2] = (icp_point_t){0.0f, 1.0f};
    reference.items[3] = (icp_point_t){1.0f, 1.0f};
    for (size_t i = 0; i < reference.num; ++i) {
        current.items[i].x = reference.items[i].x + 0.25f;
        current.items[i].y = reference.items[i].y - 0.125f;
    }

    icp(&current, &reference, 12, 0.4f, result, &job);
    if (job.has_nan || !isfinite(result[0]) || !isfinite(result[1]) ||
        !isfinite(result[2])) {
        printf("NANO_C_SLAM_GAP9:icp_fail non-finite result\n");
        return 1;
    }
    printf("NANO_C_SLAM_GAP9:icp_ok error=%f dx=%f dy=%f yaw=%f\n", job.error,
           result[0], result[1], result[2]);
    return 0;
}
