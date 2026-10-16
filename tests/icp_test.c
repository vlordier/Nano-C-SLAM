#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../cslam-gap9-app/icp/icp.h"

#define CHECK(condition) \
    do { \
        if (!(condition)) { \
            fprintf(stderr, "check failed at line %d: %s\n", __LINE__, #condition); \
            abort(); \
        } \
    } while (0)

struct pi_device cluster_dev;
static int current_core;

uint32_t pi_time_get_us(void) {
    return 1000;
}
void pi_time_wait_us(uint32_t us) {
    (void)us;
}
int pi_core_id(void) {
    return current_core;
}
void pi_cl_team_barrier(void) {}
void pi_cl_team_fork(int cores, void (*entry)(void *), void *arg) {
    for (int i = 0; i < cores; i++) {
        current_core = i;
        entry(arg);
    }
}
void pi_task_block(pi_task_t *task) {
    (void)task;
}
void pi_task_wait_on(pi_task_t *task) {
    (void)task;
}
struct pi_cluster_task *pi_cluster_task(struct pi_cluster_task *task,
                                        void (*entry)(void *), void *arg) {
    task->entry = entry;
    task->arg = arg;
    return task;
}
void pi_cluster_send_task_to_cl_async(struct pi_device *device,
                                      struct pi_cluster_task *task,
                                      pi_task_t *completion) {
    (void)device;
    (void)completion;
    task->entry(task->arg);
}
void cl_memcpy_from_l2(void *dest, const void *src, int32_t size) {
    memcpy(dest, src, size);
}
void cl_memcpy_to_l2(void *dest, void *src, int32_t size) {
    memcpy(dest, src, size);
}

int main(void) {
    icp_points_t reference = {.num = 1, .items = {{0, 0}}};
    icp_points_t moved = {.num = 0};
    float result[3] = {1, 1, 1};
    icp_job_t job;

    icp(&reference, &moved, 1, 0.4f, result, &job);
    CHECK(job.has_nan && isinf(job.error));
    CHECK(result[0] == 0 && result[1] == 0 && result[2] == 0);

    moved.num = 1;
    moved.items[0] = (icp_point_t){100, 100};
    icp(&reference, &moved, 1, 0.4f, result, &job);
    CHECK(job.has_nan && isinf(job.error));

    moved.num = 2;
    moved.items[0] = (icp_point_t){0.1f, 0};
    moved.items[1] = (icp_point_t){100, 0};
    icp(&reference, &moved, 1, 0.4f, result, &job);
    CHECK(!job.has_nan && isfinite(job.error));
    CHECK(fabsf(result[0] + 0.1f) < 0.0001f);
    CHECK(fabsf(result[1]) < 0.0001f);

    icp(&reference, &moved, 0, 0.4f, result, &job);
    CHECK(job.has_nan && isinf(job.error));

    moved.num = SIZE_POINTS + 1;
    icp(&reference, &moved, 1, 0.4f, result, &job);
    CHECK(job.has_nan && isinf(job.error));
    return 0;
}
