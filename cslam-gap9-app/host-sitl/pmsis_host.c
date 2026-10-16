/* CPU implementation of the PMSIS cluster boundary for host SITL only. */
#include <stdint.h>
#include <string.h>

#include "pmsis.h"

struct pi_device cluster_dev;
static int current_core;
static uint32_t elapsed_us;

uint32_t pi_time_get_us(void) { return elapsed_us; }

void pi_time_wait_us(uint32_t us) { elapsed_us += us; }

int pi_core_id(void) { return current_core; }

void pi_cl_team_barrier(void) {}

void pi_cl_team_critical_enter(void) {}

void pi_cl_team_critical_exit(void) {}

void pi_cl_team_fork(int cores, void (*entry)(void *), void *arg) {
    for (int core = 0; core < cores; core++) {
        current_core = core;
        entry(arg);
    }
    current_core = 0;
}

void pi_evt_sig_init(pi_evt_t *evt) { (void)evt; }

void pi_evt_wait(pi_evt_t *evt) { (void)evt; }

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
    (void)completion;
    task->entry(task->arg);
}

void cl_memcpy_from_l2(void *dest, const void *src, int32_t size) {
    memcpy(dest, src, (size_t)size);
}

void cl_memcpy_to_l2(void *dest, void *src, int32_t size) {
    memcpy(dest, src, (size_t)size);
}
