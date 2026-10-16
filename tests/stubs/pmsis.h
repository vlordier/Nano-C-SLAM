#ifndef TEST_PMSIS_H
#define TEST_PMSIS_H

#include <stdint.h>

#define PI_L2
#define PI_CL_L1

struct pi_device {
    int unused;
};
struct pi_cluster_task {
    void (*entry)(void *);
    void *arg;
};
typedef struct {
    int unused;
} pi_evt_t;

uint32_t pi_time_get_us(void);
void pi_time_wait_us(uint32_t us);
int pi_core_id(void);
void pi_cl_team_barrier(void);
void pi_cl_team_fork(int cores, void (*entry)(void *), void *arg);
void pi_evt_sig_init(pi_evt_t *evt);
void pi_evt_wait(pi_evt_t *evt);
struct pi_cluster_task *pi_cluster_task(struct pi_cluster_task *task,
                                        void (*entry)(void *), void *arg);
void pi_cluster_send_task_to_cl_async(struct pi_device *device,
                                      struct pi_cluster_task *task,
                                      pi_evt_t *completion);

#endif
