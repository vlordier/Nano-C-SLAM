/* Isolate cluster L1-to-FC L2 DMA behavior from SLAM and matrix code. */
#include <stdint.h>
#include <stdio.h>

#include "dma_transfers.h"
#include "pmsis.h"

struct pi_device cluster_dev;
static struct pi_cluster_task cluster_task;
static pi_evt_t cluster_done;

PI_L2 static uint32_t l2_source[4];
PI_L2 static uint32_t l2_wrapper_destination[4];
PI_L2 static uint32_t l2_sdk_destination[4];
PI_CL_L1 static uint32_t l1_source[4];
PI_CL_L1 static uint32_t l1_wrapper_readback[4];
PI_CL_L1 static uint32_t l1_sdk_readback[4];

static const uint32_t sentinel[4] = {
    UINT32_C(0x13579bdf),
    UINT32_C(0x2468ace0),
    UINT32_C(0xdeadbeef),
    UINT32_C(0x01020304),
};

static void run_dma_roundtrip(void *unused) {
    (void)unused;
    cl_memcpy_from_l2(l1_source, l2_source, sizeof(l1_source));
    cl_memcpy_to_l2(l2_wrapper_destination, l1_source, sizeof(l1_source));
    cl_memcpy_from_l2(l1_wrapper_readback, l2_wrapper_destination,
                      sizeof(l1_wrapper_readback));

    pi_cl_dma_copy_t sdk_copy = {0};
    sdk_copy.dir = PI_CL_DMA_DIR_LOC2EXT;
    sdk_copy.merge = 0;
    sdk_copy.size = sizeof(l1_source);
    sdk_copy.ext = (uint32_t)l2_sdk_destination;
    sdk_copy.loc = (uint32_t)l1_source;
    pi_cl_dma_memcpy(&sdk_copy);
    pi_cl_dma_wait(&sdk_copy);
    cl_memcpy_from_l2(l1_sdk_readback, l2_sdk_destination, sizeof(l1_sdk_readback));
}

int main(void) {
    struct pi_cluster_conf cluster_conf;
    pi_cluster_conf_init(&cluster_conf);
    cluster_conf.id = 0;
    pi_open_from_conf(&cluster_dev, &cluster_conf);
    if (pi_cluster_open(&cluster_dev)) {
        printf("NANO_C_SLAM_GVSOC_DMA:cluster_open_fail\n");
        pmsis_exit(-1);
        return -1;
    }

    for (size_t index = 0; index < 4; ++index) {
        l2_source[index] = sentinel[index];
        l2_wrapper_destination[index] = 0;
        l2_sdk_destination[index] = 0;
    }

    pi_evt_sig_init(&cluster_done);
    pi_cluster_send_task_to_cl_async(
        &cluster_dev, pi_cluster_task(&cluster_task, run_dma_roundtrip, NULL),
        &cluster_done);
    pi_evt_wait(&cluster_done);

    int source_matches = 1;
    int wrapper_destination_matches = 1;
    int wrapper_readback_matches = 1;
    int sdk_destination_matches = 1;
    int sdk_readback_matches = 1;
    for (size_t index = 0; index < 4; ++index) {
        source_matches &= l1_source[index] == sentinel[index];
        wrapper_destination_matches &= l2_wrapper_destination[index] == sentinel[index];
        wrapper_readback_matches &= l1_wrapper_readback[index] == sentinel[index];
        sdk_destination_matches &= l2_sdk_destination[index] == sentinel[index];
        sdk_readback_matches &= l1_sdk_readback[index] == sentinel[index];
    }

    if (!source_matches) {
        printf("NANO_C_SLAM_GVSOC_DMA:fail l2_to_l1=%d\n", source_matches);
        pmsis_exit(-1);
        return -1;
    }

    if (!wrapper_destination_matches && !wrapper_readback_matches &&
        !sdk_destination_matches && !sdk_readback_matches) {
        printf("NANO_C_SLAM_GVSOC_DMA:observed_l1_to_l2_gap "
               "wrapper=%08x sdk=%08x expected=%08x wrapper_addr=%08x "
               "sdk_addr=%08x\n",
               l2_wrapper_destination[0], l2_sdk_destination[0], sentinel[0],
               (uint32_t)l2_wrapper_destination, (uint32_t)l2_sdk_destination);
        pmsis_exit(0);
        return 0;
    }

    if (!wrapper_destination_matches || !wrapper_readback_matches ||
        !sdk_destination_matches || !sdk_readback_matches) {
        printf("NANO_C_SLAM_GVSOC_DMA:fail l2_to_l1=%d wrapper_l1_to_l2=%d "
               "wrapper_readback=%d sdk_l1_to_l2=%d sdk_readback=%d\n",
               source_matches, wrapper_destination_matches, wrapper_readback_matches,
               sdk_destination_matches, sdk_readback_matches);
        pmsis_exit(-1);
        return -1;
    }

    printf("NANO_C_SLAM_GVSOC_DMA:ok words=4 l2_to_l1=%d wrapper_l1_to_l2=%d "
           "sdk_l1_to_l2=%d\n",
           source_matches, wrapper_destination_matches, sdk_destination_matches);
    pmsis_exit(0);
    return 0;
}
