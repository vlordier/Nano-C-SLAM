/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#include "graph-based-slam.h"
#include "pmsis.h"

#define FREQU_FC 400 * 1000 * 1000
#define FREQU_CL 400 * 1000 * 1000

struct pi_device cluster_dev;
struct pi_cluster_conf cl_conf;

#include "external_point_cloud.h"
#include "gpio_config.h"
#include "icp/icp.h"
#include "pmsis.h"
#include "point_cloud.h"
#include "rcm-sparse-matrix.h"
#include "spi_buffers.h"
#include "spi_cmds_decode.h"
#include "spi_driver.h"

static uint8_t *spi_buf, *spi_buf_t;
static float requested_poses[3 * 100];
PI_L2 icp_points_t ref_scan, cur_scan;

int get_spi_command(uint8_t *spi_cmd, uint8_t len) {
    if (spi_cmd[0] == 'S' && spi_cmd[len - 1] == 'E')
        return spi_cmd[1];
    else
        return -1;
}

uint8_t verbose = 1;

#if defined(NANO_C_SLAM_GVSOC_SPI_TRACE) ||                                    \
    defined(NANO_C_SLAM_GVSOC_SPI_MAIN_PROBE)
#define NANO_C_SLAM_GVSOC_HAS_PACKET_COUNTERS 1
#endif
#ifndef NANO_C_SLAM_GVSOC_SPI_MAIN_PROBE_PACKET_COUNT
#define NANO_C_SLAM_GVSOC_SPI_MAIN_PROBE_PACKET_COUNT 2
#endif

#ifdef NANO_C_SLAM_GVSOC_SPI_TRACE
#include "gap9_gvsoc_spi_trace_transport.h"
#endif

void slam_loop() {
    gpio_init();
    if (spi_buffers_allocate(&spi_buf, &spi_buf_t, pi_l2_malloc, pi_l2_free) !=
        0) {
        printf("NANO_C_SLAM_SPI:error stage=buffer_alloc status=-1\n");
        pmsis_exit(-1);
        return;
    }
    if (spi_slave_driver_init() != 0) {
        pmsis_exit(-1);
        return;
    }

    pcl_pose_t pose0 = {0};
    ext_aug_pose_t ext_pose0 = {0};
    ext_pose_upd_t ext_pose_upd0 = {0};

    int16_t scan_info[2];
    int16_t pose_request[2];
    float icp_res[3];
    slam_job_t slam_job = {0};
    icp_job_t icp_job = {0};

#ifdef NANO_C_SLAM_GVSOC_HAS_PACKET_COUNTERS
    int16_t pose_packets_decoded = 0;
    int16_t poses_inserted = 0;
    int16_t bad_pose_checksums = 0;
    int16_t scan_packets_decoded = 0;
    int16_t scans_registered = 0;
    int16_t optimize_packets = 0;
    uint16_t processed_packets = 0;
#endif

    while (1) {
#ifdef NANO_C_SLAM_GVSOC_SPI_TRACE
        if (!gap9_gvsoc_spi_trace_has_packet()) break;
#endif
#ifdef NANO_C_SLAM_GVSOC_SPI_MAIN_PROBE
        if (processed_packets >= NANO_C_SLAM_GVSOC_SPI_MAIN_PROBE_PACKET_COUNT)
            break;
#endif
        // Wait new SPI packet
        spi_rcv_blocking(spi_buf, SPI_PKT_SIZE);
        pi_time_wait_us(100);
        int cmd = get_spi_command(spi_buf, SPI_PKT_SIZE);
#ifdef NANO_C_SLAM_GVSOC_SPI_TRACE
        int trace_decode_result = SPI_ERR;
        int trace_checksum_valid = -1;
        int16_t trace_packet_id = -1;
#endif
#ifdef NANO_C_SLAM_GVSOC_SPI_TRACE
        if (cmd == POSE_CMD || cmd == SCAN_CMD)
            memcpy(&trace_packet_id, &spi_buf[2], sizeof(trace_packet_id));
#endif

        switch (cmd) {
        case POSE_CMD: // Received augmented pose: timestamp + id + pose value +
                       // 4 x ToF
        {
            spi_transfer_t result =
                spi_decode_pose(spi_buf, SPI_PKT_SIZE, &pose0);
#ifdef NANO_C_SLAM_GVSOC_SPI_TRACE
            trace_decode_result = result;
            trace_checksum_valid = result == SPI_OK;
#endif
#ifdef NANO_C_SLAM_GVSOC_HAS_PACKET_COUNTERS
            if (result == SPI_OK) pose_packets_decoded++;
            if (result == SPI_CHECKSUM_ERR) bad_pose_checksums++;
#endif
            if (result == SPI_OK) {
                if (pcl_add_augmented_pose(pose0)) {
#ifdef NANO_C_SLAM_GVSOC_HAS_PACKET_COUNTERS
                    poses_inserted++;
#endif
                }
            }
            break;
        }

        case EXT_POSE_CMD: // Received external augmented pose
            if (spi_decode_ext_pose(spi_buf, SPI_PKT_SIZE, &ext_pose0) ==
                SPI_OK) {
                add_ext_aug_pose_to_buf(ext_pose0);
            }
            break;

        case UPD_EXT_POSE_CMD: // Received external pose update
            if (spi_decode_ext_pose_upd(spi_buf, SPI_PKT_SIZE,
                                        &ext_pose_upd0) == SPI_OK) {
                add_pose_upd_to_buf(ext_pose_upd0);
                // printf("Pose %d from %d: %.1f %.1f %.1f\n",
                // ext_pose_upd0.pose_id, ext_pose_upd0.drone_id,
                // ext_pose_upd0.pos[0], ext_pose_upd0.pos[1],
                // ext_pose_upd0.pos[2]);
            }
            break;

        case SCAN_CMD: // New internal scan
        {
            spi_transfer_t result =
                spi_decode_scan_info(spi_buf, SPI_PKT_SIZE, scan_info);
#ifdef NANO_C_SLAM_GVSOC_SPI_TRACE
            trace_decode_result = result;
            trace_checksum_valid = result == SPI_OK;
#endif
            if (result == SPI_OK) {
#ifdef NANO_C_SLAM_GVSOC_HAS_PACKET_COUNTERS
                scan_packets_decoded++;
                uint16_t scans_before = pcl_get_scan_pose_count();
#endif
                set_cf_gpio(1); // GAP9 busy
                int16_t scan_pose_id = scan_info[0];
                add_scan_pose_id(scan_pose_id);
#ifdef NANO_C_SLAM_GVSOC_HAS_PACKET_COUNTERS
                if (pcl_get_scan_pose_count() > scans_before)
                    scans_registered++;
#endif
                if (verbose)
                    printf("Int scan %d rcv! %d poses\n\n", scan_pose_id,
                           pcl_get_pose_count());
                // print_poses();
                set_cf_gpio(0); // GAP9 free
            }
            break;
        }

        case OPTIMIZE_CMD: // Optimize pose graph
#ifdef NANO_C_SLAM_GVSOC_HAS_PACKET_COUNTERS
            optimize_packets++;
#endif
            set_cf_gpio(1); // GAP9 busy
            if (verbose) printf("Opt..\n");

            // Implement new ext constraints
            check_for_external_constraints();

            // Update modified ext constraints
            update_external_constraints();

            // Optimize graph
            pcl_optimize(&slam_job); // Optimize graph
            set_cf_gpio(0);          // GAP9 free

            // Send feedback to the STM32
            memcpy(spi_buf_t, &slam_job, sizeof(slam_job_t));
#ifdef NANO_C_SLAM_GVSOC_SPI_MAIN_PROBE_SKIP_SPI_TX
            printf("NANO_C_SLAM_GVSOC_SPI_MAIN:response_phase=skipped "
                   "reason=GVSOC_slave_TX_not_validated\n");
#else
            spi_send_blocking(spi_buf_t, sizeof(slam_job_t));
#endif
            if (verbose) printf("\n");
            break;

        case POSE_REQ_CMD: // Send poses back to STM32
            if (spi_decode_pose_req(spi_buf, SPI_PKT_SIZE, pose_request) ==
                SPI_OK) {
                int16_t from_id = pose_request[0];
                int16_t nr_of_poses = pose_request[1];
                if (nr_of_poses > 100 ||
                    !pcl_to_poses(requested_poses, from_id, nr_of_poses))
                    break;

                spi_send_blocking((uint8_t *)requested_poses,
                                  3 * sizeof(float) * nr_of_poses);
            }
            break;
        }
#ifdef NANO_C_SLAM_GVSOC_SPI_TRACE
        printf("NANO_C_SLAM_GVSOC_SPI:packet index=%u command=%d id=%d "
               "length=%d checksum=%d decoder=%d poses=%d scans=%d "
               "constraints=%ld response_bytes=%u\n",
               gap9_gvsoc_spi_trace_packet_count() - 1, cmd, trace_packet_id,
               SPI_PKT_SIZE, trace_checksum_valid, trace_decode_result,
               pcl_get_pose_count(), pcl_get_scan_pose_count(),
               (long)pcl_get_constraint_count(),
               (unsigned)gap9_gvsoc_spi_trace_response_size());
#endif
#ifdef NANO_C_SLAM_GVSOC_SPI_MAIN_PROBE
        processed_packets++;
#endif
    }

#ifdef NANO_C_SLAM_GVSOC_SPI_MAIN_PROBE
    int passed;
#if NANO_C_SLAM_GVSOC_SPI_MAIN_PROBE_PACKET_COUNT == 2
    passed = processed_packets == 2 && pose_packets_decoded == 1 &&
             poses_inserted == 1 && bad_pose_checksums == 0 &&
             scan_packets_decoded == 1 && scans_registered == 1 &&
             pcl_get_pose_count() == 1 && pcl_get_scan_pose_count() == 1;
    printf("NANO_C_SLAM_GVSOC_SPI_MAIN:run_%s packets=%u pose_decoded=%d "
           "pose_inserted=%d scans_decoded=%d scans_registered=%d "
           "poses=%d scan_origins=%u\n",
           passed ? "ok" : "fail", processed_packets, pose_packets_decoded,
           poses_inserted, scan_packets_decoded, scans_registered,
           pcl_get_pose_count(), pcl_get_scan_pose_count());
#elif NANO_C_SLAM_GVSOC_SPI_MAIN_PROBE_PACKET_COUNT == 66
    passed = processed_packets == 66 && pose_packets_decoded == 60 &&
             poses_inserted == 60 && bad_pose_checksums == 1 &&
             scan_packets_decoded == 4 && scans_registered == 3 &&
             optimize_packets == 1 && pcl_get_pose_count() == 60 &&
             pcl_get_scan_pose_count() == 3 &&
             pcl_get_constraint_count() == 2 &&
             pcl_get_optimizer_input_unique_edge_count() == 2 &&
             slam_job.number_of_poses == 60 && slam_job.main_graph_size == 3 &&
             slam_job.has_nan == 0 && slam_job.main_solver_returned == 1 &&
             slam_job.successful_solver_call_count >= 1 &&
             slam_job.optimization_completed == 1;
    printf("NANO_C_SLAM_GVSOC_SPI_MAIN:mapping_%s packets=%u "
           "pose_decoded=%d pose_inserted=%d bad_pose_checksum=%d "
           "scans_decoded=%d scans_registered=%d constraints=%ld "
           "optimizer_edges=%ld optimize=%d graph=%d solver_calls=%d "
           "response_expected_hex=",
           passed ? "ok" : "fail", processed_packets, pose_packets_decoded,
           poses_inserted, bad_pose_checksums, scan_packets_decoded,
           scans_registered, (long)pcl_get_constraint_count(),
           (long)pcl_get_optimizer_input_unique_edge_count(), optimize_packets,
           slam_job.main_graph_size, slam_job.successful_solver_call_count);
    const uint8_t *response_bytes = (const uint8_t *)&slam_job;
    for (size_t index = 0; index < sizeof(slam_job); ++index)
        printf("%02x", response_bytes[index]);
    printf("\n");
#else
#error "SPI main probe packet count must be 2 or 66"
#endif
    // Keep the simulated target alive long enough for the host control script
    // to drain its asynchronous SPI RX callback and persist the transfer.
    pi_time_wait_us(100000);
    pmsis_exit(passed ? 0 : -1);
#endif

#ifdef NANO_C_SLAM_GVSOC_SPI_TRACE
    int32_t accepted_constraints = pcl_get_constraint_count();
    int32_t optimizer_edges = pcl_get_optimizer_input_unique_edge_count();
    int16_t edge0_from = -1;
    int16_t edge0_to = -1;
    int16_t edge1_from = -1;
    int16_t edge1_to = -1;
    pcl_get_constraint_endpoints(0, &edge0_from, &edge0_to);
    pcl_get_constraint_endpoints(1, &edge1_from, &edge1_to);
    int passed = gap9_gvsoc_spi_trace_validate(
        pose_packets_decoded, poses_inserted, bad_pose_checksums,
        scan_packets_decoded, scans_registered, optimize_packets,
        accepted_constraints, optimizer_edges, &slam_job);
    printf("NANO_C_SLAM_GVSOC_SPI:mapping_%s packets=%u pose_decoded=%d "
           "pose_inserted=%d bad_pose_checksum=%d scans_decoded=%d "
           "scans_registered=%d constraints=%ld optimizer_edges=%ld "
           "constraint_endpoints=%d-%d,%d-%d optimize=%d graph=%d "
           "solver_calls=%d response_bytes=%u\n",
           passed ? "ok" : "fail", gap9_gvsoc_spi_trace_packet_count(),
           pose_packets_decoded, poses_inserted, bad_pose_checksums,
           scan_packets_decoded, scans_registered, (long)accepted_constraints,
           (long)optimizer_edges, edge0_from, edge0_to, edge1_from, edge1_to,
           optimize_packets, slam_job.main_graph_size,
           slam_job.successful_solver_call_count,
           (unsigned)gap9_gvsoc_spi_trace_response_size());
    pmsis_exit(passed ? 0 : -1);
#endif
}

int main_func(void) {
    pi_freq_set(PI_FREQ_DOMAIN_FC, FREQU_FC);
    pi_freq_set(PI_FREQ_DOMAIN_CL, FREQU_CL);
    int32_t cur_fc_freq = pi_freq_get(PI_FREQ_DOMAIN_FC);
    int32_t cur_cl_freq = pi_freq_get(PI_FREQ_DOMAIN_CL);

    if (verbose)
        printf("FC frequency : %ld\nCL frequency : %ld\n", (long)cur_fc_freq,
               (long)cur_cl_freq);

    /* Init cluster configuration structure. */
    pi_cluster_conf_init(&cl_conf);
    cl_conf.id = 0;

    /* Configure & open cluster. */
    pi_open_from_conf(&cluster_dev, &cl_conf);
    if (pi_cluster_open(&cluster_dev)) {
        if (verbose) printf("Cluster open failed !\n");
        pmsis_exit(-1);
    }

    slam_loop();

    return 0;
}

int main(void) { return main_func(); }
