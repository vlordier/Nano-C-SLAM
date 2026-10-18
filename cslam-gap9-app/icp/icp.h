/*
 * Authors: The code is based on the implementation of Carl Friess in the work
 * Friess, Carl, et al. "Fully Onboard SLAM for Distributed Mapping with a Swarm
 * of Nano-Drones." arXiv preprint arXiv:2309.03678 (2023).
 * https://arxiv.org/pdf/2309.03678.pdf
 */

#ifndef SLAM_FIRMWARE_ICP_H
#define SLAM_FIRMWARE_ICP_H

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "pmsis.h"

#define SIZE_POINTS 700

typedef struct {
    float x;
    float y;
} icp_point_t;

typedef struct {
    size_t num;
    icp_point_t items[SIZE_POINTS];
} icp_points_t;

typedef struct {
    float error;
    int16_t time_ms;
    int16_t has_nan; // Nonzero when alignment is invalid or non-finite.
} icp_job_t;

typedef enum {
    ICP_RESULT_NO_MATCHES = 0,
    ICP_RESULT_INVALID_SOLVER = 1,
    ICP_RESULT_SENTINEL_ERROR = 2,
    ICP_RESULT_NONFINITE_CORRECTION = 3,
    ICP_RESULT_ERROR_REJECTED = 4,
    ICP_RESULT_ACCEPTED = 5,
} icp_result_status_t;

static inline const char *icp_result_status_name(icp_result_status_t status) {
    switch (status) {
    case ICP_RESULT_NO_MATCHES: return "no_matches";
    case ICP_RESULT_INVALID_SOLVER: return "invalid_solver";
    case ICP_RESULT_SENTINEL_ERROR: return "sentinel_error";
    case ICP_RESULT_NONFINITE_CORRECTION: return "nonfinite_correction";
    case ICP_RESULT_ERROR_REJECTED: return "error_rejected";
    case ICP_RESULT_ACCEPTED: return "accepted";
    }
    return "unknown";
}

/* `error` is mean squared point distance (m^2); threshold follows that unit. */
static inline icp_result_status_t icp_classify_result(
    size_t match_count, const icp_job_t *job, const float correction[3],
    float maximum_mean_squared_error) {
    if (match_count == 0) return ICP_RESULT_NO_MATCHES;
    if (job == NULL || job->has_nan != 0) return ICP_RESULT_INVALID_SOLVER;
    if (!isfinite(job->error)) return ICP_RESULT_SENTINEL_ERROR;
    if (correction == NULL || !isfinite(correction[0]) ||
        !isfinite(correction[1]) || !isfinite(correction[2]))
        return ICP_RESULT_NONFINITE_CORRECTION;
    if (!isfinite(maximum_mean_squared_error) ||
        maximum_mean_squared_error <= 0.0f || job->error < 0.0f ||
        job->error >= maximum_mean_squared_error)
        return ICP_RESULT_ERROR_REJECTED;
    return ICP_RESULT_ACCEPTED;
}

static inline float point_norm(icp_point_t p) {
    return sqrtf(p.x * p.x + p.y * p.y);
}

static inline void point_normalize(icp_point_t *p) {
    float norm = point_norm(*p);
    p->x /= norm;
    p->y /= norm;
}

static inline float points_dist_squared(icp_point_t a, icp_point_t b) {
    float x = a.x - b.x;
    float y = a.y - b.y;
    return x * x + y * y;
}

void svd(const float in[4], float u[4], float sigma[4], float v[4]);

void icp(const icp_points_t *Q, const icp_points_t *P, int max_iterations,
         float error_bound, float *rotation, icp_job_t *job);

/**
 * Compute nearest-neighbor correspondences on the cluster.
 *
 * @param P Points to match from, with at most SIZE_POINTS entries.
 * @param Q Reference points, with at most SIZE_POINTS entries.
 * @param error_bound Maximum accepted squared distance.
 * @param cor_P Output reference index for each P point, or -1 if unmatched.
 * @param cor_Q Output number of P points matched to each Q point.
 * @return True if the request was valid and correspondences were computed.
 */
bool compute_correspondences_parallel(const icp_points_t *P,
                                      const icp_points_t *Q, float error_bound,
                                      int16_t cor_P[], int16_t cor_Q[]);

#endif // SLAM_FIRMWARE_ICP_H
