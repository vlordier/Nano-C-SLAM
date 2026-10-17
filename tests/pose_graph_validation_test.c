#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "../cslam-gap9-app/slam/cholesky.h"
#include "../cslam-gap9-app/slam/graph-based-slam.h"
#include "../cslam-gap9-app/slam/rcm/rcm-sparse-matrix.h"
#include "../cslam-gap9-app/slam/utils_math.h"

void cholesky_sparse_parallel(csr *matrix, int16_t size, int16_t *permutation) {
    (void)matrix;
    (void)size;
    (void)permutation;
    abort();
}

#define CHECK(condition) \
    do { \
        if (!(condition)) \
            abort(); \
    } while (0)

uint32_t pi_time_get_us(void) {
    return 0;
}
void pi_time_wait_us(uint32_t us) {
    (void)us;
}

int main(void) {
    float invalid_graph_poses[6] = {0.0f};
    constraint invalid_graph_edge = {.fromIdx = 0, .toIdx = 2, .z = {1.0f, 0.0f, 0.0f}};
    int16_t completed_iterations = -1;
    CHECK(!ls_slam_with_iterations(invalid_graph_poses, &invalid_graph_edge, 2, 1,
                                   &completed_iterations));
    CHECK(completed_iterations == 0);
    invalid_graph_edge.toIdx = 1;
    invalid_graph_edge.z[0] = NAN;
    completed_iterations = -1;
    CHECK(!ls_slam_with_iterations(invalid_graph_poses, &invalid_graph_edge, 2, 1,
                                   &completed_iterations));
    CHECK(completed_iterations == 0);

    float singular[3][3] = {{1, 2, 3}, {2, 4, 6}, {0, 0, 0}};
    float inverse[3][3] = {{7, 7, 7}, {7, 7, 7}, {7, 7, 7}};
    CHECK(!matinv_3x3(singular, inverse));
    CHECK(inverse[0][0] == 7.0f && inverse[2][2] == 7.0f);
    float rigid[3][3] = {{0, -1, 2}, {1, 0, 3}, {0, 0, 1}};
    CHECK(matinv_3x3(rigid, rigid));
    CHECK(fabsf(rigid[0][2] + 3.0f) < 1.0e-6f);
    CHECK(fabsf(rigid[1][2] - 2.0f) < 1.0e-6f);

    float poses[6] = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f};
    measurement odometry[1] = {{{1.0f, 0.0f, 0.0f}}};
    constraint edge = {.fromIdx = 0, .toIdx = 2, .z = {1.0f, 0.0f, 0.0f}};
    csr matrix = {0};
    float rhs[3 * MAX_POSES];
    for (size_t i = 0; i < 3 * MAX_POSES; ++i)
        rhs[i] = 17.0f;

    build_H_and_b(poses, &matrix, rhs, odometry, &edge, 2, 1);
    CHECK(matrix.nz_cnt == 0);
    CHECK(rhs[0] == 17.0f && rhs[1] == 17.0f && rhs[2] == 17.0f);

    build_H_and_b(poses, &matrix, rhs, odometry, NULL, 2, 1);
    CHECK(matrix.nz_cnt == 0);
    build_H_and_b(poses, &matrix, rhs, odometry, &edge, 2, MAX_CONSTRAINTS + 1);
    CHECK(matrix.nz_cnt == 0);

    edge.toIdx = 1;
    edge.z[1] = NAN;
    build_H_and_b(poses, &matrix, rhs, odometry, &edge, 2, 1);
    CHECK(matrix.nz_cnt == 0);
    CHECK(rhs[0] == 17.0f && rhs[1] == 17.0f && rhs[2] == 17.0f);

    edge.z[1] = 0.0f;
    build_H_and_b(poses, &matrix, rhs, odometry, &edge, MAX_POSES + 1, 1);
    CHECK(matrix.nz_cnt == 0);
    CHECK(rhs[0] == 17.0f && rhs[1] == 17.0f && rhs[2] == 17.0f);

    csr invalid_factor = {0};
    float solution[1] = {4.0f};
    CHECK(!forward_substitution(&invalid_factor, solution, 1));
    CHECK(solution[0] == 4.0f);
    CHECK(!backward_substitution(&invalid_factor, solution, 1));
    CHECK(solution[0] == 4.0f);

    float rhs_one[2] = {3.0f, 5.0f};
    int16_t duplicate_permutation[2] = {0, 0};
    CHECK(!solve_for_x(&invalid_factor, rhs_one, duplicate_permutation, 2));
    CHECK(rhs_one[0] == 3.0f && rhs_one[1] == 5.0f);
    return 0;
}
