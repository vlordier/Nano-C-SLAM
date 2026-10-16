#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "../cslam-gap9-app/slam/cholesky.h"
#include "../cslam-gap9-app/slam/graph-based-slam.h"
#include "../cslam-gap9-app/slam/rcm/rcm-sparse-matrix.h"

void insert_element(csr *mat, int16_t n, int16_t row, int16_t col, float value) {
    (void)mat;
    (void)n;
    (void)row;
    (void)col;
    (void)value;
    abort();
}

void add_to_element(csr *mat, int16_t n, int16_t row, int16_t col, float value) {
    (void)mat;
    (void)n;
    (void)row;
    (void)col;
    (void)value;
    abort();
}

float get_element(csr *mat, int16_t n, int16_t row, int16_t col) {
    (void)n;
    for (int32_t i = mat->row_ptr[row]; i < mat->row_ptr[row + 1]; ++i) {
        if (mat->col_idx[i] == col)
            return mat->data[i];
    }
    return 0.0f;
}

void cholesky_sparse_parallel(csr *mat, int16_t n, int16_t *perm) {
    (void)mat;
    (void)n;
    (void)perm;
    abort();
}

void permute_array(float *array, int16_t *perm, int16_t n) {
    (void)array;
    (void)perm;
    (void)n;
    abort();
}

void permutation_inverse(int16_t *perm, int16_t *inverse, int n) {
    (void)perm;
    (void)inverse;
    (void)n;
    abort();
}

void transpose_lower_triangular_in_place(csr *mat, int16_t n) {
    (void)mat;
    (void)n;
    abort();
}

#define CHECK(condition) \
    do { \
        if (!(condition)) \
            abort(); \
    } while (0)

int main(void) {
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
