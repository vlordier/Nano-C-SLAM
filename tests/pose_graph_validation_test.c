#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "../cslam-gap9-app/slam/graph-based-slam.h"

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
    return 0;
}
