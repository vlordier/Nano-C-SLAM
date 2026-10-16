/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "assert.h"
#include "cholesky.h"
#include "config_size.h"
#include "graph-based-slam.h"
#include "pmsis.h"
#include "rcm-sparse-matrix.h"
#include "sparse-matrix.h"
#include "utils_math.h"

extern csr sp_mat0;
extern csr sp_mat1;

PI_L2 float array0[MAX_SIZE];
PI_L2 float array1[MAX_SIZE];
PI_L2 int16_t array2[MAX_SIZE];
PI_L2 int16_t array3[MAX_SIZE];

PI_L2 measurement measurements[MAX_POSES - 1];

static int valid_pose_graph(const float *x, const constraint *constraints,
                            int16_t nr_of_poses, int16_t nr_of_constr) {
    if (x == NULL || nr_of_poses <= 0 || nr_of_poses > MAX_POSES ||
        nr_of_constr < 0 || nr_of_constr > MAX_CONSTRAINTS ||
        (nr_of_constr > 0 && constraints == NULL))
        return 0;

    for (int32_t i = 0; i < 3 * (int32_t)nr_of_poses; ++i) {
        if (!isfinite(x[i])) return 0;
    }
    for (int16_t i = 0; i < nr_of_constr; ++i) {
        const constraint *edge = &constraints[i];
        if (edge->fromIdx < 0 || edge->fromIdx >= nr_of_poses ||
            edge->toIdx < 0 || edge->toIdx >= nr_of_poses ||
            !isfinite(edge->z[0]) || !isfinite(edge->z[1]) ||
            !isfinite(edge->z[2]))
            return 0;
    }
    return 1;
}

static void v2t(float pose[3], float T[3][3]) {
    float c = cosf(pose[2]);
    float s = sinf(pose[2]);
    T[0][0] = c;
    T[0][1] = -s;
    T[0][2] = pose[0];
    T[1][0] = s;
    T[1][1] = c;
    T[1][2] = pose[1];
    T[2][0] = 0;
    T[2][1] = 0;
    T[2][2] = 1;
}

static void t2v(float T[3][3], float v[3]) {
    float x = T[0][2];
    float y = T[1][2];
    float theta = atan2f(T[1][0], T[0][0]);
    v[0] = x;
    v[1] = y;
    v[2] = theta;
}

void linearize_pose_pose_constraint(float *x1, float *x2, float *z,
                                    float A[3][3], float B[3][3], float *e) {
    // Compute e
    float Zij[3][3];
    float Xi[3][3];
    float Xj[3][3];

    v2t(z, Zij);
    v2t(x1, Xi);
    v2t(x2, Xj);

    matinv_3x3(Zij, Zij);
    matinv_3x3(Xi, Xi);
    matmul_3x3(Zij, Xi, Xi);
    matmul_3x3(Xi, Xj, Xi);

    t2v(Xi, e);

    // compute A & B
    float s = sinf(x1[2]);
    float c = cosf(x1[2]);

    float s2 = sinf(z[2]);
    float c2 = cosf(z[2]);

    float RiT[2][2] = {{c, s}, {-s, c}};
    float RijT[2][2] = {{c2, s2}, {-s2, c2}};
    float dRiT_dTheta[2][2] = {{-s, c}, {-c, -s}};
    float delta_t[2] = {x2[0] - x1[0], x2[1] - x1[1]};

    float aux0[2][2];
    float aux1[2];

    matmul_2x2(RijT, dRiT_dTheta, aux0);
    matmul_2x2x1(aux0, delta_t, aux1);
    matmul_2x2(RijT, RiT, aux0);

    A[0][0] = -aux0[0][0];
    A[0][1] = -aux0[0][1];
    A[0][2] = aux1[0];
    A[1][0] = -aux0[1][0];
    A[1][1] = -aux0[1][1];
    A[1][2] = aux1[1];
    A[2][0] = 0;
    A[2][1] = 0;
    A[2][2] = -1;

    B[0][0] = aux0[0][0];
    B[0][1] = aux0[0][1];
    B[0][2] = 0;
    B[1][0] = aux0[1][0];
    B[1][1] = aux0[1][1];
    B[1][2] = 0;
    B[2][0] = 0;
    B[2][1] = 0;
    B[2][2] = 1;
}

void build_H_and_b(float *x, csr *H, float *b, measurement *measurements,
                   constraint *constraints, int16_t nr_of_poses,
                   int16_t nr_of_constr) {
    if (H == NULL || b == NULL || measurements == NULL ||
        !valid_pose_graph(x, constraints, nr_of_poses, nr_of_constr))
        return;

    float z[3];
    float A[3][3], B[3][3], e[3];

    for (int16_t i = 0; i < nr_of_poses - 1; ++i) {
        if (!isfinite(measurements[i].z[0]) ||
            !isfinite(measurements[i].z[1]) || !isfinite(measurements[i].z[2]))
            return;
    }

    int16_t dim = (int16_t)(3 * nr_of_poses);

    insert_element(H, dim, 0, 0, 1000);
    insert_element(H, dim, 1, 1, 1000);
    insert_element(H, dim, 2, 0, 0);
    insert_element(H, dim, 2, 1, 0);
    insert_element(H, dim, 2, 2, 1000);

    for (int16_t i = 1; i < nr_of_poses; i++) {
        float xi[3] = {x[3 * i - 3], x[3 * i - 2], x[3 * i - 1]};
        float xj[3] = {x[3 * i], x[3 * i + 1], x[3 * i + 2]};
        memcpy(z, measurements[i - 1].z, sizeof(z));

        linearize_pose_pose_constraint(xi, xj, z, A, B, e);
        float bi[3], bj[3];
        float Hii[3][3], Hjj[3][3], Hij[3][3];
        transpose_and_mult_3x3x1(A, e, bi);
        transpose_and_mult_3x3x1(B, e, bj);
        transpose_and_mult_3x3(A, A, Hii);
        transpose_and_mult_3x3(A, B, Hij);
        transpose_and_mult_3x3(B, B, Hjj);

        add_to_element(H, dim, 3 * (i - 1) + 0, 3 * (i - 1) + 0,
                       Hii[0][0]); // Hii
        add_to_element(H, dim, 3 * (i - 1) + 1, 3 * (i - 1) + 1,
                       Hii[1][1]); // Hii
        insert_element(H, dim, 3 * (i - 1) + 2, 3 * (i - 1) + 0,
                       Hii[2][0]); // Hii
        insert_element(H, dim, 3 * (i - 1) + 2, 3 * (i - 1) + 1,
                       Hii[2][1]); // Hii
        add_to_element(H, dim, 3 * (i - 1) + 2, 3 * (i - 1) + 2,
                       Hii[2][2]); // Hii

        insert_element(H, dim, 3 * i + 0, 3 * (i - 1) + 0, -1); // Hij

        insert_element(H, dim, 3 * i + 0, 3 * i + 0, Hjj[0][0]); // Hjj

        insert_element(H, dim, 3 * i + 1, 3 * (i - 1) + 1, -1); // Hij

        insert_element(H, dim, 3 * i + 1, 3 * i + 1, Hjj[1][1]); // Hjj

        insert_element(H, dim, 3 * i + 0, 3 * (i - 1) + 2, Hij[2][0]); // Hij
        insert_element(H, dim, 3 * i + 1, 3 * (i - 1) + 2, Hij[2][1]); // Hij
        insert_element(H, dim, 3 * i + 2, 3 * (i - 1) + 2, -1);        // Hij

        // insert_element(H, dim, 3*i + 2, 3*i + 0, 0); // Hjj
        // insert_element(H, dim, 3*i + 2, 3*i + 1, 0); // Hjj
        insert_element(H, dim, 3 * i + 2, 3 * i + 2, Hjj[2][2]); // Hjj

        for (int16_t i0 = 0; i0 < 3; i0++) {
            b[3 * (i - 1) + i0] += -bi[i0];
            b[3 * i + i0] += -bj[i0];
        }
    }

    for (int16_t i = 0; i < nr_of_constr; i++) {
        int fromIdx = constraints[i].fromIdx;
        int toIdx = constraints[i].toIdx;
        float info = 20.0f;
        memcpy(z, constraints[i].z, sizeof(z));

        float xi[3] = {x[3 * fromIdx], x[3 * fromIdx + 1], x[3 * fromIdx + 2]};
        float xj[3] = {x[3 * toIdx], x[3 * toIdx + 1], x[3 * toIdx + 2]};

        linearize_pose_pose_constraint(xi, xj, z, A, B, e);
        float bi[3], bj[3];
        float Hii[3][3], Hjj[3][3], Hij[3][3];
        transpose_and_mult_3x3x1(A, e, bi);
        transpose_and_mult_3x3x1(B, e, bj);
        transpose_and_mult_3x3(A, A, Hii);
        transpose_and_mult_3x3(A, B, Hij);
        transpose_and_mult_3x3(B, B, Hjj);

        for (int16_t i0 = 0; i0 < 3; i0++) {
            add_to_element(H, dim, 3 * fromIdx + i0, 3 * fromIdx + i0,
                           info * Hii[i0][i0]); // Hii
            add_to_element(H, dim, 3 * toIdx + i0, 3 * toIdx + i0,
                           info * Hjj[i0][i0]); // Hjj
            insert_element(H, dim, 3 * fromIdx + i0, 3 * toIdx + i0,
                           info * (-1.0f)); // Hij
        }
        add_to_element(H, dim, 3 * fromIdx + 2, 3 * fromIdx + 0,
                       info * Hii[2][0]); // Hii
        add_to_element(H, dim, 3 * fromIdx + 2, 3 * fromIdx + 1,
                       info * Hii[2][1]); // Hii

        insert_element(H, dim, 3 * fromIdx + 2, 3 * toIdx + 0,
                       info * Hij[2][0]); // Hij
        insert_element(H, dim, 3 * fromIdx + 2, 3 * toIdx + 1,
                       info * Hij[2][1]); // Hij

        for (int16_t i0 = 0; i0 < 3; i0++) {
            b[3 * fromIdx + i0] += -bi[i0] * info;
            b[3 * toIdx + i0] += -bj[i0] * info;
        }
    }
}

bool forward_substitution(csr *L, float *b, int N) {
    if (L == NULL || b == NULL || N <= 0 || N > MAX_SIZE) return false;
    for (int i = 0; i < N; ++i) {
        float diagonal = get_element(L, N, i, i);
        if (!isfinite(diagonal) || diagonal <= 0.0f || !isfinite(b[i]))
            return false;
    }

    float *sol = array1;
    memset(sol, 0, N * sizeof(float));
    sol[0] = b[0] / get_element(L, N, 0, 0);
    if (!isfinite(sol[0])) return false;
    for (int i = 1; i < N; i++) {
        float sum = 0;
        for (int32_t k = L->row_ptr[i]; k < L->row_ptr[i + 1]; k++)
            if (i > L->col_idx[k]) sum += L->data[k] * sol[L->col_idx[k]];
        sol[i] = (1.0f / get_element(L, N, i, i)) * (b[i] - sum);
        if (!isfinite(sol[i])) return false;
    }

    memcpy(b, sol, N * sizeof(float));
    return true;
}

bool backward_substitution(csr *L, float *b, int N) {
    if (L == NULL || b == NULL || N <= 0 || N > MAX_SIZE) return false;
    for (int i = 0; i < N; ++i) {
        float diagonal = get_element(L, N, i, i);
        if (!isfinite(diagonal) || diagonal <= 0.0f || !isfinite(b[i]))
            return false;
    }

    float *sol = array1;
    memset(sol, 0, N * sizeof(float));
    sol[N - 1] = b[N - 1] / get_element(L, N, N - 1, N - 1);
    if (!isfinite(sol[N - 1])) return false;
    for (int i = N - 2; i >= 0; i--) {
        float sum = 0;
        for (int32_t k = L->row_ptr[i]; k < L->row_ptr[i + 1]; k++)
            if (i < L->col_idx[k]) sum += L->data[k] * sol[L->col_idx[k]];
        sol[i] = (1.0f / get_element(L, N, i, i)) * (b[i] - sum);
        if (!isfinite(sol[i])) return false;
    }

    memcpy(b, sol, N * sizeof(float));
    return true;
}

bool solve_for_x(csr *H, float *b, int16_t *perm, int16_t size) {
    if (H == NULL || b == NULL || perm == NULL || size <= 0 || size > MAX_SIZE)
        return false;
    for (int16_t i = 0; i < size; ++i) {
        if (perm[i] < 0 || perm[i] >= size || !isfinite(b[i])) return false;
        for (int16_t j = 0; j < i; ++j) {
            if (perm[i] == perm[j]) return false;
        }
    }

    if (H->nz_cnt < -1 || H->nz_cnt >= MAX_SIZE_SP_MAT) {
        printf("Input SpMat element overflow\n");
        return false;
    }

    // Cholesky Decomposition
    cholesky_sparse_parallel(H, size, perm);
    // cholesky_sparse_fc_only(H, size, perm);

    if (H->nz_cnt < -1 || H->nz_cnt >= MAX_SIZE_SP_MAT) {
        printf("SpMat element overflow\n");
        return false;
    }

    for (int16_t i = 0; i < size; ++i) {
        float diagonal = get_element(H, size, i, i);
        if (!isfinite(diagonal) || diagonal <= 0.0f) return false;
    }

    // Permute array
    permute_array(b, perm, size);

    /* L * L.T * x = b;   we note L.T * x = y
       Solves L * y = b; */
    if (!forward_substitution(H, b, size)) return false;
    float *y = b;

    // Solves L.T * x = y
    transpose_lower_triangular_in_place(H, size);
    if (!backward_substitution(H, y, size)) return false;
    float *x = y;

    // Inverse permute the solution
    int16_t *permInv = array3;
    permutation_inverse(perm, permInv, size);
    permute_array(x, permInv, size);
    for (int16_t i = 0; i < size; ++i) {
        if (!isfinite(x[i])) return false;
    }
    return true;
}

void measurement_from_poses(float *x_from, float *x_to, float *z) {
    for (int16_t k = 0; k < 3; k++) z[k] = x_to[k] - x_from[k];

    float s = sinf(-x_from[2]);
    float c = cosf(-x_from[2]);
    float R[2][2] = {{c, -s}, {s, c}};
    matmul_2x2x1(R, z, z);
}

void measurements_from_poses(float *x, measurement *measurements,
                             int nr_of_poses) {
    if (x == NULL || measurements == NULL || nr_of_poses < 2 ||
        nr_of_poses > MAX_POSES)
        return;
    for (int32_t i = 0; i < 3 * nr_of_poses; ++i) {
        if (!isfinite(x[i])) return;
    }

    float z[3];
    for (int16_t i = 1; i < nr_of_poses; i++) {
        float xi[3] = {x[3 * i - 3], x[3 * i - 2], x[3 * i - 1]};
        float xj[3] = {x[3 * i], x[3 * i + 1], x[3 * i + 2]};
        measurement_from_poses(xi, xj, z);

        measurements[i - 1].z[0] = z[0];
        measurements[i - 1].z[1] = z[1];
        measurements[i - 1].z[2] = z[2];
    }
}

int16_t abs_int(int16_t x) {
    if (x < 0)
        return x * (-1);
    else
        return x;
}

int16_t bandwidth_test(csr *mat, int N, int16_t *perm) {
    int16_t perm_inv[N];
    for (int16_t i = 0; i < N; i++)
        for (int16_t j = 0; j < N; j++)
            if (perm[j] == i) {
                perm_inv[i] = j;
                break;
            }

    // Bandwidth test
    int16_t bw_len = 0;
    for (int16_t i = 0; i < N; i++) {
        for (int32_t k = mat->row_ptr[i]; k < mat->row_ptr[i + 1]; k++) {
            int j = mat->col_idx[k];
            int indx = perm_inv[i];
            int indy = perm_inv[j];
            if (abs_int(indx - indy) > bw_len) {
                bw_len = abs_int(indx - indy);
            }
        }
    }
    return bw_len;
}

bool ls_slam(float *x, constraint *constraints, int16_t nr_of_poses,
             int16_t nr_of_constr) {
    if (!valid_pose_graph(x, constraints, nr_of_poses, nr_of_constr))
        return false;

    csr *H = &sp_mat0;
    memset(array0, 0, MAX_SIZE * sizeof(float));
    memset(array1, 0, MAX_SIZE * sizeof(float));

    float *b = array0;
    int16_t *perm = array2;

    // Compute the graph edge measurements
    measurements_from_poses(x, measurements, nr_of_poses);
    for (int16_t k = 0; k < 3; k++) {
        // Empty the sparse matrix
        init_sp_mat(H);
        memset(b, 0, 3 * nr_of_poses * sizeof(float));

        // Compute the input of the linear equation system
        build_H_and_b(x, H, b, measurements, constraints, nr_of_poses,
                      nr_of_constr);

        // Get the permutation that minimizes the bandwidth of H
        rcm_fast(H, perm, 3 * nr_of_poses);

        // Solve the linear equation system
        if (!solve_for_x(H, b, perm, (int16_t)(3 * nr_of_poses))) return false;

        // Update the poses
        for (int16_t i = 0; i < 3 * nr_of_poses; i++) {
            if (!isfinite(x[i] + b[i])) return false;
        }
        for (int16_t i = 0; i < 3 * nr_of_poses; i++) x[i] += b[i];
    }
    return true;
}
