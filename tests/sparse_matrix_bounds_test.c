#include <math.h>
#include <stdlib.h>

#include "../cslam-gap9-app/slam/sparse-matrix.h"

#define CHECK(condition) \
    do { \
        if (!(condition)) \
            abort(); \
    } while (0)

csr sp_mat1;

int main(void) {
    static csr matrix;
    init_sp_mat(&matrix);
    CHECK(get_element(&matrix, MAX_SIZE, -1, 0) == 0.0f);
    CHECK(get_element(&matrix, MAX_SIZE + 1, 0, 0) == 0.0f);
    CHECK(!already_added(&matrix, MAX_SIZE, MAX_SIZE, 0));
    insert_element(&matrix, MAX_SIZE, -1, 0, 1.0f);
    add_to_element(&matrix, MAX_SIZE, 0, MAX_SIZE, 1.0f);
    CHECK(matrix.nz_cnt == 0);

    for (int16_t row = 0; row < MAX_SIZE; ++row)
        for (int16_t col = 0; col < 9; ++col)
            insert_element(&matrix, MAX_SIZE, row, col, 1.0f);
    CHECK(matrix.nz_cnt == MAX_SIZE_SP_MAT - 1);
    CHECK(get_element(&matrix, MAX_SIZE, MAX_SIZE - 1, 8) == 1.0f);

    insert_element(&matrix, MAX_SIZE, MAX_SIZE - 1, 9, 2.0f);
    CHECK(matrix.nz_cnt == MAX_SIZE_SP_MAT - 1);
    CHECK(get_element(&matrix, MAX_SIZE, MAX_SIZE - 1, 9) == 0.0f);
    return 0;
}
