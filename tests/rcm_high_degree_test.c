#include <stdint.h>
#include <stdlib.h>

#include "../cslam-gap9-app/slam/rcm/rcm-sparse-matrix.h"

#define CHECK(condition) \
    do { \
        if (!(condition)) \
            abort(); \
    } while (0)

csr sp_mat1;
float array1[MAX_SIZE];

uint32_t pi_time_get_us(void) {
    return 0;
}
void pi_time_wait_us(uint32_t us) {
    (void)us;
}

int main(void) {
    static csr matrix;
    int16_t permutation[132];
    uint8_t seen[132] = {0};
    init_sp_mat(&matrix);

    for (int16_t leaf = 1; leaf < 132; ++leaf)
        insert_element(&matrix, 132, leaf, 0, 1.0f);

    rcm_fast(&matrix, permutation, 132);
    for (int16_t i = 0; i < 132; ++i) {
        CHECK(permutation[i] >= 0 && permutation[i] < 132);
        CHECK(!seen[permutation[i]]);
        seen[permutation[i]] = 1;
    }
    return 0;
}
