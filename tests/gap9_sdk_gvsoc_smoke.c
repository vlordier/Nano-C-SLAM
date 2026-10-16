#include <math.h>
#include <stdio.h>

#include "icp.h"

int main(void) {
    const float covariance[4] = {3.0f, 0.0f, 0.0f, 2.0f};
    float u[4], sigma[4], v[4];
    svd(covariance, u, sigma, v);

    if (!isfinite(sigma[0]) || !isfinite(sigma[3]) || fabsf(sigma[0] - 3.0f) > 0.001f ||
        fabsf(sigma[3] - 2.0f) > 0.001f) {
        printf("NANO_C_SLAM_GVSOC:icp_svd_fail\n");
        pmsis_exit(-1);
        return -1;
    }

    printf("NANO_C_SLAM_GVSOC:icp_svd_ok sigma0_milli=%d sigma1_milli=%d\n",
           (int)(sigma[0] * 1000.0f), (int)(sigma[3] * 1000.0f));
    pmsis_exit(0);
    return 0;
}
