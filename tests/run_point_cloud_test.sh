#!/usr/bin/env sh
set -eu

if [ "$(uname -s)" = Darwin ]; then
    linker_gc=-Wl,-dead_strip
else
    linker_gc=-Wl,--gc-sections
fi

cc -std=gnu11 -Wall -Wextra -Werror -fsanitize=address,undefined \
    -ffunction-sections -fdata-sections \
    -Itests/stubs -Icslam-gap9-app/point-cloud -Icslam-gap9-app/icp \
    -Icslam-gap9-app/slam -Icslam-gap9-app/utils \
    tests/point_cloud_test.c cslam-gap9-app/point-cloud/point_cloud.c \
    "$linker_gc" -lm -o /tmp/point_cloud_test
/tmp/point_cloud_test

cc -std=gnu11 -Wall -Wextra -Werror -fsanitize=address,undefined \
    -ffunction-sections -fdata-sections \
    -Itests/stubs -Icslam-gap9-app/point-cloud -Icslam-gap9-app/icp \
    -Icslam-gap9-app/slam -Icslam-gap9-app/utils \
    tests/external_point_cloud_test.c \
    cslam-gap9-app/point-cloud/external_point_cloud.c \
    "$linker_gc" -lm -o /tmp/external_point_cloud_test
/tmp/external_point_cloud_test

cc -std=gnu11 -Wall -Wextra -Werror -fsanitize=address,undefined \
    -ffunction-sections -fdata-sections \
    -Itests/stubs -Icslam-gap9-app/icp -Icslam-gap9-app/slam \
    tests/icp_test.c cslam-gap9-app/icp/icp.c \
    "$linker_gc" -lm -o /tmp/icp_test
/tmp/icp_test

cc -std=gnu11 -Wall -Wextra -Werror -fsanitize=address,undefined \
    -fno-omit-frame-pointer \
    -Itests/stubs -Icslam-gap9-app/spi -Icslam-gap9-app/point-cloud \
    -Icslam-gap9-app/icp -Icslam-gap9-app/slam \
    tests/gap9_spi_decode_test.c cslam-gap9-app/spi/spi_cmds_decode.c \
    -lm -o /tmp/gap9_spi_decode_test
/tmp/gap9_spi_decode_test

cc -std=gnu11 -Wall -Wextra -Werror -fsanitize=address,undefined \
    -ffunction-sections -fdata-sections \
    -Itests/stubs -Icslam-gap9-app/slam -Icslam-gap9-app/slam/rcm \
    tests/pose_graph_validation_test.c \
    cslam-gap9-app/slam/graph-based-slam.c \
    cslam-gap9-app/slam/utils_math.c \
    "$linker_gc" -lm -o /tmp/pose_graph_validation_test
/tmp/pose_graph_validation_test

cc -std=gnu11 -Wall -Wextra -Werror -fsanitize=address,undefined \
    -Itests/stubs -Icslam-gap9-app/slam \
    tests/sparse_matrix_bounds_test.c cslam-gap9-app/slam/sparse-matrix.c \
    "$linker_gc" -lm -o /tmp/sparse_matrix_bounds_test
/tmp/sparse_matrix_bounds_test
