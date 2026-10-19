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
    -Itests/stubs/gap9_spi -Icslam-gap9-app/spi \
    tests/gap9_spi_driver_test.c cslam-gap9-app/spi/spi_driver.c \
    -o /tmp/gap9_spi_driver_test
/tmp/gap9_spi_driver_test

cc -std=gnu11 -Wall -Wextra -Werror -fsanitize=address,undefined \
    -Itests/stubs/gap9_spi -Icslam-gap9-app/spi \
    tests/gap9_spi_buffers_test.c cslam-gap9-app/spi/spi_buffers.c \
    -o /tmp/gap9_spi_buffers_test
/tmp/gap9_spi_buffers_test

cc -std=gnu11 -Wall -Wextra -Werror -fsanitize=address,undefined \
    -Icslam-gap9-app/spi tests/gap9_spi_diagnostics_test.c \
    cslam-gap9-app/spi/spi_diagnostics.c -o /tmp/gap9_spi_diagnostics_test
/tmp/gap9_spi_diagnostics_test

cc -std=gnu11 -Wall -Wextra -Werror -fsanitize=address,undefined \
    -Itests/stubs/flight_control -Icslam-stm32-app/src \
    -Icslam-stm32-app/src/exploration \
    tests/stm32_flight_control_test.c cslam-stm32-app/src/flight_commander.c \
    cslam-stm32-app/src/exploration/limit_actuation.c \
    -lm -o /tmp/stm32_flight_control_test
for scenario in abort-before-takeoff ready-gate stale-before-takeoff \
    stale-during-takeoff stale-active takeoff-failure land-handoff land-retry; do
    /tmp/stm32_flight_control_test "$scenario"
done

cc -std=gnu11 -Wall -Wextra -Werror -fsanitize=address,undefined \
    -Itests/stubs/flight_control -Icslam-stm32-app/src \
    tests/tof_freshness_test.c cslam-stm32-app/src/tof_freshness.c \
    -o /tmp/tof_freshness_test
/tmp/tof_freshness_test

cc -std=gnu11 -Wall -Wextra -Werror -fsanitize=address,undefined \
    -Itests/stubs/flight_control -Icslam-stm32-app/src \
    tests/mission_timeout_test.c cslam-stm32-app/src/mission_timeout.c \
    -o /tmp/mission_timeout_test
/tmp/mission_timeout_test

cc -std=gnu11 -Wall -Wextra -Werror -fsanitize=address,undefined \
    -ffunction-sections -fdata-sections \
    -Itests/stubs -Icslam-gap9-app/slam -Icslam-gap9-app/slam/rcm \
    tests/pose_graph_validation_test.c \
    cslam-gap9-app/slam/graph-based-slam.c \
    cslam-gap9-app/slam/utils_math.c \
    cslam-gap9-app/slam/sparse-matrix.c \
    cslam-gap9-app/slam/rcm/rcm-sparse-matrix.c \
    cslam-gap9-app/slam/rcm/circular-queue.c \
    cslam-gap9-app/slam/rcm/heapsort.c \
    cslam-gap9-app/slam/globals.c \
    "$linker_gc" -lm -o /tmp/pose_graph_validation_test
/tmp/pose_graph_validation_test

cc -std=gnu11 -Wall -Wextra -Werror -fsanitize=address,undefined \
    -Itests/stubs -Icslam-gap9-app/slam \
    tests/sparse_matrix_bounds_test.c cslam-gap9-app/slam/sparse-matrix.c \
    "$linker_gc" -lm -o /tmp/sparse_matrix_bounds_test
/tmp/sparse_matrix_bounds_test

cc -std=gnu11 -Wall -Wextra -Werror -fsanitize=address,undefined \
    -Itests/stubs -Icslam-gap9-app/slam -Icslam-gap9-app/slam/rcm \
    tests/circular_queue_test.c cslam-gap9-app/slam/rcm/circular-queue.c \
    "$linker_gc" -o /tmp/circular_queue_test
/tmp/circular_queue_test

cc -std=gnu11 -Wall -Wextra -Werror -fsanitize=address,undefined \
    -Itests/stubs -Icslam-gap9-app/slam -Icslam-gap9-app/slam/rcm \
    tests/rcm_high_degree_test.c cslam-gap9-app/slam/rcm/rcm-sparse-matrix.c \
    cslam-gap9-app/slam/rcm/circular-queue.c \
    cslam-gap9-app/slam/rcm/heapsort.c cslam-gap9-app/slam/sparse-matrix.c \
    "$linker_gc" -lm -o /tmp/rcm_high_degree_test
/tmp/rcm_high_degree_test
