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
