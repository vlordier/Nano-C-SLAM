#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
output="${NANO_C_SLAM_HOST_SITL:-$repo_root/cslam-gap9-app/BUILD-host/mapping_replay}"
mkdir -p "$(dirname "$output")"

if [[ "$(uname -s)" == Darwin ]]; then
    linker_gc=-Wl,-dead_strip
else
    linker_gc=-Wl,--gc-sections
fi

cc -std=gnu11 -O2 -Wall -Wextra -Werror -fsanitize=address,undefined \
    -ffunction-sections -fdata-sections \
    -I"$repo_root/tests/stubs" \
    -I"$repo_root/cslam-gap9-app/point-cloud" \
    -I"$repo_root/cslam-gap9-app/icp" \
    -I"$repo_root/cslam-gap9-app/slam" \
    -I"$repo_root/cslam-gap9-app/slam/rcm" \
    -I"$repo_root/cslam-gap9-app/dma" \
    "$repo_root/cslam-gap9-app/host-sitl/mapping_replay.c" \
    "$repo_root/cslam-gap9-app/host-sitl/pmsis_host.c" \
    "$repo_root/cslam-gap9-app/point-cloud/point_cloud.c" \
    "$repo_root/cslam-gap9-app/point-cloud/external_point_cloud.c" \
    "$repo_root/cslam-gap9-app/point-cloud/transformations.c" \
    "$repo_root/cslam-gap9-app/point-cloud/utils_array.c" \
    "$repo_root/cslam-gap9-app/icp/icp.c" \
    "$repo_root/cslam-gap9-app/slam/graph-based-slam.c" \
    "$repo_root/cslam-gap9-app/slam/globals.c" \
    "$repo_root/cslam-gap9-app/slam/sparse-matrix.c" \
    "$repo_root/cslam-gap9-app/slam/cholesky.c" \
    "$repo_root/cslam-gap9-app/slam/utils_math.c" \
    "$repo_root/cslam-gap9-app/slam/rcm/circular-queue.c" \
    "$repo_root/cslam-gap9-app/slam/rcm/heapsort.c" \
    "$repo_root/cslam-gap9-app/slam/rcm/rcm-sparse-matrix.c" \
    "$linker_gc" -lm -o "$output"

printf 'Built host mapping replay: %s\n' "$output"
