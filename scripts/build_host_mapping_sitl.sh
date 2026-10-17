#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
output="${NANO_C_SLAM_HOST_SITL:-$repo_root/cslam-gap9-app/BUILD-host/mapping_replay}"
mapping_smoke="$repo_root/cslam-gap9-app/BUILD-host/gap9_mapping_smoke"
mkdir -p "$(dirname "$output")"

if [[ "$(uname -s)" == Darwin ]]; then
    linker_gc=-Wl,-dead_strip
else
    linker_gc=-Wl,--gc-sections
fi

includes=(
    -I"$repo_root/tests/stubs"
    -I"$repo_root/cslam-gap9-app/point-cloud"
    -I"$repo_root/cslam-gap9-app/icp"
    -I"$repo_root/cslam-gap9-app/slam"
    -I"$repo_root/cslam-gap9-app/slam/rcm"
    -I"$repo_root/cslam-gap9-app/dma"
)
core_sources=(
    "$repo_root/cslam-gap9-app/host-sitl/pmsis_host.c"
    "$repo_root/cslam-gap9-app/point-cloud/point_cloud.c"
    "$repo_root/cslam-gap9-app/point-cloud/external_point_cloud.c"
    "$repo_root/cslam-gap9-app/point-cloud/transformations.c"
    "$repo_root/cslam-gap9-app/point-cloud/utils_array.c"
    "$repo_root/cslam-gap9-app/icp/icp.c"
    "$repo_root/cslam-gap9-app/slam/graph-based-slam.c"
    "$repo_root/cslam-gap9-app/slam/globals.c"
    "$repo_root/cslam-gap9-app/slam/sparse-matrix.c"
    "$repo_root/cslam-gap9-app/slam/cholesky.c"
    "$repo_root/cslam-gap9-app/slam/utils_math.c"
    "$repo_root/cslam-gap9-app/slam/rcm/circular-queue.c"
    "$repo_root/cslam-gap9-app/slam/rcm/heapsort.c"
    "$repo_root/cslam-gap9-app/slam/rcm/rcm-sparse-matrix.c"
)
flags=(-std=gnu11 -O2 -Wall -Wextra -Werror "-fsanitize=address,undefined")
flags+=(-ffunction-sections -fdata-sections)

cc "${flags[@]}" "${includes[@]}" \
    "$repo_root/cslam-gap9-app/host-sitl/mapping_replay.c" \
    "${core_sources[@]}" "$linker_gc" -lm -o "$output"
smoke_flags=("${flags[@]}" -DNANO_C_SLAM_ICP_DIAGNOSTICS=1)
cc "${smoke_flags[@]}" "${includes[@]}" \
    "$repo_root/tests/gap9_sdk_gvsoc_mapping_smoke.c" \
    "${core_sources[@]}" "$linker_gc" -lm -o "$mapping_smoke"

printf 'Built host mapping replay: %s\n' "$output"
printf 'Built host GAP9 mapping core smoke: %s\n' "$mapping_smoke"
