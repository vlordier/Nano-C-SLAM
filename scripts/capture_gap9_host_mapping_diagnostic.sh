#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
binary="$repo_root/cslam-gap9-app/BUILD-host/gap9_mapping_smoke"
output="${1:-$repo_root/artifacts/gap9_gvsoc_evidence/host_mapping_diagnostic.json}"

"$repo_root/scripts/build_host_mapping_sitl.sh"

compiler_id="$(cc --version | head -n 1)"
linker_gc=-Wl,--gc-sections
if [[ "$(uname -s)" == Darwin ]]; then
    linker_gc=-Wl,-dead_strip
fi
compiler_flags="-std=gnu11 -O2 -Wall -Wextra -Werror -fsanitize=address,undefined -ffunction-sections -fdata-sections -DNANO_C_SLAM_ICP_DIAGNOSTICS=1 $linker_gc"

uv run --locked --project "$repo_root" python -m scripts.capture_gap9_host_mapping_diagnostic \
    --binary "$binary" \
    --output "$output" \
    --toolchain-id "$compiler_id" \
    --compiler-flags "$compiler_flags"
