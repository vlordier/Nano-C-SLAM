#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SDK_ROOT="${GAP9_SDK_ROOT:-$HOME/Work/da3_gap9/third_party/gap_sdk_private}"
TOOLCHAIN_ROOT="${GAP9_TOOLCHAIN_ROOT:-$HOME/Work/da3_gap9/deploy/gap_riscv_toolchain}"
BUILD_IMAGE="${GAP9_BUILD_IMAGE:-nano-c-slam-gap9-build:ubuntu22-uv0117}"

for required in \
    "$SDK_ROOT/utils/cmake/setup.cmake" \
    "$TOOLCHAIN_ROOT/bin/riscv32-unknown-elf-gcc"; do
    [[ -e "$required" ]] || { echo "missing GAP9 build prerequisite: $required" >&2; exit 2; }
done

if ! docker image inspect "$BUILD_IMAGE" >/dev/null 2>&1; then
    docker build --platform linux/amd64 \
        --file "$ROOT/scripts/Dockerfile.gap9-build" \
        --tag "$BUILD_IMAGE" "$ROOT"
fi

docker run --rm --platform linux/amd64 \
    -v "$ROOT:/project" \
    -v "$SDK_ROOT:/gap_sdk:ro" \
    -v "$TOOLCHAIN_ROOT:/toolchain:ro" \
    "$BUILD_IMAGE" bash -euo pipefail -c '
export PATH=/toolchain/bin:$PATH
export GAP_SDK_HOME=/gap_sdk
export GAP_RISCV_GCC_TOOLCHAIN=/toolchain
git -C /gap_sdk describe --abbrev=0 --tags --match "release*" >/dev/null
set +u
source /gap_sdk/configs/gap9_v2.sh
set -u

SDK_PYTHON_ARGS=(
    --with "pyelftools>=0.24"
    --with "prettytable==0.7.2"
    --with "pexpect>=4,<5"
    --with six
    --with kconfiglib
    --with fdt
    --with pyserial
    --with pycryptodome
    --with requests
    --with "protobuf>=3.20.2,<4"
    --with tabulate
    --with prometheus-client
    --with tqdm
    --with pyyaml
    --with argcomplete
    --with configparser
    --with lz4
    --with rich
    --with psutil
    --with xxhash
)
uv run --no-project --python /usr/bin/python3 "${SDK_PYTHON_ARGS[@]}" \
    cmake -S /project/cslam-gap9-app -B /project/cslam-gap9-app/BUILD \
    -DCMAKE_BUILD_TYPE=Release
uv run --no-project --python /usr/bin/python3 "${SDK_PYTHON_ARGS[@]}" \
    cmake --build /project/cslam-gap9-app/BUILD --target nano_c_slam_gap9 -j1
test -s /project/cslam-gap9-app/BUILD/nano_c_slam_gap9
riscv32-unknown-elf-size /project/cslam-gap9-app/BUILD/nano_c_slam_gap9
'

echo "GAP9 firmware ELF: $ROOT/cslam-gap9-app/BUILD/nano_c_slam_gap9"
