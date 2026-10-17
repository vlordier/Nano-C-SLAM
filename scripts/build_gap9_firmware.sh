#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SDK_ROOT="${GAP9_SDK_ROOT:-$HOME/Work/da3_gap9/third_party/gap_sdk_private}"
TOOLCHAIN_ROOT="${GAP9_TOOLCHAIN_ROOT:-$HOME/Work/da3_gap9/deploy/gap_riscv_toolchain}"
BUILD_IMAGE="${GAP9_BUILD_IMAGE:-nano-c-slam-gap9-build:ubuntu22-uv0117}"
BUILD_DNS="${GAP9_BUILD_DNS:-1.1.1.1}"

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

docker run --rm --platform linux/amd64 --dns "$BUILD_DNS" \
    -v "$ROOT:/project" \
    -v "$SDK_ROOT:/gap_sdk:ro" \
    -v "$TOOLCHAIN_ROOT:/toolchain:ro" \
    --mount type=volume,source=nano-c-slam-gap9-uv-cache,target=/root/.cache/uv \
    "$BUILD_IMAGE" bash -euo pipefail -c '
export PATH=/toolchain/bin:$PATH
export GAP_SDK_HOME=/gap_sdk
export GAP_RISCV_GCC_TOOLCHAIN=/toolchain
BUILD_ROOT=/project/.build-gap9-container
BUILD_JOBS="${GAP9_BUILD_JOBS:-4}"
mkdir -p "$BUILD_ROOT"
git config --global --add safe.directory /gap_sdk
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
    cmake -S /project/cslam-gap9-app -B "$BUILD_ROOT/BUILD" \
    -DCMAKE_BUILD_TYPE=Release
uv run --no-project --python /usr/bin/python3 "${SDK_PYTHON_ARGS[@]}" \
    cmake --build "$BUILD_ROOT/BUILD" --target nano_c_slam_gap9 "-j$BUILD_JOBS"
test -s "$BUILD_ROOT/BUILD/nano_c_slam_gap9"
riscv32-unknown-elf-size "$BUILD_ROOT/BUILD/nano_c_slam_gap9"
uv run --no-project --python /usr/bin/python3 "${SDK_PYTHON_ARGS[@]}" \
    cmake -S /project/cslam-gap9-app -B "$BUILD_ROOT/BUILD-gvsoc" \
    -DCMAKE_BUILD_TYPE=Release -DNANO_C_SLAM_GVSOC_SMOKE=ON \
    -DKCONFIG_CONFIG=/project/tests/gap9_gvsoc_sdk.config
uv run --no-project --python /usr/bin/python3 "${SDK_PYTHON_ARGS[@]}" \
    cmake --build "$BUILD_ROOT/BUILD-gvsoc" --target nano_c_slam_gvsoc_smoke "-j$BUILD_JOBS"
test -s "$BUILD_ROOT/BUILD-gvsoc/nano_c_slam_gvsoc_smoke"
riscv32-unknown-elf-size "$BUILD_ROOT/BUILD-gvsoc/nano_c_slam_gvsoc_smoke"
uv run --no-project --python /usr/bin/python3 "${SDK_PYTHON_ARGS[@]}" \
    cmake -S /project/cslam-gap9-app -B "$BUILD_ROOT/BUILD-gvsoc-mapping" \
    -DCMAKE_BUILD_TYPE=Release -DNANO_C_SLAM_GVSOC_MAPPING_SMOKE=ON \
    -DKCONFIG_CONFIG=/project/tests/gap9_gvsoc_sdk.config
uv run --no-project --python /usr/bin/python3 "${SDK_PYTHON_ARGS[@]}" \
    cmake --build "$BUILD_ROOT/BUILD-gvsoc-mapping" \
    --target nano_c_slam_gvsoc_mapping_smoke "-j$BUILD_JOBS"
test -s "$BUILD_ROOT/BUILD-gvsoc-mapping/nano_c_slam_gvsoc_mapping_smoke"
riscv32-unknown-elf-size \
    "$BUILD_ROOT/BUILD-gvsoc-mapping/nano_c_slam_gvsoc_mapping_smoke"
mkdir -p /project/cslam-gap9-app/BUILD-gvsoc-mapping
cp "$BUILD_ROOT/BUILD-gvsoc-mapping/nano_c_slam_gvsoc_mapping_smoke" \
    /project/cslam-gap9-app/BUILD-gvsoc-mapping/nano_c_slam_gvsoc_mapping_smoke
'

echo "GAP9 firmware ELF: $ROOT/.build-gap9-container/BUILD/nano_c_slam_gap9"
