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
    -e GAP9_BUILD_GVSOC_DMA_SMOKE="${GAP9_BUILD_GVSOC_DMA_SMOKE:-0}" \
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
if [[ "${GAP9_BUILD_GVSOC_DMA_SMOKE:-0}" == "1" ]]; then
    DMA_BUILD_ROOT="$BUILD_ROOT/BUILD-gvsoc-dma"
    uv run --no-project --python /usr/bin/python3 "${SDK_PYTHON_ARGS[@]}" \
        cmake -S /project/cslam-gap9-app -B "$DMA_BUILD_ROOT" \
        -DCMAKE_BUILD_TYPE=Release -DNANO_C_SLAM_GVSOC_DMA_SMOKE=ON \
        -DKCONFIG_CONFIG=/project/tests/gap9_gvsoc_sdk.config
    uv run --no-project --python /usr/bin/python3 "${SDK_PYTHON_ARGS[@]}" \
        cmake --build "$DMA_BUILD_ROOT" --target nano_c_slam_gvsoc_dma_smoke \
        "-j$BUILD_JOBS"
    test -s "$DMA_BUILD_ROOT/nano_c_slam_gvsoc_dma_smoke"
    riscv32-unknown-elf-size "$DMA_BUILD_ROOT/nano_c_slam_gvsoc_dma_smoke"
    mkdir -p /project/cslam-gap9-app/BUILD-gvsoc-dma
    cp "$DMA_BUILD_ROOT/nano_c_slam_gvsoc_dma_smoke" \
        /project/cslam-gap9-app/BUILD-gvsoc-dma/nano_c_slam_gvsoc_dma_smoke
    exit 0
fi
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
    -DNANO_C_SLAM_ICP_DIAGNOSTICS=ON \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
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
cp "$BUILD_ROOT/BUILD-gvsoc-mapping/compile_commands.json" \
    /project/cslam-gap9-app/BUILD-gvsoc-mapping/compile_commands.json
MAPPING_ARTIFACTS=/project/cslam-gap9-app/BUILD-gvsoc-mapping
rm -f "$MAPPING_ARTIFACTS/build-provenance.json"
riscv32-unknown-elf-gcc --version | sed -n 1p \
    > "$MAPPING_ARTIFACTS/toolchain-id.txt"
uv run --no-project --python /usr/bin/python3 \
    /project/scripts/write_gap9_build_provenance.py \
    --elf "$MAPPING_ARTIFACTS/nano_c_slam_gvsoc_mapping_smoke" \
    --compile-commands "$MAPPING_ARTIFACTS/compile_commands.json" \
    --toolchain-id "$MAPPING_ARTIFACTS/toolchain-id.txt" \
    --sdk-root /gap_sdk \
    --output "$MAPPING_ARTIFACTS/build-provenance.json"
'

if [[ "${GAP9_BUILD_GVSOC_DMA_SMOKE:-0}" == "1" ]]; then
    echo "GAP9 GVSOC DMA smoke ELF: $ROOT/cslam-gap9-app/BUILD-gvsoc-dma/nano_c_slam_gvsoc_dma_smoke"
else
    echo "GAP9 firmware ELF: $ROOT/.build-gap9-container/BUILD/nano_c_slam_gap9"
fi
