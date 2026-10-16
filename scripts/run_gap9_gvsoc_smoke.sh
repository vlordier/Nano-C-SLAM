#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SDK_ROOT="${GAP9_SDK_ROOT:-$HOME/Work/da3_gap9/third_party/gap_sdk_private}"
TOOLCHAIN_ROOT="${GAP9_TOOLCHAIN_ROOT:-$HOME/Work/da3_gap9/deploy/gap_riscv_toolchain}"
STIMULUS_ROOT="${GAP9_STIMULUS_ROOT:-$HOME/Work/da3_gap9}"
WORK_ROOT="${GAP9_GVSOC_WORK_ROOT:-${TMPDIR:-/tmp}/nano-c-slam-gvsoc}"
BUILD_IMAGE="${GAP9_BUILD_IMAGE:-nano-c-slam-gap9-build:ubuntu22-uv0117}"
GVSOC_LOG="$WORK_ROOT/gvsoc.log"

if [[ "$(uname -s)" != Darwin ]]; then
    echo "This workflow builds with Linux Docker and runs with the local macOS GVSOC runtime." >&2
    exit 2
fi

for required in \
    "$SDK_ROOT/utils/gapy_v2/bin/gapy" \
    "$SDK_ROOT/install/workstation/bin/gvsoc_launcher" \
    "$SDK_ROOT/install/workstation/models/gap9/soc.dylib" \
    "$TOOLCHAIN_ROOT/bin/riscv32-unknown-elf-gcc" \
    "$TOOLCHAIN_ROOT/bin/riscv32-unknown-elf-objcopy"; do
    [[ -e "$required" ]] || { echo "missing GAP9 GVSOC prerequisite: $required" >&2; exit 2; }
done

if [[ "$(file "$SDK_ROOT/install/workstation/bin/gvsoc_launcher")" != *"Mach-O"* ]]; then
    echo "expected a macOS GVSOC launcher in $SDK_ROOT" >&2
    exit 2
fi

mkdir -p "$WORK_ROOT"
if ! docker image inspect "$BUILD_IMAGE" >/dev/null 2>&1; then
    docker build --platform linux/amd64 \
        --file "$ROOT/scripts/Dockerfile.gap9-build" \
        --tag "$BUILD_IMAGE" "$ROOT"
fi

docker run --rm --platform linux/amd64 \
    -v "$ROOT:/project:ro" \
    -v "$SDK_ROOT:/gap_sdk:ro" \
    -v "$TOOLCHAIN_ROOT:/toolchain:ro" \
    -v "$WORK_ROOT:/work" \
    "$BUILD_IMAGE" bash -euo pipefail -c '
cc=riscv32-unknown-elf-gcc
objcopy=riscv32-unknown-elf-objcopy
runtime=/gap_sdk/gvsoc/gvsoc/docs/developer_manual/tutorials/utils
"$cc" -dumpmachine | grep -qx riscv32-unknown-elf
"$cc" -march=rv32imcxgap9 -mabi=ilp32 -O2 -g \
    -fno-tree-loop-distribute-patterns -ffunction-sections -fdata-sections \
    -I"$runtime" -T/project/tests/gap9_gvsoc_link.ld -nostartfiles -nostdlib \
    -Wl,--gc-sections \
    -I/project/tests -I/project/tests/stubs -I/project/cslam-gap9-app/icp \
    -I/project/cslam-gap9-app/slam \
    "$runtime/init.c" "$runtime/crt0.S" "$runtime/alloc.c" "$runtime/io.c" \
    "$runtime/prf.c" "$runtime/string.c" "$runtime/fprintf.c" \
    /project/tests/gap9_gvsoc_smoke.c /project/cslam-gap9-app/icp/icp.c \
    -Wl,--allow-multiple-definition -lm -lgcc \
    -o /work/nano_c_slam_gvsoc_smoke.elf
"$objcopy" -O binary /work/nano_c_slam_gvsoc_smoke.elf /work/nano_c_slam_gvsoc_smoke.bin
'

ELF="$WORK_ROOT/nano_c_slam_gvsoc_smoke.elf"
LAYOUT="$WORK_ROOT/nano_c_slam_gvsoc_layout.json"
if [[ "${NANO_C_SLAM_RUN_GVSOC:-0}" != "1" ]]; then
    printf 'Built GAP9 ICP smoke ELF: %s\n' "$ELF"
    echo "Set NANO_C_SLAM_RUN_GVSOC=1 to execute it with the installed macOS GVSOC runtime."
    exit 0
fi

for stimulus in chip.efuse_preload.data chip.soc.mram.bin; do
    if [[ ! -e "$WORK_ROOT/$stimulus" ]]; then
        [[ -e "$STIMULUS_ROOT/$stimulus" ]] || {
            echo "missing GAP9 GVSOC stimulus: $STIMULUS_ROOT/$stimulus" >&2
            exit 2
        }
        cp "$STIMULUS_ROOT/$stimulus" "$WORK_ROOT/$stimulus"
    fi
done
# The installed private GAP9 model has a fixed XIP work-directory path.
ln -sfn "$WORK_ROOT" /tmp/gvsoc_workdir

set +u
# shellcheck source=/dev/null
source "$SDK_ROOT/configs/gap9_v2.sh"
set -u
export PYTHONPATH="$SDK_ROOT/gvsoc/gvsoc/models:$SDK_ROOT/gvsoc/gvsoc_gap:$SDK_ROOT/install/workstation/python:$SDK_ROOT/utils:${PYTHONPATH:-}"
export GVSOC_MODULES="$SDK_ROOT/gvsoc/gvsoc/models;$SDK_ROOT/gvsoc/gvsoc_gap"
export PATH="$SDK_ROOT/install/workstation/bin:$SDK_ROOT/utils/gapy_v2/bin:$PATH"
export DYLD_LIBRARY_PATH="$SDK_ROOT/install/workstation/lib:${DYLD_LIBRARY_PATH:-}"
export DYLD_FALLBACK_LIBRARY_PATH="$SDK_ROOT/install/workstation/lib"
export GAP9_REAL_GVSOC_LAUNCHER="$SDK_ROOT/install/workstation/bin/gvsoc_launcher"
LAUNCHER_BIN="$WORK_ROOT/launcher-bin"
mkdir -p "$LAUNCHER_BIN"
ln -sf "$ROOT/scripts/gvsoc_tty_launcher.sh" "$LAUNCHER_BIN/gvsoc_launcher"
export PATH="$LAUNCHER_BIN:$PATH"

cd "$WORK_ROOT"
uv run --project "$ROOT" --python 3.12 \
    python "$ROOT/scripts/create_gap9_gvsoc_layout.py" "$ELF" "$LAYOUT"
uv run --no-project --python 3.12 \
    --with pycryptodome --with kconfiglib --with pyelftools \
    --with prettytable --with pexpect --with lz4 --with rich \
    --with psutil --with xxhash --with fdt --with six \
    python "$ROOT/scripts/gapy_private_wrapper.py" \
    "$SDK_ROOT/utils/gapy_v2/bin/gapy" \
    --platform gvsoc --target gap9.gap9_v2 \
    --target-dir "$SDK_ROOT/gvsoc/gvsoc_gap" \
    --target-dir "$SDK_ROOT/gvsoc/gvsoc/models" \
    --model-dir "$SDK_ROOT/install/workstation/models" \
    --target-property boot.flash_device=mram \
    --target-property boot.mode=flash \
    --target-property boot.mode_on_pads=mram \
    --flash-content "$LAYOUT@mram" \
    --work-dir "$WORK_ROOT" image flash run 2>&1 | tee "$GVSOC_LOG"

grep -q 'NANO_C_SLAM_GAP9:icp_ok' "$GVSOC_LOG"
echo "GAP9 GVSOC ICP smoke passed; log: $GVSOC_LOG"
