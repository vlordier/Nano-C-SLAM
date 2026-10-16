#!/usr/bin/env bash
# The GAP SDK is an external path and is deliberately supplied by the caller.
# shellcheck source=/dev/null
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SDK_ROOT="${GAP9_PRIVATE_SDK_ROOT:-$HOME/Work/da3_gap9/third_party/gap_sdk_private}"
TOOLCHAIN_ROOT="${GAP9_TOOLCHAIN_ROOT:-$HOME/Work/da3_gap9/deploy/gap_riscv_toolchain}"
WORK_ROOT="${GAP9_GVSOC_WORK_ROOT:-${TMPDIR:-/tmp}/nano-c-slam-gvsoc}"

if [[ "$(uname -s)" != Darwin ]]; then
    echo "This local smoke path expects the macOS GVSOC launcher; use a Linux GVSOC SDK in a Linux container." >&2
    exit 2
fi

if [[ "$(file "$SDK_ROOT/install/workstation/bin/gvsoc_launcher")" != *"Mach-O"* ]]; then
    echo "expected a macOS GVSOC launcher in $SDK_ROOT" >&2
    exit 2
fi

for required in \
    "$SDK_ROOT/utils/gapy_v2/bin/gapy" \
    "$SDK_ROOT/install/workstation/models/gap9/soc.dylib" \
    "$SDK_ROOT/install/workstation/lib/libpulpvp.dylib" \
    "$TOOLCHAIN_ROOT/bin/riscv32-unknown-elf-gcc" \
    "$TOOLCHAIN_ROOT/bin/riscv32-unknown-elf-objcopy"; do
    [[ -e "$required" ]] || { echo "missing GAP9 GVSOC prerequisite: $required" >&2; exit 2; }
done

mkdir -p "$WORK_ROOT"
docker run --rm --platform linux/amd64 \
    -v "$ROOT:/project:ro" -v "$SDK_ROOT:/gap_sdk:ro" \
    -v "$TOOLCHAIN_ROOT:/toolchain:ro" -v "$WORK_ROOT:/work" \
    -w /work ubuntu:22.04 bash -lc '
set -euo pipefail
apt-get update -qq
apt-get install -y -qq python3 python3-pip python3-kconfiglib python3-pyelftools python3-prettytable python3-pexpect python3-lz4 python3-rich python3-psutil python3-six libgmp-dev libmpfr-dev libmpc-dev libncurses5 libfdt-dev device-tree-compiler
export PATH=/toolchain/bin:/gap_sdk/utils/bin:$PATH
export GAP_SDK_HOME=/gap_sdk
export GAP_RISCV_GCC_TOOLCHAIN=/toolchain
export PYTHONPATH=/gap_sdk/utils/gapy_v2/bin:/gap_sdk/utils:/gap_sdk/install/workstation/python:/gap_sdk/gvsoc/gvsoc/models:/gap_sdk/gvsoc/gvsoc_gap:${PYTHONPATH:-}
cc=riscv32-unknown-elf-gcc
objcopy=riscv32-unknown-elf-objcopy
"$cc" -dumpmachine | grep -qx riscv32-unknown-elf
"$cc" -march=rv32imfc -mabi=ilp32f -msoft-float -O2 -g -ffunction-sections \
    -fdata-sections -nostartfiles -Wl,-e,main -Wl,--gc-sections \
    -I/project/tests -I/project/tests/stubs -I/project/cslam-gap9-app/icp \
    -I/project/cslam-gap9-app/slam \
    /project/tests/gap9_gvsoc_smoke.c /project/cslam-gap9-app/icp/icp.c \
    -Wl,--allow-multiple-definition -Wl,--defsym=stack=0x1c100000 \
    -lm -lnosys -T /gap_sdk/gvsoc/gvsoc/docs/developer_manual/tutorials/utils/link.ld \
    -o /work/nano_c_slam_gvsoc_smoke.elf
"$objcopy" -O binary /work/nano_c_slam_gvsoc_smoke.elf /work/nano_c_slam_gvsoc_smoke.bin
'

ELF="$WORK_ROOT/nano_c_slam_gvsoc_smoke.elf"
BIN="$WORK_ROOT/nano_c_slam_gvsoc_smoke.bin"
echo "Built the GAP9-targeted Nano-C-SLAM ICP smoke ELF."
echo "The local MacOS SDK has only Mach-O GVSOC libraries and launcher; the local Linux"
echo "toolchain is only available in the Docker build container. A Linux GVSOC runtime"
echo "is required to execute this ELF. Peripheral, ToF, SPI, UWB, and flight control"
echo "are not simulated by this ICP-only harness."
printf 'ELF: %s\nBinary: %s\n' "$ELF" "$BIN"

if [[ "${NANO_C_SLAM_RUN_GVSOC:-0}" != "1" ]]; then
    exit 0
fi

set +u
source "$SDK_ROOT/configs/gap9_v2.sh"
set -u
export PATH="$SDK_ROOT/utils/gapy_v2/bin:$SDK_ROOT/utils/bin:$PATH"
export PYTHONPATH="$SDK_ROOT/gvsoc/gvsoc/models:$SDK_ROOT/gvsoc/gvsoc_gap:$SDK_ROOT/install/workstation/python:$SDK_ROOT/utils:${PYTHONPATH:-}"
export GVSOC_MODULES="$SDK_ROOT/gvsoc/gvsoc/models;$SDK_ROOT/gvsoc/gvsoc_gap"
export DYLD_LIBRARY_PATH="$SDK_ROOT/install/workstation/lib:${DYLD_LIBRARY_PATH:-}"

cd "$WORK_ROOT"
"$SDK_ROOT/utils/gapy_v2/bin/gapy" --platform gvsoc --target gap9.gap9_v2 \
    --target-dir "$SDK_ROOT/gvsoc/gvsoc_gap" \
    --target-dir "$SDK_ROOT/gvsoc/gvsoc/models" \
    --model-dir "$SDK_ROOT/install/workstation/models" \
    --binary "$ELF" --work-dir "$WORK_ROOT" run
