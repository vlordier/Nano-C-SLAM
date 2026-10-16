#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SDK_ROOT="${GAP9_SDK_ROOT:-$HOME/Work/da3_gap9/third_party/gap_sdk_private}"
TOOLCHAIN_ROOT="${GAP9_TOOLCHAIN_ROOT:-$HOME/Work/da3_gap9/deploy/gap_riscv_toolchain}"
STIMULUS_ROOT="${GAP9_STIMULUS_ROOT:-$HOME/Work/da3_gap9}"
WORK_ROOT="${GAP9_GVSOC_WORK_ROOT:-${TMPDIR:-/tmp}/nano-c-slam-gvsoc}"
ELF="$ROOT/cslam-gap9-app/BUILD-gvsoc/nano_c_slam_gvsoc_smoke"
LAYOUT="$WORK_ROOT/nano_c_slam_gvsoc_layout.json"
GVSOC_LOG="$WORK_ROOT/gvsoc.log"

if [[ "$(uname -s)" != Darwin ]]; then
    echo "This workflow runs with the local macOS GVSOC runtime." >&2
    exit 2
fi

for required in \
    "$SDK_ROOT/utils/gapy_v2/bin/gapy" \
    "$SDK_ROOT/install/workstation/bin/gvsoc_launcher" \
    "$SDK_ROOT/install/workstation/models/gap9/soc.dylib" \
    "$TOOLCHAIN_ROOT/bin/riscv32-unknown-elf-gcc"; do
    [[ -e "$required" ]] || { echo "missing GAP9 GVSOC prerequisite: $required" >&2; exit 2; }
done

if [[ ! -s "$ELF" ]]; then
    bash "$ROOT/scripts/build_gap9_firmware.sh"
fi

for stimulus in chip.efuse_preload.data chip.soc.mram.bin; do
    if [[ ! -e "$WORK_ROOT/$stimulus" ]]; then
        [[ -e "$STIMULUS_ROOT/$stimulus" ]] || {
            echo "missing GAP9 GVSOC stimulus: $STIMULUS_ROOT/$stimulus" >&2
            exit 2
        }
        mkdir -p "$WORK_ROOT"
        cp "$STIMULUS_ROOT/$stimulus" "$WORK_ROOT/$stimulus"
    fi
done
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

mkdir -p "$WORK_ROOT/launcher-bin"
if [[ "${GAP9_GVSOC_NO_TTY:-1}" == "1" ]]; then
    ln -sf "$GAP9_REAL_GVSOC_LAUNCHER" "$WORK_ROOT/launcher-bin/gvsoc_launcher"
else
    ln -sf "$ROOT/scripts/gvsoc_tty_launcher.sh" "$WORK_ROOT/launcher-bin/gvsoc_launcher"
fi
export PATH="$WORK_ROOT/launcher-bin:$PATH"

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

grep -q 'NANO_C_SLAM_GVSOC:icp_svd_ok sigma0_milli=3000 sigma1_milli=2000' "$GVSOC_LOG"
uv run --project "$ROOT" --python 3.12 \
    python "$ROOT/scripts/capture_gap9_gvsoc_evidence.py" \
    --elf "$ELF" --log "$GVSOC_LOG" \
    --output "$ROOT/artifacts/gap9_gvsoc_evidence/report.json"
echo "GAP9 GVSOC ICP SVD smoke passed; log: $GVSOC_LOG"
