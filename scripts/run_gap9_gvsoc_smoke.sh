#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SDK_ROOT="${GAP9_SDK_ROOT:-$HOME/Work/da3_gap9/third_party/gap_sdk_private}"
TOOLCHAIN_ROOT="${GAP9_TOOLCHAIN_ROOT:-$HOME/Work/da3_gap9/deploy/gap_riscv_toolchain}"
STIMULUS_ROOT="${GAP9_STIMULUS_ROOT:-$HOME/Work/da3_gap9}"
WORK_ROOT="${GAP9_GVSOC_WORK_ROOT:-${TMPDIR:-/tmp}/nano-c-slam-gvsoc}"
DEFAULT_ELF="$ROOT/cslam-gap9-app/BUILD-gvsoc/nano_c_slam_gvsoc_smoke"
MAPPING_CAPTURE="${GAP9_GVSOC_MAPPING_CAPTURE:-0}"
if [[ "$MAPPING_CAPTURE" == "1" ]]; then
    DEFAULT_ELF="$ROOT/cslam-gap9-app/BUILD-gvsoc-mapping/nano_c_slam_gvsoc_mapping_smoke"
    DEFAULT_MARKER="NANO_C_SLAM_GVSOC:mapping_fail"
else
    DEFAULT_MARKER="NANO_C_SLAM_GVSOC:icp_svd_ok sigma0_milli=3000 sigma1_milli=2000"
fi
ELF="${GAP9_GVSOC_ELF:-$DEFAULT_ELF}"
EXPECTED_MARKER="${GAP9_GVSOC_EXPECTED_MARKER:-$DEFAULT_MARKER}"
LAYOUT="$WORK_ROOT/nano_c_slam_gvsoc_layout.json"
GVSOC_LOG="$WORK_ROOT/gvsoc.log"
MAPPING_OUTPUT="${GAP9_GVSOC_MAPPING_OUTPUT:-$ROOT/artifacts/gap9_gvsoc_evidence/mapping_diagnostic.json}"
RUN_TIMEOUT_S="${GAP9_GVSOC_TIMEOUT_S:-600}"

if [[ "$MAPPING_CAPTURE" == "1" ]]; then
    if [[ "$MAPPING_OUTPUT" != /* ]]; then
        MAPPING_OUTPUT="$ROOT/$MAPPING_OUTPUT"
    fi
    if [[ -n "${GAP9_GVSOC_MAPPING_OUTPUT+x}" ]]; then
        MAPPING_OUTPUT="$(uv run --project "$ROOT" --locked --python 3.12 \
            python "$ROOT/scripts/select_gap9_mapping_output.py" \
            "$MAPPING_OUTPUT" --explicit)"
    else
        MAPPING_OUTPUT="$(uv run --project "$ROOT" --locked --python 3.12 \
            python "$ROOT/scripts/select_gap9_mapping_output.py" \
            "$MAPPING_OUTPUT")"
    fi
    echo "GAP9 mapping evidence output: $MAPPING_OUTPUT"
fi

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

if [[ "$MAPPING_CAPTURE" == "1" || ! -s "$ELF" ]]; then
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

cd "$ROOT"
uv run --project "$ROOT" --python 3.12 \
    python "$ROOT/scripts/create_gap9_gvsoc_layout.py" "$ELF" "$LAYOUT"
set +e
uv run --project "$ROOT" --python 3.12 \
    python -m scripts.run_gap9_gvsoc_target \
    --log "$GVSOC_LOG" --cwd "$WORK_ROOT" --timeout-s "$RUN_TIMEOUT_S" -- \
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
    --work-dir "$WORK_ROOT" image flash run
GVSOC_EXIT_CODE=$?
set -e

if [[ "$MAPPING_CAPTURE" == "1" ]]; then
    if [[ "$GVSOC_EXIT_CODE" == "124" ]]; then
        echo "GVSOC timed out; raw log retained at $GVSOC_LOG, no mapping artifact published." >&2
        exit 124
    fi
    uv run --project "$ROOT" --python 3.12 \
        python -m scripts.capture_gap9_gvsoc_mapping_diagnostic \
        --elf "$ELF" --log "$GVSOC_LOG" \
        --compile-commands "$ROOT/cslam-gap9-app/BUILD-gvsoc-mapping/compile_commands.json" \
        --build-provenance "$ROOT/cslam-gap9-app/BUILD-gvsoc-mapping/build-provenance.json" \
        --sdk-root "$SDK_ROOT" --output "$MAPPING_OUTPUT" \
        --gvsoc-exit-code "$GVSOC_EXIT_CODE"
    exit $?
fi

if [[ "$GVSOC_EXIT_CODE" != "0" ]]; then
    echo "GVSOC command failed with status $GVSOC_EXIT_CODE; log retained at $GVSOC_LOG" >&2
    exit "$GVSOC_EXIT_CODE"
fi

grep -Fq "$EXPECTED_MARKER" "$GVSOC_LOG"
if [[ "$ELF" == "$DEFAULT_ELF" ]]; then
    uv run --project "$ROOT" --python 3.12 \
        python "$ROOT/scripts/capture_gap9_gvsoc_evidence.py" \
        --elf "$ELF" --log "$GVSOC_LOG" \
        --output "$ROOT/artifacts/gap9_gvsoc_evidence/report.json"
    echo "GAP9 GVSOC ICP SVD smoke passed; log: $GVSOC_LOG"
elif [[ "$EXPECTED_MARKER" == NANO_C_SLAM_GVSOC:mapping_ok* ]]; then
    uv run --project "$ROOT" --python 3.12 \
        python "$ROOT/scripts/capture_gap9_gvsoc_mapping_evidence.py" \
        --elf "$ELF" --log "$GVSOC_LOG" \
        --output "$ROOT/artifacts/gap9_gvsoc_evidence/mapping_report.json"
    echo "GAP9 GVSOC mapping-core smoke passed; log: $GVSOC_LOG"
else
    echo "GAP9 GVSOC observed expected marker '$EXPECTED_MARKER'; log: $GVSOC_LOG"
fi
