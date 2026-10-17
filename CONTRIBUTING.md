# Contributing

Work in a branch and keep hardware changes separate from host tooling or documentation changes. Include the target board, toolchain and SDK versions, and the observed result when reporting a build or runtime issue.

The checks available without flight hardware are:

```sh
git submodule update --init --recursive
uv sync --locked
uv run --locked ruff check .
uv run --locked ruff format --check .
uv run --locked ty check --error-on-warning
uv run --locked python -m unittest discover -s tests -p 'test_*.py'
shellcheck -s bash -x tests/run_point_cloud_test.sh cslam-stm32-app/deploy.sh scripts/run_gap9_gvsoc_smoke.sh scripts/build_gap9_firmware.sh scripts/gvsoc_tty_launcher.sh
uv run --locked python -m py_compile cslam-stm32-app/run_demo.py
cc -std=c11 -Wall -Wextra -Werror tests/pose_update_packet_test.c -lm -o /tmp/pose_update_packet_test
/tmp/pose_update_packet_test
sh tests/run_point_cloud_test.sh
cd crazyflie-firmware && git apply ../changes.patch && cd ..
cd cslam-stm32-app && make all
```

Build the full GAP9 application with `bash scripts/build_gap9_firmware.sh` when the GAP SDK and RISC-V toolchain are installed under `~/Work/da3_gap9`. Run the SDK-linked ICP SVD smoke on the local GVSOC model with `bash scripts/run_gap9_gvsoc_smoke.sh`; this writes a bounded evidence report under `artifacts/gap9_gvsoc_evidence/`. It does not establish full ICP or pose-graph behavior. Current runtime limits and memory use are recorded in [REPRODUCTION.md](REPRODUCTION.md). The Gazebo bridge and launch steps are documented in [sim/gazebo/README.md](sim/gazebo/README.md).

Build and capture the deterministic mapping smoke through host PMSIS stubs with `bash scripts/capture_gap9_host_mapping_diagnostic.sh`. The command writes a schema-validated host-only artifact to `artifacts/gap9_gvsoc_evidence/host_mapping_diagnostic.json` by default; pass another path as its first argument to preserve multiple runs. This is a host regression baseline, not GVSOC target, sensor, SPI, flight, or hardware validation.

To run the workflow locally with `act` and a Docker-compatible engine:

```sh
DOCKER_HOST="$(docker context inspect --format '{{.Endpoints.docker.Host}}')" \
  act -W .github/workflows/checks.yml \
  -P ubuntu-24.04=catthehacker/ubuntu:act-latest \
  --container-daemon-socket -
```

The STM32 job installs the ARM toolchain in the runner, so its first run downloads several hundred megabytes.

The patch modifies the pinned Crazyflie submodule in the local checkout. If it is already applied, skip the `git apply` step. Do not commit a changed submodule revision just to carry the patch. The GAP9 build needs a compatible SDK; see [REPRODUCTION.md](REPRODUCTION.md) for the currently tested setup and blocker.

Describe how a change was validated. For flight or mapping changes, include the number of drones, firmware revision, environment, and logs. Do not treat a successful compile as evidence of flight safety or mapping accuracy.
