# Reproduction status

Tested on macOS (Apple Silicon), with the evidence dates recorded below. The repository's pinned submodules and `changes.patch` were used. Gazebo and ROS 2 SITL ran natively on macOS. No hardware flight or mapping-accuracy result was reproduced.

## Current capability status (2026-10-18)

| Capability | Status | Evidence and boundary |
| --- | --- | --- |
| One Crazyflie flying a bounded mission in native Gazebo | Passed historically | Native ROS 2 Jazzy/Gazebo Harmonic run documented below and in `sim/gazebo/README.md`; this is simulated flight. |
| Owned host C mapper processing Gazebo telemetry | Passed for retained captures | Near-field replay has 170 timestamped pose/range pairs, 9 scans, 588 projected points and 11 stored constraints submitted to the final optimizer call. This proves pipeline execution, not map accuracy or convergence. |
| GAP9 firmware build | Passed 2026-10-18 | Fresh SDK 5.20.4 build: 63,062 text, 526,920 data, 56,700 BSS; L1 117,612/131,068 B (89.73%), L2 shared 643,216/1,572,864 B (40.89%). |
| GAP9 GVSOC ICP kernel smoke | Build passed; execution evidence is older | Retained `report.json` is kernel-only evidence. It does not exercise mapping. |
| GAP9 GVSOC mapping-core smoke | Target image built 2026-10-18; fresh execution not completed | The current target ELF links after fixing a target-only `cluster_dev` declaration in `tests/gap9_sdk_gvsoc_mapping_smoke.c`. The mapping target uses 586,448 B L2 shared and 117,612 B L1. A fresh schema-v2 simulator report was not produced because Colima/Docker stopped responding during the rerun. The October schema-v1 failure remains historical and is not current-HEAD evidence. |
| Production GAP9 main loop consuming deterministic STM32 SPI packets | Not demonstrated | Current mapping smoke omits `main.c` and `spi/*.c`; see beads `ncs-ju6.4.1`–`ncs-ju6.4.5`. |
| Three-drone mission, map fusion, physical flight, quantitative map accuracy | Not demonstrated | Separate open work remains; per-drone mapping is not map fusion, and simulated four-ray lidar/ideal odometry are not hardware ToF/UWB. |

The next validation order is explicit in beads: finish the fresh provenance-bound GVSOC mapping run (`ncs-ju6.3.1.3.3`), interpret its stage diagnostics (`ncs-ju6.3.1.4`), compare identical host/target fixture results (`ncs-ju6.3.2`), then fix any measured ICP divergence (`ncs-ju6.3.3`). Only after the core smoke is understood should production SPI stimulus (`ncs-ju6.4`) and the final capability bundle (`ncs-ju6.8`) be treated as downstream goals.

## STM32 firmware

Prerequisites: an ARM GNU toolchain, `make`, and the repository's submodules. The tested toolchain was `arm-none-eabi-gcc` 15.3.1.

```sh
git submodule update --init --recursive
cd crazyflie-firmware
git apply ../changes.patch
cd ../cslam-stm32-app
make all
```

The build succeeds on this branch and produces `cslam-stm32-app/build/cf2.bin`. The linker reports 90% RAM and 95% CCM usage. The patch modifies files inside the pinned Crazyflie firmware submodule; it is required by the upstream build instructions and is not a submodule revision change.

Runtime buffers are fixed-size. The STM32 tracks up to 30 scan poses and queues up to 100 external augmented poses or 200 pose updates between transfers to GAP9. When a buffer fills, new data is dropped and a debug message is emitted. Longer missions and larger swarms require a measured capacity plan.

## GAP9 firmware

The local GAP9 SDK at `~/Work/da3_gap9/third_party/gap_sdk_private` reports release 5.20.4. The direct Make flow refers to absent FreeRTOS rules. On 2026-10-18, the SDK CMake build completed through the x86_64 `colima-act-ci` VM using Docker-over-SSH and `uv`, without modifying the SDK checkout. The current firmware reports 63,062 bytes text, 526,920 data, and 56,700 BSS; linker placement uses 643,216 B L2 shared (40.89%) and 117,612 B L1 (89.73%). L1 headroom is tight. The SDK emits a `pi_assert` implicit declaration warning from its own `udma_core.h` include chain.

The native GVSOC launcher is available locally on macOS, and the tracked result in `artifacts/gap9_gvsoc_evidence/report.json` observes the expected ICP SVD singular values (3.000, 2.000). This is kernel-only evidence. The full firmware, the SVD smoke, and the mapping-core smoke compile with the SDK 5.20.4 toolchain in the x86_64 Colima VM. The older `artifacts/gap9_gvsoc_evidence/mapping_limitation.json` is a schema-v1 run from repository revision `4f9ae1384f10c42360942b5d2d65f5e8b176a1a9`, not current-HEAD evidence. That run accepted 60 poses, extracted three scans and 240 map points, and called cluster ICP; its last pair had 80 nearest-neighbor matches, but ICP returned an invalid result, leaving zero accepted constraints and optimized poses. Its ELF/log hashes are retained, but the host/GVSOC root cause is not isolated. A fresh schema-v2 run is still pending, so the historical failure is not a current mapping verdict.

```sh
DOCKER_HOST=ssh://colima-act-ci bash scripts/build_gap9_firmware.sh
bash scripts/build_host_mapping_sitl.sh
cslam-gap9-app/BUILD-host/gap9_mapping_smoke
GAP9_GVSOC_ELF="$PWD/cslam-gap9-app/BUILD-gvsoc-mapping/nano_c_slam_gvsoc_mapping_smoke" \
bash scripts/run_gap9_gvsoc_smoke.sh # exits 1 after the expected mapping_fail marker
uv run --locked python scripts/capture_gap9_gvsoc_mapping_limitation.py \
  --elf cslam-gap9-app/BUILD-gvsoc-mapping/nano_c_slam_gvsoc_mapping_smoke \
  --log "$TMPDIR/nano-c-slam-gvsoc/gvsoc.log" \
  --output artifacts/gap9_gvsoc_evidence/mapping_limitation.json \
  --gvsoc-exit-code 1
```

The build command uses four jobs by default (`GAP9_BUILD_JOBS` can override this); set `GAP9_BUILD_DNS` if the Colima VM needs a different resolver. The separate host smoke passes the same owned point-cloud/ICP/pose-graph fixture with PMSIS stubs at the cluster compute boundary. Its result does not turn the GVSOC failure into a firmware mapping pass. Neither run establishes sensor, SPI, flight, hardware timing, or mapping accuracy. The complete firmware ELF was also observed reaching `main()` and opening the cluster in this virtual model, then waiting for the STM32 SPI packet; GVSOC has no STM32/SPI stimulus in this setup.

## Gazebo SITL

`sim/gazebo/` contains a ROS 2 bridge for the four-ray Crazyswarm2 Gazebo backend. ROS 2 Jazzy and Gazebo Harmonic ran natively on macOS using the setup documented in [sim/gazebo/README.md](sim/gazebo/README.md). The bounded square mission completed takeoff, mapping-path flight, hover, and landing; a separate 150-sample replay passed through the host C mapping core using the Gazebo 3.49 m lidar range. Its report, `artifacts/gazebo_mapping_sitl/cf1_native_gazebo_mapping_path.json`, is explicitly historical and not firmware-faithful because the mapper then used a broader range cutoff.

The default Gazebo arena returns no projected points under the firmware's 1.4 m loop-closure cutoff. The optional `NANO_C_SLAM_NEARFIELD_MAPPING=1` launch derives a temporary near-field world from the installed Crazyswarm2 SDF without editing that checkout or changing the cutoff. A native macOS run captured 60 fresh poses during a bounded square mission and completed landing; the owned host C mapper produced 3 scans, 200 projected points, 4 unique constraints stored and passed to optimization (2 explicit ICP correction attempts plus internal scan constraints), and a finite 3-node graph. All three 20-pose windows had in-range readings; across the run, each of the four beams had multiple distinct readings at or below 1400 mm. Range accounting recorded 240 readings: 11 invalid, 200 accepted, and 29 valid readings above the limit. The capture and mission log are retained in `artifacts/gazebo_mapping_sitl/cf1_nearfield_firmware_cutoff.json` and `artifacts/gazebo_mapping_sitl/cf1_nearfield_mission.log`. This proves host mapping-pipeline execution on simulated four-ray lidar and odometry, not map accuracy, GAP9 firmware/SPI execution, or hardware ToF/UWB behavior.

A newer stamped capture at `artifacts/gazebo_mapping_sitl/cf1_nearfield_stamped_capture.json` contains 170 same-drone range/odometry pairs, 9 scans, 588 projected points, 11 unique stored constraints submitted to the final `ls_slam` call, and a finite 9-node graph. The report records adjacent ICP scan-pair attempts and accepted insertions separately from all stored canonical endpoint pairs and compact optimizer-input edges. Source timestamps had 0 ns pair skew and 10 Hz median frequency; 680 readings reconcile to 11 invalid, 588 within the exact 1400 mm cutoff, and 81 valid readings above it. This demonstrates execution of the owned host mapping path on captured Gazebo telemetry. It does not establish optimizer convergence or map accuracy, nor does it bind the mission's human-readable landing result to this capture; those remain open beads. See `cslam-gap9-app/point-cloud/CONSTRAINT_ACCOUNTING.md` for the exact constraint semantics.

`artifacts/gazebo_mapping_sitl/cf1_nearfield_stamped_freshness.json` is a separate native freshness capture with per-pair age measurements: 170 pairs at 10 Hz, 100–112 ms source age, and 0 ns range/odometry skew. The host mapping replay also completed (170 poses, 9 scans, 645 projected points, 15 unique stored constraints submitted to the final optimizer call). Adversarial review found only 2.7 µm horizontal path length, so this artifact supports telemetry freshness and replay execution only; it does not demonstrate spatial excitation or a useful map. A subsequent attempt at a clean square mapping run failed when Gazebo odometry went stale, and that failed run wrote no report.

The older `cf1_native_gazebo*.json` captures have no retained raw pose/range samples or endpoint pairs. Their historic scalar constraint count is marked `legacy-unverifiable` and cannot establish unique graph edges or optimizer input.

## Python demo

The demo dependencies are locked in `pyproject.toml` and `uv.lock`. The tested environment uses Python 3.12 and `cflib` 0.1.34:

```sh
uv sync --locked
uv run --locked ruff check .
uv run --locked ruff format --check .
uv run --locked ty check --error-on-warning
uv run --locked python -m unittest discover -s tests -p 'test_*.py'
uv run --locked python -m py_compile cslam-stm32-app/run_demo.py
```

The runtime requires a Crazyradio and configured drones. From `cslam-stm32-app/`, run `uv run --locked --project .. python run_demo.py 2`. This command reaches the radio connection step on the test machine, then exits because no Crazyradio is attached. No flight was started.
