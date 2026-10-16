# Reproduction status

Tested on macOS (Apple Silicon) on 2026-10-16. The repository's pinned submodules and `changes.patch` were used. Gazebo and ROS 2 SITL ran natively on macOS. No hardware flight or mapping-accuracy result was reproduced.

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

The local GAP9 SDK at `~/Work/da3_gap9/third_party/gap_sdk_private` reports release 5.20.4. The direct Make flow refers to absent FreeRTOS rules. On 2026-10-17, the SDK CMake build completed through the x86_64 `colima-act-ci` VM using Docker-over-SSH and `uv`, without modifying the SDK checkout. The current firmware reports 62,486 bytes text, 526,920 data, and 56,700 BSS; linker placement uses 642,640 B L2 shared (40.86%) and 117,612 B L1 (89.73%). L1 headroom is tight. The SDK emits a `pi_assert` implicit declaration warning from its own `udma_core.h` include chain.

The native GVSOC launcher is available locally on macOS, and the tracked result in `artifacts/gap9_gvsoc_evidence/report.json` observes the expected ICP SVD singular values (3.000, 2.000). This is kernel-only evidence. The full firmware, the SVD smoke, and the mapping-core smoke all compiled successfully with the SDK 5.20.4 toolchain in the x86_64 Colima VM. The full firmware uses 89.73% of L1; the mapping smoke uses 579,872 B of L2 shared and 117,612 B of L1. The mapping ELF has not yet been run in native GVSOC, and `mapping_report.json` is not present, so end-to-end GVSOC mapping is not established. The Mac's filesystem ran out of space during the attempt to retain the ELF in a persistent build directory; the build script now places container outputs in the ignored `.build-gap9-container/` directory so a subsequent incremental build can preserve them.

```sh
DOCKER_HOST=ssh://colima-act-ci bash scripts/build_gap9_firmware.sh
GAP9_GVSOC_ELF="$PWD/cslam-gap9-app/BUILD-gvsoc-mapping/nano_c_slam_gvsoc_mapping_smoke" \
GAP9_GVSOC_EXPECTED_MARKER='NANO_C_SLAM_GVSOC:mapping_ok poses=60 scans=3 map_points=240 optimized=60 graph=3 has_nan=0' \
bash scripts/run_gap9_gvsoc_smoke.sh
```

The build command uses four jobs by default (`GAP9_BUILD_JOBS` can override this). A successful mapping run verifies the deterministic synthetic pose/scan fixture only; it does not establish sensor, SPI, flight, or mapping-accuracy behavior. The complete firmware ELF was also observed reaching `main()` and opening the cluster in this virtual model, then waiting for the STM32 SPI packet. GVSOC has no STM32/SPI stimulus in this setup, so that wait is expected. Neither smoke establishes hardware timing or flight safety.

## Gazebo SITL

`sim/gazebo/` contains a ROS 2 bridge for the four-ray Crazyswarm2 Gazebo backend. ROS 2 Jazzy and Gazebo Harmonic ran natively on macOS using the setup documented in [sim/gazebo/README.md](sim/gazebo/README.md). The bounded square mission completed takeoff, mapping-path flight, hover, and landing; a separate 150-sample replay passed through the host C mapping core using the Gazebo 3.49 m lidar range. Its report, `artifacts/gazebo_mapping_sitl/cf1_native_gazebo_mapping_path.json`, is explicitly historical and not firmware-faithful because the mapper then used a broader range cutoff.

The current host replay applies the firmware's 1.4 m loop-closure scan cutoff. A repeated native Gazebo capture returned no projected points under that limit and failed safely. The deterministic host fixture uses 0.9–1.2 m readings and passes with 60 poses, three scans, 240 map points, and two ICP constraints; a 1.5 m-only fixture is rejected. Gazebo inputs remain a four-ray lidar and ideal simulated odometry, and this host runner does not execute the GAP9 firmware or emulate hardware ToF/UWB interfaces.

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
