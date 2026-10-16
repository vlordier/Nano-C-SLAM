# Reproduction status

Tested on macOS (Apple Silicon) on 2026-10-16. The repository's pinned submodules and `changes.patch` were used. No hardware flight or mapping accuracy result was reproduced.

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

The local GAP9 SDK at `~/Work/da3_gap9/third_party/gap_sdk_private` reports release 5.20.4. Its Make flow refers to absent FreeRTOS rules, but the SDK's CMake integration successfully builds the full application through `scripts/build_gap9_firmware.sh` (Docker + `uv`). This compiles the same PMSIS/FreeRTOS components from source without patching the SDK checkout. The successful ELF reports 58,934 bytes text, 522,816 data, 56,708 BSS; linker placement uses 634,992 B L2 shared (40.37%) and 117,596 B L1 (89.72%). L1 headroom is tight. The SDK emits a `pi_assert` implicit declaration warning from its own `udma_core.h` include chain.

The SDK's native GVSOC runtime is available locally on macOS; its model reaches the boot address, but the current handcrafted smoke image fails during boot handoff (`addr 0xfffffff0`). The bounded ICP-only ELF does build. No successful Nano-C-SLAM GVSOC runtime result is available yet. The smoke image does not model ToF, SPI, UWB, FreeRTOS task scheduling, or Crazyflie flight control. A supported boot image/layout and successful behavioral run are still needed.

## Gazebo SITL

`sim/gazebo/` contains a ROS 2 bridge for the four-ray Crazyswarm2 Gazebo backend. Pure sensor-conversion tests pass without ROS. The current macOS setup lacks ROS 2 Jazzy and Gazebo Harmonic, so the documented end-to-end SITL launch and topic check have not been run. The bridge supplies four-ray ranges and ideal pairwise distances from simulator ground truth; it does not run the embedded firmware or emulate hardware ToF/UWB interfaces. See [sim/gazebo/README.md](sim/gazebo/README.md).

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
