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

## GAP9 firmware

The local GAP9 SDK at `~/Work/da3_gap9/third_party/gap_sdk` reports release 5.20.4. With its `configs/gap9_evk_audio.sh` environment, `make all` stops before compilation: `utils/rules/pmsis_rules.mk` includes `rtos/pmsis/os/freeRTOS/vendors/gwt/rules/freeRTOS_rules.mk`, which is absent from that SDK checkout. The adjacent `gap_sdk_private` checkout also lacks this file. A compatible SDK revision or build configuration is still needed to reproduce the GAP9 binary.

## Python demo

The demo imports with Python 3.12, `cflib` 0.1.34, and NumPy 2.5.4:

```sh
uv venv --python 3.12 .venv
uv pip install --python .venv/bin/python cflib==0.1.34 numpy==2.5.4
.venv/bin/python -m py_compile cslam-stm32-app/run_demo.py
```

The runtime requires a Crazyradio and configured drones. `cflib.crtp.scan_interfaces()` found no Crazyradio on the test machine, so the mission was not started.
