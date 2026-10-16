# Contributing

Work in a branch and keep hardware changes separate from host tooling or documentation changes. Include the target board, toolchain and SDK versions, and the observed result when reporting a build or runtime issue.

The checks available without flight hardware are:

```sh
git submodule update --init --recursive
uv sync --locked
uv run --locked python -m py_compile cslam-stm32-app/run_demo.py
cd crazyflie-firmware && git apply ../changes.patch && cd ..
cd cslam-stm32-app && make all
```

The patch modifies the pinned Crazyflie submodule in the local checkout. If it is already applied, skip the `git apply` step. Do not commit a changed submodule revision just to carry the patch. The GAP9 build needs a compatible SDK; see [REPRODUCTION.md](REPRODUCTION.md) for the currently tested setup and blocker.

Describe how a change was validated. For flight or mapping changes, include the number of drones, firmware revision, environment, and logs. Do not treat a successful compile as evidence of flight safety or mapping accuracy.
