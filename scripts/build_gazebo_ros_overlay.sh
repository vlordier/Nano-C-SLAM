#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python311="/Library/Frameworks/Python.framework/Versions/3.11/bin/python3.11"
if [[ -n "${NANO_C_SLAM_ROS_PYTHON:-}" ]]; then
    python_executable="$NANO_C_SLAM_ROS_PYTHON"
elif [[ -x "$python311" ]]; then
    python_executable="$python311"
else
    python_executable="$(command -v python3 || true)"
fi

if [[ -z "$python_executable" || ! -x "$python_executable" ]]; then
    echo "Python 3 was not found; source the ROS 2 environment first." >&2
    exit 1
fi
if [[ -z "${AMENT_PREFIX_PATH:-}" ]]; then
    echo "ROS 2 is not sourced; source Jazzy and the Crazyswarm2 workspace first." >&2
    exit 1
fi
if ! command -v colcon >/dev/null 2>&1; then
    echo "colcon was not found; source the ROS 2 environment first." >&2
    exit 1
fi
if ! command -v uv >/dev/null 2>&1; then
    echo "uv is required to run the Python build command." >&2
    exit 1
fi

mkdir -p "$repo_root/build"
build_root="$(mktemp -d "$repo_root/build/gazebo-ros-overlay.XXXXXX")"
interface_package="$repo_root/sim/gazebo/nano_c_slam_interfaces"
gazebo_package="$repo_root/sim/gazebo/nano_c_slam_gazebo"

uv run --no-project --python "$python_executable" colcon \
    --log-base "$build_root/log" build \
    --base-paths "$interface_package" "$gazebo_package" \
    --build-base "$build_root/build" \
    --install-base "$build_root/install" \
    --packages-up-to nano_c_slam_gazebo \
    --event-handlers console_direct+ \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    "-DPython3_EXECUTABLE=$python_executable"

printf 'Nano-C-SLAM ROS overlay built successfully.\n'
printf 'Source it in this shell with:\n  source "%s/install/local_setup.bash"\n' \
    "$build_root"
