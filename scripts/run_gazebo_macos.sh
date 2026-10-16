#!/bin/bash
set -euo pipefail

ros_prefix="${ROS2_PREFIX:-/opt/homebrew/opt/ros2-jazzy}"
gz_install="${GZ_MACOS_INSTALL:-$HOME/Work/gz-macOS/ws/install}"
python311="/Library/Frameworks/Python.framework/Versions/3.11/bin/python3.11"
workspace="${CRAZYSWARM2_WS:-$HOME/Work/crazyswarm2_gazebo}"
firmware_build="${CRAZYFLIE_FIRMWARE_BUILD:-$HOME/Work/Nano-C-SLAM/crazyflie-firmware/build}"
homebrew_prefix="$(brew --prefix)"
abseil_opt="$homebrew_prefix/opt/abseil"
abseil_old="$homebrew_prefix/opt/abseil@20260107.0"

if [[ ! -x "$python311" || ! -f "$ros_prefix/setup.bash" ]]; then
    echo "Install Python.org 3.11.9 and Homebrew ROS 2 Jazzy first." >&2
    exit 1
fi
if [[ ! -f "$gz_install/setup.bash" || ! -x "$gz_install/bin/gz" ]]; then
    echo "Build Gazebo macOS first: $gz_install" >&2
    exit 1
fi
if [[ ! -f "$workspace/install/local_setup.bash" ]]; then
    echo "Build the Crazyswarm2 workspace first: $workspace" >&2
    exit 1
fi
if [[ ! -d "$abseil_old/lib" ]]; then
    echo "Install the ROS compatibility dependencies; see sim/gazebo/README.md." >&2
    exit 1
fi

temporary_dir="$(mktemp -d "${TMPDIR:-/tmp}/nano-c-slam-ros2.XXXXXX")"
abseil_original_target="$(readlink "$abseil_opt")"
cleanup() {
    ln -sfn "$abseil_original_target" "$abseil_opt"
    rm -rf "$temporary_dir"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

current_abseil_target="$(cd "$(dirname "$abseil_opt")/$abseil_original_target" && pwd -P)"
mkdir -p "$temporary_dir/abseil/lib"
for library in "$abseil_old"/lib/*.dylib "$current_abseil_target"/lib/*.dylib; do
    ln -sfn "$library" "$temporary_dir/abseil/lib/${library##*/}"
done
ln -sfn "$temporary_dir/abseil" "$abseil_opt"
export PATH="/Library/Frameworks/Python.framework/Versions/3.11/bin:$ros_prefix/bin:$gz_install/bin:$PATH"
set +u
# shellcheck source=/dev/null
source "$ros_prefix/setup.bash"
# shellcheck source=/dev/null
source "$workspace/install/local_setup.bash"
# shellcheck source=/dev/null
source "$gz_install/setup.bash"
set -u
# Keep the source-built Gazebo CLI ahead of Homebrew's ROS-bundled CLI.
export PATH="$gz_install/bin:$PATH"
export PYTHONPATH="$firmware_build${PYTHONPATH:+:$PYTHONPATH}"
export GZ_CONFIG_PATH="$gz_install/share/gz"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$gz_install/lib:$ros_prefix/lib/gz-sim-8/plugins:$ros_prefix/lib"
export GZ_SIM_PHYSICS_ENGINE_PATH="$gz_install/lib/gz-physics-7/engine-plugins"
gazebo_models="$(uv run --no-project --python "$python311" ros2 pkg prefix ros_gz_crazyflie_gazebo)/share/ros_gz_crazyflie_gazebo/models"
export GZ_SIM_RESOURCE_PATH="$gazebo_models:$gz_install/share/gz/gz-sim8/worlds"
export GZ_RENDERING_PLUGIN_PATH="$gz_install/lib/gz-rendering-8/engine-plugins"
export GZ_RENDERING_RESOURCE_PATH="$gz_install/share/gz/gz-rendering8"
export GZ_GUI_PLUGIN_PATH="$gz_install/lib/gz-sim-8/plugins/gui:$gz_install/lib/gz-gui-8/plugins"
export QML2_IMPORT_PATH="$gz_install/lib/gz-sim-8/plugins/gui"
export GZ_PARTITION="${NANO_C_SLAM_GZ_PARTITION:-nano_c_slam_${UID:-$(id -u)}}"
export GZ_DISCOVERY_MULTICAST_IP="${GZ_DISCOVERY_MULTICAST_IP:-239.255.0.17}"
export GZ_DISCOVERY_MSG_PORT="${GZ_DISCOVERY_MSG_PORT:-42327}"
export GZ_DISCOVERY_SRV_PORT="${GZ_DISCOVERY_SRV_PORT:-42328}"

printf 'Gazebo: %s\n' "$(command -v gz)"
printf 'Gazebo partition: %s\n' "$GZ_PARTITION"
printf 'Gazebo GUI will join after the server starts.\n'

uv run --no-project --python "$python311" ros2 launch \
    nano_c_slam_gazebo crazyswarm2_gazebo.launch.py
