#!/bin/bash
set -euo pipefail

ros_prefix="${ROS2_PREFIX:-/opt/homebrew/opt/ros2-jazzy}"
gz_install="${GZ_MACOS_INSTALL:-$HOME/Work/gz-macOS/ws/install}"
python311="/Library/Frameworks/Python.framework/Versions/3.11/bin/python3.11"
workspace="${CRAZYSWARM2_WS:-$HOME/Work/crazyswarm2_gazebo}"
firmware_build="${CRAZYFLIE_FIRMWARE_BUILD:-$HOME/Work/Nano-C-SLAM/crazyflie-firmware/build}"
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
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
if [[ -e "$abseil_opt" && ! -L "$abseil_opt" ]]; then
    echo "Expected Homebrew's Abseil opt path to be a symlink: $abseil_opt" >&2
    exit 1
fi
"$repo_root/scripts/check_gazebo_abseil_opt.sh" "$abseil_opt"

temporary_dir="$(mktemp -d "${TMPDIR:-/tmp}/nano-c-slam-ros2.XXXXXX")"
abseil_original_target="$(readlink "$abseil_opt" 2>/dev/null || true)"
abseil_original_path="$abseil_original_target"
if [[ -n "$abseil_original_path" && "$abseil_original_path" != /* ]]; then
    abseil_original_path="$(dirname "$abseil_opt")/$abseil_original_path"
fi
if [[ -z "$abseil_original_path" || ! -d "$abseil_original_path/lib" ]]; then
    abseil_cellar="$(brew --cellar abseil)"
    abseil_version="$(brew list --versions abseil | awk '{print $2}')"
    abseil_original_path="$abseil_cellar/$abseil_version"
    if [[ -z "$abseil_original_path" || ! -d "$abseil_original_path/lib" ]]; then
        rm -rf "$temporary_dir"
        echo "Could not locate the installed Abseil libraries under $abseil_cellar." >&2
        exit 1
    fi
    abseil_original_target="../Cellar/abseil/${abseil_original_path##*/}"
fi
abseil_swapped=0
cleanup() {
    if [[ "$abseil_swapped" -eq 1 ]]; then
        ln -sfn "$abseil_original_target" "$abseil_opt" || true
    fi
    rm -rf "$temporary_dir"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

current_abseil_target="$(cd "$abseil_original_path" && pwd -P)"
mkdir -p "$temporary_dir/abseil/lib"
for library in "$abseil_old"/lib/*.dylib "$current_abseil_target"/lib/*.dylib; do
    ln -sfn "$library" "$temporary_dir/abseil/lib/${library##*/}"
done
abseil_swapped=1
ln -sfn "$temporary_dir/abseil" "$abseil_opt"
export PATH="/Library/Frameworks/Python.framework/Versions/3.11/bin:$ros_prefix/bin:$gz_install/bin:$PATH"
set +u
# shellcheck source=/dev/null
source "$ros_prefix/setup.bash"
# shellcheck source=/dev/null
source "$workspace/install/local_setup.bash"
# shellcheck source=/dev/null
source "$gz_install/setup.bash"
if [[ -n "${NANO_C_SLAM_OVERLAY_SETUP:-}" ]]; then
    if [[ ! -f "$NANO_C_SLAM_OVERLAY_SETUP" ]]; then
        echo "Nano-C-SLAM ROS overlay setup not found: $NANO_C_SLAM_OVERLAY_SETUP" >&2
        exit 1
    fi
    # shellcheck source=/dev/null
    source "$NANO_C_SLAM_OVERLAY_SETUP"
fi
set -u
# Keep the source-built Gazebo CLI ahead of Homebrew's ROS-bundled CLI.
export PATH="$gz_install/bin:$PATH"

# Fail before starting Gazebo if the sourced workspace still exposes the old
# telemetry interface. The mapping verifier requires source odometry stamps.
if ! interface_definition="$(uv run --no-project --python "$python311" \
    ros2 interface show nano_c_slam_interfaces/msg/StampedPairwiseRanges 2>&1)"; then
    printf '%s\n' \
        "The sourced ROS workspace does not provide StampedPairwiseRanges." \
        "Build the Nano-C-SLAM overlay with scripts/build_gazebo_ros_overlay.sh and set NANO_C_SLAM_OVERLAY_SETUP to its local_setup.bash." \
        "ROS interface lookup output: $interface_definition" >&2
    exit 1
fi
if [[ "$interface_definition" != *"ranges_m"* || "$interface_definition" != *"source_stamps"* ]]; then
    echo "The sourced StampedPairwiseRanges interface is stale; rebuild and source the current Nano-C-SLAM ROS overlay." >&2
    exit 1
fi

export PYTHONPATH="$firmware_build${PYTHONPATH:+:$PYTHONPATH}"
export GZ_CONFIG_PATH="$gz_install/share/gz"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$gz_install/lib:$ros_prefix/lib/gz-sim-8/plugins:$ros_prefix/lib"
export GZ_SIM_PHYSICS_ENGINE_PATH="$gz_install/lib/gz-physics-7/engine-plugins"
gazebo_models="$(uv run --no-project --python "$python311" ros2 pkg prefix ros_gz_crazyflie_gazebo)/share/ros_gz_crazyflie_gazebo/models"
nearfield_world=""
if [[ "${NANO_C_SLAM_NEARFIELD_MAPPING:-0}" == "1" ]]; then
    gazebo_worlds="$(uv run --no-project --python "$python311" ros2 pkg prefix ros_gz_crazyflie_gazebo)/share/ros_gz_crazyflie_gazebo/worlds"
    nearfield_world="$temporary_dir/nano_c_slam_nearfield.sdf"
    uv run --no-project --python "$python311" python \
        "$repo_root/scripts/prepare_gazebo_nearfield_world.py" \
        --source "$gazebo_worlds/crazyflie_swarm_world.sdf" \
        --output "$nearfield_world"
    printf 'Mapping scene: near-field walls/obstacles inside the 1.4 m gate\n'
fi
export GZ_SIM_RESOURCE_PATH="$gazebo_models:$gz_install/share/gz/gz-sim8/worlds"
export GZ_RENDERING_PLUGIN_PATH="$gz_install/lib/gz-rendering-8/engine-plugins"
export GZ_RENDERING_RESOURCE_PATH="$gz_install/share/gz/gz-rendering8"
export GZ_GUI_PLUGIN_PATH="$gz_install/lib/gz-sim-8/plugins/gui:$gz_install/lib/gz-gui-8/plugins"
export QML2_IMPORT_PATH="$gz_install/lib/gz-sim-8/plugins/gui"
export GZ_PARTITION="${NANO_C_SLAM_GZ_PARTITION:-nano_c_slam_${UID:-$(id -u)}}"
# A local GUI/server pair should advertise loopback instead of every macOS NIC.
export GZ_IP="${GZ_IP:-127.0.0.1}"
export ROS_DOMAIN_ID="${NANO_C_SLAM_ROS_DOMAIN_ID:-42}"
export GZ_DISCOVERY_MULTICAST_IP="${GZ_DISCOVERY_MULTICAST_IP:-239.255.0.17}"
export GZ_DISCOVERY_MSG_PORT="${GZ_DISCOVERY_MSG_PORT:-42327}"
export GZ_DISCOVERY_SRV_PORT="${GZ_DISCOVERY_SRV_PORT:-42328}"
failure_truth_odometry_argument=()
if [[ "${NANO_C_SLAM_FAILURE_TRUTH_ODOMETRY:-0}" == "1" ]]; then
    failure_truth_odometry_argument=("failure_truth_odometry:=true")
fi

printf 'Gazebo: %s\n' "$(command -v gz)"
printf 'Gazebo partition: %s\n' "$GZ_PARTITION"
printf 'ROS domain ID: %s\n' "$ROS_DOMAIN_ID"
printf 'Gazebo GUI will join after the server starts.\n'

if [[ -n "$nearfield_world" ]]; then
    uv run --no-project --python "$python311" ros2 launch \
        nano_c_slam_gazebo crazyswarm2_gazebo.launch.py "world:=$nearfield_world" \
        "${failure_truth_odometry_argument[@]}"
else
    uv run --no-project --python "$python311" ros2 launch \
        nano_c_slam_gazebo crazyswarm2_gazebo.launch.py \
        "${failure_truth_odometry_argument[@]}"
fi
