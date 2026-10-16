#!/bin/bash
set -euo pipefail

ros_prefix="${ROS2_PREFIX:-/opt/homebrew/opt/ros2-jazzy}"
python311="/Library/Frameworks/Python.framework/Versions/3.11/bin/python3.11"
workspace="${CRAZYSWARM2_WS:-$HOME/Work/crazyswarm2_gazebo}"
firmware_build="${CRAZYFLIE_FIRMWARE_BUILD:-$HOME/Work/Nano-C-SLAM/crazyflie-firmware/build}"
homebrew_prefix="$(brew --prefix)"
abseil_opt="$homebrew_prefix/opt/abseil"
abseil_old="$homebrew_prefix/opt/abseil@20260107.0"
ffmpeg8="$homebrew_prefix/opt/ffmpeg@8/lib"
ogre23="$homebrew_prefix/opt/ogre2.3/lib"

if [[ ! -x "$python311" || ! -f "$ros_prefix/setup.bash" ]]; then
    echo "Install Python.org 3.11.9 and Homebrew ROS 2 Jazzy first." >&2
    exit 1
fi
if [[ ! -f "$workspace/install/local_setup.bash" ]]; then
    echo "Build the Crazyswarm2 workspace first: $workspace" >&2
    exit 1
fi
if [[ ! -d "$abseil_old/lib" || ! -d "$ffmpeg8" || ! -d "$ogre23" ]]; then
    echo "Install the ROS compatibility dependencies; see sim/gazebo/README.md." >&2
    exit 1
fi

temporary_dir="$(mktemp -d "${TMPDIR:-/tmp}/nano-c-slam-ros2.XXXXXX")"
abseil_original_target="$(readlink "$abseil_opt")"
declare -a compat_links=()

cleanup() {
    ln -sfn "$abseil_original_target" "$abseil_opt"
    for link in "${compat_links[@]:-}"; do
        if [[ -L "$link" ]]; then
            unlink "$link"
        fi
    done
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

add_compat_link() {
    local destination="$1"
    local target="$2"
    if [[ ! -e "$destination" && ! -L "$destination" ]]; then
        ln -s "$target" "$destination"
        compat_links+=("$destination")
    fi
}

for library in libavdevice.62.dylib libavformat.62.dylib libavcodec.62.dylib \
    libavutil.60.dylib libswscale.9.dylib; do
    add_compat_link "$homebrew_prefix/opt/ffmpeg/lib/$library" "$ffmpeg8/$library"
done
for library in libOgreMain.2.3.1.dylib libOgreHlmsPbs.2.3.1.dylib \
    libOgreHlmsUnlit.2.3.1.dylib libOgreOverlay.2.3.1.dylib \
    libOgrePlanarReflections.2.3.1.dylib; do
    add_compat_link "$ogre23/$library" "$ogre23/${library/.2.3.1./.2.3.3.}"
done

export PATH="/Library/Frameworks/Python.framework/Versions/3.11/bin:$ros_prefix/bin:$PATH"
set +u
# shellcheck source=/dev/null
source "$ros_prefix/setup.bash"
# shellcheck source=/dev/null
source "$workspace/install/local_setup.bash"
set -u
export PYTHONPATH="$firmware_build${PYTHONPATH:+:$PYTHONPATH}"
export GZ_CONFIG_PATH="$ros_prefix/share/gz"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ros_prefix/lib/gz-sim-8/plugins:$ros_prefix/lib"
export GZ_SIM_PHYSICS_ENGINE_PATH="$ros_prefix/lib/gz-physics-7/engine-plugins"
gazebo_models="$(uv run --no-project --python "$python311" ros2 pkg prefix ros_gz_crazyflie_gazebo)/share/ros_gz_crazyflie_gazebo/models"
export GZ_SIM_RESOURCE_PATH="$gazebo_models:$ros_prefix/share/gz/gz-sim8/worlds"
export GZ_RENDERING_PLUGIN_PATH="$ros_prefix/lib/gz-rendering-8/engine-plugins"
export GZ_RENDERING_RESOURCE_PATH="$ros_prefix/share/gz/gz-rendering8"
export GZ_GUI_PLUGIN_PATH="$ros_prefix/lib/gz-sim-8/plugins/gui:$ros_prefix/lib/gz-gui-8/plugins"
export QML2_IMPORT_PATH="$ros_prefix/lib/gz-sim-8/plugins/gui"

uv run --no-project --python "$python311" ros2 launch \
    nano_c_slam_gazebo crazyswarm2_gazebo.launch.py
