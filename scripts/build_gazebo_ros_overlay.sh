#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace="${CRAZYSWARM2_WS:-$HOME/Work/crazyswarm2_gazebo}"
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
base_paths=("$interface_package" "$gazebo_package")
colcon_package_targets=(nano_c_slam_gazebo)
upstream_gazebo_package="$workspace/src/ros_gz_crazyflie/ros_gz_crazyflie_gazebo"
upstream_bringup_package="$workspace/src/ros_gz_crazyflie/ros_gz_crazyflie_bringup"
upstream_multiranger_root="$workspace/src/crazyflie_ros2_multiranger"
capture_upstream_sources=0

if [[ -d "$upstream_gazebo_package" && -d "$upstream_bringup_package" ]]; then
    base_paths+=("$upstream_gazebo_package" "$upstream_bringup_package")
    capture_upstream_sources=1
elif ! ros2 pkg prefix ros_gz_crazyflie_gazebo >/dev/null 2>&1 || \
    ! ros2 pkg prefix ros_gz_crazyflie_bringup >/dev/null 2>&1; then
    echo "Crazyswarm Gazebo packages are neither installed nor available in $workspace/src." >&2
    exit 1
fi

# The installed Crazyswarm overlay can retain dangling ament-index links after
# its build tree is cleaned. ros_gz_sim discovers every indexed package while
# starting, so rebuild the multiranger package markers in this isolated overlay.
for package in \
    crazyflie_ros2_multiranger_bringup \
    crazyflie_ros2_multiranger_simple_mapper \
    crazyflie_ros2_multiranger_wall_following; do
    package_path="$upstream_multiranger_root/$package"
    if [[ -d "$package_path" ]]; then
        base_paths+=("$package_path")
        colcon_package_targets+=("$package")
    fi
done

capture_source_snapshot() {
    local destination="$1"
    {
        printf 'nano-c-slam-revision: '
        git -C "$repo_root" rev-parse HEAD
        printf 'crazyflie-workspace: %s\n' "$workspace"
        printf 'crazyflie-revision: '
        git -C "$workspace" rev-parse HEAD
        printf '%s\n' 'crazyflie-status:'
        git -C "$workspace" status --porcelain --untracked-files=all
        printf '%s\n' 'crazyflie-submodules:'
        git -C "$workspace" submodule status --recursive
        printf '%s\n' 'nested-submodule-status:'
        # The submodule command evaluates this string in each child repository.
        # shellcheck disable=SC2016
        git -C "$workspace" submodule foreach --recursive \
            'printf "[%s]\\n" "$displaypath"; git status --porcelain --untracked-files=all'
        printf '%s\n' 'source-file-sha256:'
        find "$workspace/src" -type f -not -path '*/.git/*' -print0 \
            | LC_ALL=C sort -z \
            | xargs -0 shasum -a 256
        printf '%s\n' 'source-symlinks:'
        find "$workspace/src" -type l -not -path '*/.git/*' -print \
            | LC_ALL=C sort \
            | while IFS= read -r source_link; do
                printf '%s -> %s\n' "$source_link" "$(readlink "$source_link")"
            done
        printf '%s\n' 'nano-c-slam-build-input-sha256:'
        find "$interface_package" "$gazebo_package" -type f -print0 \
            | LC_ALL=C sort -z \
            | xargs -0 shasum -a 256
    } >"$destination"
}

if [[ "$capture_upstream_sources" -eq 1 ]]; then
    source_snapshot_before="$build_root/source-snapshot.before.txt"
    source_snapshot_after="$build_root/source-snapshot.after.txt"
    capture_source_snapshot "$source_snapshot_before"
fi

printf 'build-command: '
printf '%q ' uv run --no-project --python "$python_executable" colcon build
printf '\npython: %s\ncolcon: %s\n' "$python_executable" "$(command -v colcon)"

colcon_arguments=(
    --log-base "$build_root/log"
    build
    --base-paths "${base_paths[@]}"
    --build-base "$build_root/build"
    --install-base "$build_root/install"
    --packages-up-to "${colcon_package_targets[@]}"
    --event-handlers console_direct+
    --cmake-args
    -DCMAKE_BUILD_TYPE=Release
    "-DPython3_EXECUTABLE=$python_executable"
)

receipt="$build_root/build-receipt.txt"
{
    printf 'schema: nano-c-slam-gazebo-overlay-build-v1\n'
    printf 'python: %s\ncolcon: %s\n' "$python_executable" "$(command -v colcon)"
    printf 'command:'
    printf ' %q' uv run --no-project --python "$python_executable" colcon "${colcon_arguments[@]}"
    printf '\n'
    if [[ "$capture_upstream_sources" -eq 1 ]]; then
        printf 'source_snapshot_sha256: '
        shasum -a 256 "$source_snapshot_before" | awk '{print $1}'
    else
        printf 'source_snapshot_sha256: unavailable (Crazyswarm packages were already installed)\n'
    fi
} >"$receipt"

uv run --no-project --python "$python_executable" colcon "${colcon_arguments[@]}"

if [[ "$capture_upstream_sources" -eq 1 ]]; then
    capture_source_snapshot "$source_snapshot_after"
    if ! cmp -s "$source_snapshot_before" "$source_snapshot_after"; then
        echo "Crazyswarm or Nano-C-SLAM source changed during the isolated overlay build." >&2
        exit 1
    fi
    rm "$source_snapshot_after"
fi

set +u
# shellcheck source=/dev/null
source "$build_root/install/local_setup.bash"
set -u
for package in nano_c_slam_interfaces "${colcon_package_targets[@]}"; do
    package_prefix="$(uv run --no-project --python "$python_executable" ros2 pkg prefix "$package")"
    if [[ "$package_prefix" != "$build_root/install/$package" ]]; then
        echo "Nano-C-SLAM package did not resolve from the new overlay: $package_prefix" >&2
        exit 1
    fi
done
if [[ "$capture_upstream_sources" -eq 1 ]]; then
    for package in ros_gz_crazyflie_gazebo ros_gz_crazyflie_bringup; do
        package_prefix="$(uv run --no-project --python "$python_executable" ros2 pkg prefix "$package")"
        if [[ "$package_prefix" != "$build_root/install/$package" ]]; then
            echo "Crazyswarm Gazebo package did not resolve from the new overlay: $package_prefix" >&2
            exit 1
        fi
    done
    cmp "$upstream_gazebo_package/worlds/crazyflie_swarm_world.sdf" \
        "$build_root/install/ros_gz_crazyflie_gazebo/share/ros_gz_crazyflie_gazebo/worlds/crazyflie_swarm_world.sdf"
    cmp "$upstream_bringup_package/launch/gazebo_crazyswarm2.launch.py" \
        "$build_root/install/ros_gz_crazyflie_bringup/share/ros_gz_crazyflie_bringup/launch/gazebo_crazyswarm2.launch.py"
fi
interface_definition="$(uv run --no-project --python "$python_executable" ros2 interface show nano_c_slam_interfaces/msg/StampedPairwiseRanges)"
if [[ "$interface_definition" != *ranges_m* || "$interface_definition" != *source_stamps* ]]; then
    echo "The built StampedPairwiseRanges interface is missing required source fields." >&2
    exit 1
fi
uv run --no-project --python "$python_executable" ros2 launch \
    nano_c_slam_gazebo crazyswarm2_gazebo.launch.py --show-args >/dev/null

printf 'Nano-C-SLAM ROS overlay built successfully.\n'
if [[ "${#colcon_package_targets[@]}" -gt 1 ]]; then
    printf 'The overlay also rebuilds available Crazyswarm multiranger packages.\n'
fi
printf 'Source it in this shell with:\n  source "%s/install/local_setup.bash"\n' \
    "$build_root"
