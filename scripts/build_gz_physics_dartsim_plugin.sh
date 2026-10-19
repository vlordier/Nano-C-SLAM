#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
gz_root="${GZ_MACOS_ROOT:-$HOME/Work/gz-macOS}"
physics_source="$gz_root/gz-physics"
gz_install="$gz_root/ws/install"
dartsim_prefix="$(brew --prefix dartsim)"
bullet_prefix="$(brew --prefix bullet)"
build_root="$repo_root/build/gz-physics-dartsim-rebuild"
install_root="$build_root/install"

if [[ ! -f "$physics_source/CMakeLists.txt" || ! -f "$gz_install/setup.bash" ]]; then
    echo "Expected gz-physics source and installed Gazebo workspace under $gz_root." >&2
    exit 1
fi
dart_cmake_dir="$dartsim_prefix/share/dart/cmake"
if [[ ! -f "$dart_cmake_dir/DARTConfig.cmake" ]]; then
    echo "DART CMake package was not found under $dartsim_prefix." >&2
    exit 1
fi

mkdir -p "$repo_root/build"
cmake -S "$physics_source" -B "$build_root" \
    -DCMAKE_BUILD_TYPE=Release \
    "-DCMAKE_INSTALL_PREFIX=$install_root" \
    "-DCMAKE_PREFIX_PATH=$gz_install;$dartsim_prefix;$bullet_prefix" \
    -DBUILD_TESTING=OFF \
    "-DDART_DIR=$dart_cmake_dir"
cmake --build "$build_root" --target gz-physics7-dartsim-plugin --parallel 4

plugin="$build_root/lib/libgz-physics7-dartsim-plugin.7.8.0.dylib"
physics_library="$build_root/lib/libgz-physics7.7.8.0.dylib"
custom_engine_directory="$build_root/custom"
custom_engine="$custom_engine_directory/libnano-c-slam-dartsim-plugin.dylib"
if [[ ! -f "$plugin" || ! -f "$physics_library" ]]; then
    echo "The rebuilt Gazebo DART plugin or its physics library is missing." >&2
    exit 1
fi
mkdir -p "$custom_engine_directory"
cp "$plugin" "$custom_engine"

receipt="$build_root/build-receipt.txt"
{
    printf 'schema: nano-c-slam-gz-dartsim-build-v1\n'
    printf 'source: %s\n' "$physics_source"
    printf 'source_revision: %s\n' "$(git -C "$physics_source" rev-parse HEAD)"
    printf 'source_status:\n'
    git -C "$physics_source" status --porcelain --untracked-files=all
    printf 'dartsim_prefix: %s\n' "$dartsim_prefix"
    printf 'dartsim_version: %s\n' "$(brew list --versions dartsim)"
    printf 'cmake_version: %s\n' "$(cmake --version | head -n 1)"
    printf 'plugin_sha256: %s\n' "$(shasum -a 256 "$plugin" | awk '{print $1}')"
    printf 'custom_engine_sha256: %s\n' \
        "$(shasum -a 256 "$custom_engine" | awk '{print $1}')"
    printf 'physics_library_sha256: %s\n' \
        "$(shasum -a 256 "$physics_library" | awk '{print $1}')"
    printf 'engine_path: %s/lib\n' "$build_root"
} >"$receipt"

printf 'Rebuilt Gazebo DART engine plugin against Homebrew DART.\n'
printf 'Receipt: %s\n' "$receipt"
printf 'For the Nano-C-SLAM launcher, set:\n'
printf '  NANO_C_SLAM_GZ_PHYSICS_ENGINE_PATH="%s/custom:%s/lib/gz-physics-7/engine-plugins"\n' \
    "$build_root" "$gz_install"
printf '  NANO_C_SLAM_GZ_PHYSICS_ENGINE="nano-c-slam-dartsim-plugin"\n'
