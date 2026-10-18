#!/usr/bin/env bash
set -euo pipefail

SDK_ROOT="${GAP9_SDK_ROOT:-$HOME/Work/da3_gap9/third_party/gap_sdk_private}"
SDK_MCHAN_SOURCE="gvsoc/gvsoc_gap/gap/mchan/mchan_v7_impl.cpp"
SDK_COMMIT="$(git -C "$SDK_ROOT" rev-parse HEAD)"
BUILD_ROOT="${GAP9_GVSOC_MODEL_BUILD_ROOT:-${TMPDIR:-/tmp}/nano-c-slam-gvsoc-model-$SDK_COMMIT}"
MODEL_ROOT="${GAP9_GVSOC_MODEL_ROOT:-$BUILD_ROOT/models}"
INSTALLED_MODELS="$SDK_ROOT/install/workstation/models"
ENGINE_LIBRARY="$SDK_ROOT/build_gvsoc/engine/libpulpvp.dylib"

for required in \
    "$SDK_ROOT/.git" \
    "$INSTALLED_MODELS/gap9/soc.dylib" \
    "$ENGINE_LIBRARY"; do
    [[ -e "$required" ]] || {
        echo "missing GAP9 GVSOC model prerequisite: $required" >&2
        exit 2
    }
done

if [[ "$MODEL_ROOT" -ef "$INSTALLED_MODELS" ]]; then
    echo "refusing to replace the installed SDK model directory" >&2
    exit 2
fi

mkdir -p "$BUILD_ROOT"
git -C "$SDK_ROOT" show "$SDK_COMMIT:$SDK_MCHAN_SOURCE" \
    > "$BUILD_ROOT/mchan_v7_impl.cpp"

"${CXX:-c++}" -arch arm64 -fPIC -fno-stack-protector -D__GVSOC__ \
    -I"$SDK_ROOT/gvsoc/gvsoc_gap" \
    -I"$SDK_ROOT/gvsoc/gvsoc_gap/gap/mchan" \
    -I"$SDK_ROOT/gvsoc/gvsoc/engine/include" \
    -c "$BUILD_ROOT/mchan_v7_impl.cpp" \
    -o "$BUILD_ROOT/mchan_v7_impl.o"
"${CXX:-c++}" -arch arm64 -bundle -Wl,-headerpad_max_install_names \
    -Xlinker -undefined -Xlinker dynamic_lookup \
    -o "$BUILD_ROOT/mchan_v7_impl.dylib" \
    "$BUILD_ROOT/mchan_v7_impl.o" \
    -Wl,-rpath,"$SDK_ROOT/build_gvsoc/engine" "$ENGINE_LIBRARY" \
    -lz -lpthread

if [[ ! -e "$MODEL_ROOT/gap9/soc.dylib" ]]; then
    mkdir -p "$MODEL_ROOT"
    ditto "$INSTALLED_MODELS" "$MODEL_ROOT"
fi
mkdir -p "$MODEL_ROOT/gap/mchan"
cp "$BUILD_ROOT/mchan_v7_impl.dylib" \
    "$MODEL_ROOT/gap/mchan/mchan_v7_impl.dylib"

cat > "$BUILD_ROOT/model-provenance.txt" <<EOF
sdk_commit=$SDK_COMMIT
mchan_source_sha256=$(shasum -a 256 "$BUILD_ROOT/mchan_v7_impl.cpp" | awk '{print $1}')
compiler=$("${CXX:-c++}" --version | sed -n '1p')
model_root=$MODEL_ROOT
EOF

cat "$BUILD_ROOT/model-provenance.txt"
echo "GAP9_GVSOC_MODEL_ROOT=$MODEL_ROOT"
