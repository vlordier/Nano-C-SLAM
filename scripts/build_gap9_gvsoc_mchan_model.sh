#!/usr/bin/env bash
set -euo pipefail

SDK_ROOT="${GAP9_SDK_ROOT:-$HOME/Work/da3_gap9/third_party/gap_sdk_private}"
SDK_MCHAN_SOURCE="gvsoc/gvsoc_gap/gap/mchan/mchan_v7_impl.cpp"
SDK_COMMIT="${GAP9_GVSOC_MCHAN_SOURCE_REVISION:-45247566dbfd1ec7bb021f158fefe133c2aa8e10}"
if [[ ! "$SDK_COMMIT" =~ ^[0-9a-f]{40}$ ]]; then
    echo "GAP9_GVSOC_MCHAN_SOURCE_REVISION must be a full lowercase commit hash" >&2
    exit 2
fi
SDK_CHECKOUT="$(git -C "$SDK_ROOT" rev-parse HEAD)"
SDK_STATUS="$(git -C "$SDK_ROOT" status --porcelain --untracked-files=all)"
BUILD_ROOT="${GAP9_GVSOC_MODEL_BUILD_ROOT:-${TMPDIR:-/tmp}/nano-c-slam-gvsoc-model-$SDK_COMMIT}"
MODEL_ROOT="${GAP9_GVSOC_MODEL_ROOT:-$BUILD_ROOT/models}"
INSTALLED_MODELS="$SDK_ROOT/install/workstation/models"
ENGINE_LIBRARY="$SDK_ROOT/build_gvsoc/engine/libpulpvp.dylib"
CXX_PATH="$(command -v "${CXX:-c++}")"

for required in \
    "$SDK_ROOT/.git" \
    "$INSTALLED_MODELS/gap9/soc.dylib" \
    "$ENGINE_LIBRARY"; do
    [[ -e "$required" ]] || {
        echo "missing GAP9 GVSOC model prerequisite: $required" >&2
        exit 2
    }
done

if ! git -C "$SDK_ROOT" cat-file -e "$SDK_COMMIT:$SDK_MCHAN_SOURCE"; then
    echo "pinned MCHAN source revision is unavailable in the SDK checkout: $SDK_COMMIT" >&2
    exit 2
fi

mkdir -p "$MODEL_ROOT"
MODEL_ROOT="$(cd "$MODEL_ROOT" && pwd -P)"
INSTALLED_MODELS="$(cd "$INSTALLED_MODELS" && pwd -P)"
if [[ "$MODEL_ROOT" == "$INSTALLED_MODELS" ||
      "$MODEL_ROOT" == "$INSTALLED_MODELS/"* ]]; then
    echo "refusing to replace or write inside the installed SDK model directory" >&2
    exit 2
fi

mkdir -p "$BUILD_ROOT"
git -C "$SDK_ROOT" show "$SDK_COMMIT:$SDK_MCHAN_SOURCE" \
    > "$BUILD_ROOT/mchan_v7_impl.cpp"

"$CXX_PATH" -arch arm64 -fPIC -fno-stack-protector -D__GVSOC__ \
    -I"$SDK_ROOT/gvsoc/gvsoc_gap" \
    -I"$SDK_ROOT/gvsoc/gvsoc_gap/gap/mchan" \
    -I"$SDK_ROOT/gvsoc/gvsoc/engine/include" \
    -c "$BUILD_ROOT/mchan_v7_impl.cpp" \
    -o "$BUILD_ROOT/mchan_v7_impl.o"
"$CXX_PATH" -arch arm64 -bundle -Wl,-headerpad_max_install_names \
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
sdk_source_commit=$SDK_COMMIT
sdk_checkout_commit=$SDK_CHECKOUT
sdk_worktree_dirty=$([[ -n "$SDK_STATUS" ]] && echo true || echo false)
mchan_source_sha256=$(shasum -a 256 "$BUILD_ROOT/mchan_v7_impl.cpp" | awk '{print $1}')
compiler_path=$CXX_PATH
compiler_version=$("$CXX_PATH" --version | sed -n '1p')
compiler_sha256=$(shasum -a 256 "$CXX_PATH" | awk '{print $1}')
engine_sha256=$(shasum -a 256 "$ENGINE_LIBRARY" | awk '{print $1}')
soc_model_sha256=$(shasum -a 256 "$INSTALLED_MODELS/gap9/soc.dylib" | awk '{print $1}')
plugin_sha256=$(shasum -a 256 "$MODEL_ROOT/gap/mchan/mchan_v7_impl.dylib" | awk '{print $1}')
model_root=$MODEL_ROOT
EOF

cat "$BUILD_ROOT/model-provenance.txt"
echo "GAP9_GVSOC_MODEL_ROOT=$MODEL_ROOT"
