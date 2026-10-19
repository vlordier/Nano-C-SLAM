#!/usr/bin/env bash
set -euo pipefail

if [[ $# != 2 ]]; then
    echo "usage: link_gvsoc_workdir.sh ABSOLUTE_WORK_ROOT ABSOLUTE_LINK_PATH" >&2
    exit 2
fi

work_root="$1"
link_path="$2"
if [[ "$work_root" != /* || "$link_path" != /* || "$work_root" == "$link_path" ]]; then
    echo "work root and link path must be distinct absolute paths" >&2
    exit 2
fi

mkdir -p "$work_root" "$(dirname "$link_path")"
ln -sfn "$work_root" "$link_path"
