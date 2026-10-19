#!/usr/bin/env bash
set -euo pipefail

abseil_opt="${1:?usage: check_gazebo_abseil_opt.sh ABSEIL_OPT_PATH}"
if [[ ! -d "$abseil_opt" ]]; then
    exit 0
fi

resolved_abseil_path="$(cd -P "$abseil_opt" && pwd -P)"
case "$resolved_abseil_path" in
    */nano-c-slam-ros2.*/abseil)
        echo "Another Nano-C-SLAM Gazebo launch is using the shared Abseil compatibility link: $abseil_opt" >&2
        echo "Wait for that launch to exit before starting another Gazebo world." >&2
        exit 1
        ;;
esac
