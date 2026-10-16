#!/usr/bin/env bash
set -euo pipefail

: "${GAP9_REAL_GVSOC_LAUNCHER:?GAP9_REAL_GVSOC_LAUNCHER is required}"
exec script -q /dev/null "$GAP9_REAL_GVSOC_LAUNCHER" "$@"
