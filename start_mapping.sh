#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="${SCRIPT_DIR}"

# Mapping is now part of start_odom.sh: DLIO supplies odometry, while the
# GTSAM backend publishes a loop-corrected map and trajectory.
exec "${ROOT_DIR}/start_odom.sh" "$@"
