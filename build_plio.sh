#!/usr/bin/env bash
# Rebuild both affected packages so an old installed fusion cannot zero times.
set -Eeuo pipefail
ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
case "${1:-}" in
  ""|--start) ;;
  -h|--help)
    echo "Usage: bash build_plio.sh [--start [start_odom.sh options]]"
    echo "--start reuses start_odom.sh SDK discovery and diagnostic rosbag recording."
    exit 0 ;;
  *) echo "Unknown option: $1" >&2; exit 2 ;;
esac
set +u
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/livox/install/setup.bash"
set -u
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
(
  cd "${ROOT_DIR}/slam"
  colcon build --packages-select fusion_ws --parallel-workers 1 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
)
(
  cd "${ROOT_DIR}/odom"
  colcon build --packages-select plio --parallel-workers 1 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
  ctest --test-dir build/plio \
    -R '^(static_imu|imu_prediction|lidar_covariance)_test$' --output-on-failure
)
echo "PLIO and fusion built; validated calibration installed."
if [[ "${1:-}" == --start ]]; then
  shift
  exec bash "${ROOT_DIR}/start_odom.sh" -a plio "$@"
fi
