#!/usr/bin/env bash
set -Eeuo pipefail
ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
case "${1:-}" in
  ""|--start) ;;
  -h|--help)
    echo 'Usage: bash build_dlio.sh [--start [start_odom.sh options]]'
    echo 'Build/test DLIO and fusion; --start reuses SDK discovery and rosbag recording.'
    exit 0 ;;
  *) echo "Unknown option: $1" >&2; exit 2 ;;
esac
set +u
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/livox/install/setup.bash"
if [[ -f "${ROOT_DIR}/odom/install/setup.bash" ]]; then
  source "${ROOT_DIR}/odom/install/setup.bash"
fi
set -u
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
export MAKEFLAGS="${MAKEFLAGS:--j2}"
(
  cd "${ROOT_DIR}/slam"
  colcon build --packages-select fusion_ws --parallel-workers 1 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
)
(
  cd "${ROOT_DIR}/odom"
  # A fresh workspace also needs the loop_closure message interface package.
  if [[ -f install/loop_closure/share/loop_closure/cmake/loop_closureConfig.cmake ]]; then
    selection=(--packages-select odom_ws)
  else
    selection=(--packages-up-to odom_ws)
  fi
  colcon build "${selection[@]}" --parallel-workers 1 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
  ctest --test-dir build/odom_ws -R '^dlio_motion_test$' --output-on-failure
)
echo 'DLIO and fusion built; motion regression tests passed.'
if [[ "${1:-}" == --start ]]; then
  shift
  exec bash "${ROOT_DIR}/start_odom.sh" -a dlio "$@"
fi
