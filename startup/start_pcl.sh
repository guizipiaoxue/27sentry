#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

set +u
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/livox/install/setup.bash"
source "${ROOT_DIR}/slam/install/setup.bash"
set -u

# Livox ROS Driver 2 运行时依赖动态库；编译时链接成功不代表运行时能找到它。
# 可通过 SDK_DIR 显式指定安装目录，否则按常见位置查找。
SDK_DIRS=(
  "${SDK_DIR:-}"
  "${ROOT_DIR}/livox/src/livox_ros_driver2/.livox_sdk/lib"
  "${ROOT_DIR}/livox/install/livox_ros_driver2/lib"
  "/usr/local/lib"
  "${ROOT_DIR}/../Livox-SDK2/build/sdk_core"
)
SDK_FOUND=""
for dir in "${SDK_DIRS[@]}"; do
  if [[ -n "${dir}" && -f "${dir}/liblivox_lidar_sdk_sentry.so" ]]; then
    SDK_FOUND="${dir}"
    break
  fi
done
if [[ -z "${SDK_FOUND}" ]]; then
  echo "[start_pcl] liblivox_lidar_sdk_sentry.so not found." >&2
  echo "           Run bash livox/build_isolated_sdk.sh first (or set SDK_DIR to an isolated SDK)." >&2
  exit 1
fi
export LD_LIBRARY_PATH="${SDK_FOUND}:${LD_LIBRARY_PATH:-}"

CALIBRATION_FILE="${CALIBRATION_FILE:-${ROOT_DIR}/slam/config/lidar_calibration.yaml}"
LIDAR3_TOPIC="${LIDAR3_TOPIC:-livox/lidar_192_168_1_3}"
LIDAR5_TOPIC="${LIDAR5_TOPIC:-livox/lidar_192_168_1_5}"

source "${ROOT_DIR}/scripts/ptp_runtime.sh"
ptp_check_host "${ROOT_DIR}" "${LIVOX_CONFIG:-${ROOT_DIR}/livox/src/livox_ros_driver2/config/MID360_config_2.json}"
ptp_wait_sensors

ros2 run cali_ws pcl_publish \
  --ros-args \
  -p "calibration_file:=${CALIBRATION_FILE}" \
  -p "lidar3_topic:=${LIDAR3_TOPIC}" \
  -p "lidar5_topic:=${LIDAR5_TOPIC}"
