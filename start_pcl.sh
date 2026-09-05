#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

set +u
source /opt/ros/humble/setup.bash
source "${SCRIPT_DIR}/livox/install/setup.bash"
source "${SCRIPT_DIR}/slam/install/setup.bash"
set -u

CALIBRATION_FILE="${CALIBRATION_FILE:-${SCRIPT_DIR}/slam/config/lidar_calibration.yaml}"
LIDAR3_TOPIC="${LIDAR3_TOPIC:-livox/lidar_192_168_1_3}"
LIDAR5_TOPIC="${LIDAR5_TOPIC:-livox/lidar_192_168_1_5}"

ros2 run cali_ws pcl_publish \
  --ros-args \
  -p "calibration_file:=${CALIBRATION_FILE}" \
  -p "lidar3_topic:=${LIDAR3_TOPIC}" \
  -p "lidar5_topic:=${LIDAR5_TOPIC}"
