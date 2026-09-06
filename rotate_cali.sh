#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="${SCRIPT_DIR}"
set +u
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/livox/install/setup.bash"
[[ -f "${ROOT_DIR}/odom/install/setup.bash" ]] && source "${ROOT_DIR}/odom/install/setup.bash"
[[ -f "${ROOT_DIR}/slam/install/setup.bash" ]] && source "${ROOT_DIR}/slam/install/setup.bash"
set -u

export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"
LIVOX_CONFIG="${ROOT_DIR}/livox/src/livox_ros_driver2/config/MID360_config_2.json"
POINT_TOPIC="${POINT_TOPIC:-livox/lidar_192_168_1_3}"
IMU_TOPIC="${IMU_TOPIC:-livox/imu_192_168_1_3}"
POSE_TOPIC="${POSE_TOPIC:-pose}"
OUTPUT_FILE="${OUTPUT_FILE:-${ROOT_DIR}/slam/gimbal_lidar.yaml}"

DRIVER_PID=""; ODOM_PID=""; CALI_PID=""
cleanup() {
  local status=$?
  for pid in "${CALI_PID}" "${ODOM_PID}" "${DRIVER_PID}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
    fi
  done
  exit "${status}"
}
trap cleanup EXIT INT TERM

ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -p xfer_format:=0 -p multi_topic:=1 -p data_src:=0 \
  -p publish_freq:=10.0 -p output_data_type:=0 \
  -p frame_id:=livox_frame -p user_config_path:="${LIVOX_CONFIG}" \
  -p cmdline_input_bd_code:=livox0000000001 &
DRIVER_PID=$!
sleep "${DRIVER_STARTUP_WAIT:-2}"

ros2 run single_test single_lidar_odom --ros-args \
  --params-file "${ROOT_DIR}/odom/src/direct_lidar_inertial_odometry/cfg/dlio.yaml" \
  --params-file "${ROOT_DIR}/odom/src/direct_lidar_inertial_odometry/cfg/params.yaml" \
  -r "pointcloud:=${POINT_TOPIC}" -r "imu:=${IMU_TOPIC}" &
ODOM_PID=$!
sleep "${ODOM_STARTUP_WAIT:-3}"

ros2 run cali_ws circle_cali_dlio --ros-args \
  -p pose_topic:="${POSE_TOPIC}" -p output_file:="${OUTPUT_FILE}" &
CALI_PID=$!
wait "${CALI_PID}"
