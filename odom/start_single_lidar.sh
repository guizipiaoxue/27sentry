#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

# ROS 2 与本项目工作空间
set +u
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/livox/install/setup.bash"
[[ -f "${ROOT_DIR}/odom/install/setup.bash" ]] && source "${ROOT_DIR}/odom/install/setup.bash"
set -u

# 优先使用隔离 SDK，独立库名避免误加载系统 SDK。
export LD_LIBRARY_PATH="${SDK_DIR:-${ROOT_DIR}/livox/src/livox_ros_driver2/.livox_sdk/lib}:/usr/local/lib:${LD_LIBRARY_PATH:-}"

# MID360 3 号雷达配置
LIVOX_CONFIG="${LIVOX_CONFIG:-${ROOT_DIR}/livox/src/livox_ros_driver2/config/MID360_config_2.json}"
POINT_TOPIC="livox/lidar_192_168_1_3"
IMU_TOPIC="livox/imu_192_168_1_3"

source "${ROOT_DIR}/scripts/ptp_runtime.sh"
ptp_check_host "${ROOT_DIR}" "${LIVOX_CONFIG}"

ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -p xfer_format:=0 \
  -p multi_topic:=1 \
  -p data_src:=0 \
  -p publish_freq:=10.0 \
  -p output_data_type:=0 \
  -p frame_id:=livox_frame \
  -p require_ptp_sync:=true \
  -p "ptp_utc_offset_seconds:=${PTP_UTC_OFFSET}" \
  -p use_sim_time:=false \
  -p user_config_path:="${LIVOX_CONFIG}" \
  -p cmdline_input_bd_code:=livox0000000001 &

LIVOX_PID=$!
trap 'kill "${LIVOX_PID}" 2>/dev/null || true' EXIT INT TERM
sleep 2
ptp_wait_sensors

ros2 run single_test single_lidar_odom --ros-args \
  --params-file "${ROOT_DIR}/odom/src/direct_lidar_inertial_odometry/cfg/dlio.yaml" \
  --params-file "${ROOT_DIR}/odom/src/direct_lidar_inertial_odometry/cfg/params.yaml" \
  -p odom/computeTimeOffset:=false \
  -p use_sim_time:=false \
  -r "pointcloud:=${POINT_TOPIC}" \
  -r "imu:=${IMU_TOPIC}"
