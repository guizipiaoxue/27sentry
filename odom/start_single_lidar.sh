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

# Livox SDK 固定安装路径
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"

# MID360 3 号雷达配置
LIVOX_CONFIG="${ROOT_DIR}/livox/src/livox_ros_driver2/config/MID360_config_2.json"
POINT_TOPIC="livox/lidar_192_168_1_3"
IMU_TOPIC="livox/imu_192_168_1_3"

ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -p xfer_format:=0 \
  -p multi_topic:=1 \
  -p data_src:=0 \
  -p publish_freq:=10.0 \
  -p output_data_type:=0 \
  -p frame_id:=livox_frame \
  -p user_config_path:="${LIVOX_CONFIG}" \
  -p cmdline_input_bd_code:=livox0000000001 &

LIVOX_PID=$!
trap 'kill "${LIVOX_PID}" 2>/dev/null || true' EXIT INT TERM
sleep 2

ros2 run single_test single_lidar_odom --ros-args \
  --params-file "${ROOT_DIR}/odom/src/direct_lidar_inertial_odometry/cfg/dlio.yaml" \
  --params-file "${ROOT_DIR}/odom/src/direct_lidar_inertial_odometry/cfg/params.yaml" \
  -r "pointcloud:=${POINT_TOPIC}" \
  -r "imu:=${IMU_TOPIC}"
