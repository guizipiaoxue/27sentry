#!/usr/bin/env bash
# 一键启动：Livox 驱动、单雷达 DLIO 里程计和圆周标定节点。
set -Eeuo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
LIVOX_WS="${SCRIPT_DIR}/livox"; ODOM_WS="${SCRIPT_DIR}/odom"; CALI_WS="${SCRIPT_DIR}/slam"
LIDAR_TOPIC="${LIDAR_TOPIC:-livox/lidar_192_168_1_3}"; IMU_TOPIC="${IMU_TOPIC:-livox/imu_192_168_1_3}"
POSE_TOPIC="${POSE_TOPIC:-pose}"; OUTPUT_FILE="${OUTPUT_FILE:-${CALI_WS}/gimbal_lidar.yaml}"
for f in "${ROS_SETUP}" "${LIVOX_WS}/install/setup.bash" "${ODOM_WS}/install/setup.bash" "${CALI_WS}/install/setup.bash"; do
  [[ -f "${f}" ]] || { echo "[rotate_cali] missing: ${f}" >&2; exit 1; }
done
set +u; source "${ROS_SETUP}"; source "${LIVOX_WS}/install/setup.bash"; source "${ODOM_WS}/install/setup.bash"; source "${CALI_WS}/install/setup.bash"; set -u
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"
DRIVER_PID=""; ODOM_PID=""; CALI_PID=""
cleanup() { local s=$?; for p in "${CALI_PID}" "${ODOM_PID}" "${DRIVER_PID}"; do [[ -n "${p}" ]] && kill -0 "${p}" 2>/dev/null && { kill "${p}" 2>/dev/null || true; wait "${p}" 2>/dev/null || true; }; done; exit "${s}"; }
trap cleanup EXIT INT TERM
ros2 launch livox_ros_driver2 msg_MID360_launch.py & DRIVER_PID=$!; sleep "${DRIVER_STARTUP_WAIT:-2}"
ros2 run single_test single_lidar_odom --ros-args --params-file "${ODOM_WS}/src/direct_lidar_inertial_odometry/cfg/dlio.yaml" --params-file "${ODOM_WS}/src/direct_lidar_inertial_odometry/cfg/params.yaml" -r "pointcloud:=${LIDAR_TOPIC}" -r "imu:=${IMU_TOPIC}" & ODOM_PID=$!; sleep "${ODOM_STARTUP_WAIT:-3}"
ros2 run cali_ws circle_cali_dlio --ros-args -p pose_topic:="${POSE_TOPIC}" -p output_file:="${OUTPUT_FILE}" & CALI_PID=$!
wait "${CALI_PID}"
