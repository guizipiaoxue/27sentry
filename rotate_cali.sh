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

if ! ros2 pkg prefix cali_ws >/dev/null 2>&1 || [[ ! -e "${ROOT_DIR}/slam/install/cali_ws/lib/cali_ws/circle_cali_dlio" ]]; then
  echo "[rotate_cali] circle_cali_dlio is not built. Build it first:" >&2
  echo "  cd ${ROOT_DIR}/slam && source /opt/ros/humble/setup.bash && colcon build --packages-select cali_ws --symlink-install" >&2
  exit 1
fi

export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"
LIVOX_CONFIG="${ROOT_DIR}/livox/src/livox_ros_driver2/config/MID360_config_2.json"
POINT_TOPIC="${POINT_TOPIC:-livox/lidar_192_168_1_3}"
IMU_TOPIC="${IMU_TOPIC:-livox/imu_192_168_1_3}"
POSE_TOPIC="${POSE_TOPIC:-pose}"
OUTPUT_FILE="${OUTPUT_FILE:-${ROOT_DIR}/slam/gimbal_lidar.yaml}"
# Circle fitting / motion-continuity filters.  Keep these configurable from the
# shell because the appropriate limits depend on the platform's motion speed.
MIN_SAMPLES="${MIN_SAMPLES:-100}"
MAX_POSITION_JUMP="${MAX_POSITION_JUMP:-0.50}"
MAX_ANGULAR_JUMP="${MAX_ANGULAR_JUMP:-0.35}"
MAX_LINEAR_SPEED="${MAX_LINEAR_SPEED:-5.0}"
MAX_ANGULAR_SPEED="${MAX_ANGULAR_SPEED:-6.0}"
RESIDUAL_THRESHOLD="${RESIDUAL_THRESHOLD:-0.05}"
FIT_WINDOW="${FIT_WINDOW:-2000}"

DRIVER_PID=""; ODOM_PID=""; CALI_PID=""
cleanup() {
  local status=$?
  trap - EXIT INT TERM
  for pid in "${CALI_PID}" "${ODOM_PID}" "${DRIVER_PID}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
    fi
  done
  pkill -TERM -f 'circle_cali_dlio|single_lidar_odom|livox_ros_driver2_node' 2>/dev/null || true
  sleep 1
  for pid in "${CALI_PID}" "${ODOM_PID}" "${DRIVER_PID}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -KILL -- "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
    fi
  done
  pkill -KILL -f 'circle_cali_dlio|single_lidar_odom|livox_ros_driver2_node' 2>/dev/null || true
  wait 2>/dev/null || true
  exit "${status}"
}
trap cleanup EXIT INT TERM

setsid ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -p xfer_format:=0 -p multi_topic:=1 -p data_src:=0 \
  -p publish_freq:=10.0 -p output_data_type:=0 \
  -p frame_id:=livox_frame -p user_config_path:="${LIVOX_CONFIG}" \
  -p cmdline_input_bd_code:=livox0000000001 &
DRIVER_PID=$!
sleep "${DRIVER_STARTUP_WAIT:-2}"

setsid ros2 run single_test single_lidar_odom --ros-args \
  --params-file "${ROOT_DIR}/odom/src/direct_lidar_inertial_odometry/cfg/dlio.yaml" \
  --params-file "${ROOT_DIR}/odom/src/direct_lidar_inertial_odometry/cfg/params.yaml" \
  -r "pointcloud:=${POINT_TOPIC}" -r "imu:=${IMU_TOPIC}" \
  >"${ROOT_DIR}/dlio_odom.log" 2>&1 &
ODOM_PID=$!
sleep "${ODOM_STARTUP_WAIT:-3}"

setsid ros2 run cali_ws circle_cali_dlio --ros-args \
  -p pose_topic:="${POSE_TOPIC}" \
  -p output_file:="${OUTPUT_FILE}" \
  -p min_samples:="${MIN_SAMPLES}" \
  -p max_position_jump:="${MAX_POSITION_JUMP}" \
  -p max_angular_jump:="${MAX_ANGULAR_JUMP}" \
  -p max_linear_speed:="${MAX_LINEAR_SPEED}" \
  -p max_angular_speed:="${MAX_ANGULAR_SPEED}" \
  -p residual_threshold:="${RESIDUAL_THRESHOLD}" \
  -p fit_window:="${FIT_WINDOW}" &
CALI_PID=$!
wait "${CALI_PID}"
