#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="${SCRIPT_DIR}"

usage() {
  cat <<'EOF'
Usage: ./rotate_cali.sh [OPTIONS]

Calibrate both MID360-to-gimbal rotations from two independent DLIO yaw
trajectories. The configured XY translations are treated as known mounting
measurements; both Z translations keep the same height.

Options:
  --bag PATH       Replay a PointCloud2 rosbag instead of starting the driver
  --rate RATE      Rosbag playback rate (default: 1.0)
  --record-bag     Record raw sensors and both DLIO poses (live default)
  --no-record-bag  Disable diagnostic recording
  --bag-output DIR Set the diagnostic rosbag output directory
  -h, --help       Show this help

Useful environment variables:
  SDK_DIR, LIVOX_CONFIG, LIDAR5_TOPIC, LIDAR3_TOPIC, IMU5_TOPIC, IMU3_TOPIC
  LIDAR5_OUTPUT, LIDAR3_OUTPUT, MIN_DIRECTION_DEG, MAX_CIRCLE_RESIDUAL

Live use needs no arguments:
  ./rotate_cali.sh
EOF
}

BAG_PATH="${BAG_PATH:-}"
BAG_RATE="${BAG_RATE:-1.0}"
RECORD_ROSBAG="${RECORD_ROSBAG:-}"
ROSBAG_OUTPUT="${ROSBAG_OUTPUT:-}"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag)
      if [[ $# -lt 2 ]]; then
        echo "[rotate_cali] --bag requires a path." >&2
        exit 2
      fi
      BAG_PATH="$2"
      shift 2
      ;;
    --bag=*)
      BAG_PATH="${1#*=}"
      shift
      ;;
    --rate)
      if [[ $# -lt 2 ]]; then
        echo "[rotate_cali] --rate requires a value." >&2
        exit 2
      fi
      BAG_RATE="$2"
      shift 2
      ;;
    --rate=*)
      BAG_RATE="${1#*=}"
      shift
      ;;
    --record-bag)
      RECORD_ROSBAG=1
      shift
      ;;
    --no-record-bag)
      RECORD_ROSBAG=0
      shift
      ;;
    --bag-output)
      if [[ $# -lt 2 ]]; then
        echo "[rotate_cali] --bag-output requires a path." >&2
        exit 2
      fi
      ROSBAG_OUTPUT="$2"
      shift 2
      ;;
    --bag-output=*)
      ROSBAG_OUTPUT="${1#*=}"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[rotate_cali] Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

ROS_SETUP="/opt/ros/humble/setup.bash"
LIVOX_SETUP="${ROOT_DIR}/livox/install/setup.bash"
ODOM_SETUP="${ROOT_DIR}/odom/install/setup.bash"
SLAM_SETUP="${ROOT_DIR}/slam/install/setup.bash"
for setup_file in "${ROS_SETUP}" "${LIVOX_SETUP}" "${ODOM_SETUP}" "${SLAM_SETUP}"; do
  if [[ ! -f "${setup_file}" ]]; then
    echo "[rotate_cali] Missing setup file: ${setup_file}" >&2
    echo "[rotate_cali] Build the corresponding workspace first." >&2
    exit 1
  fi
done

set +u
source "${ROS_SETUP}"
source "${LIVOX_SETUP}"
source "${ODOM_SETUP}"
source "${SLAM_SETUP}"
set -u

# Match start_odom.sh: allow an explicit SDK_DIR, then check the known source
# tree and local driver fallback in addition to /usr/local/lib.
SDK_DIRS=(
  "${SDK_DIR:-}"
  "/usr/local/lib"
  "${ROOT_DIR}/../Livox-SDK2/build/sdk_core"
  "${ROOT_DIR}/livox/src/livox_ros_driver2/.livox_sdk/lib"
)
SDK_FOUND=""
for sdk_dir in "${SDK_DIRS[@]}"; do
  if [[ -n "${sdk_dir}" && -f "${sdk_dir}/liblivox_lidar_sdk_shared.so" ]]; then
    SDK_FOUND="${sdk_dir}"
    break
  fi
done
if [[ -z "${SDK_FOUND}" ]]; then
  echo "[rotate_cali] liblivox_lidar_sdk_shared.so not found." >&2
  echo "              Set SDK_DIR=/path/to/sdk/library." >&2
  exit 1
fi
export LD_LIBRARY_PATH="${SDK_FOUND}:${LD_LIBRARY_PATH:-}"

LIVOX_CONFIG="${LIVOX_CONFIG:-${ROOT_DIR}/livox/src/livox_ros_driver2/config/MID360_config_2.json}"
DLIO_CONFIG="${DLIO_CONFIG:-${ROOT_DIR}/odom/src/direct_lidar_inertial_odometry/cfg/dlio.yaml}"
DLIO_PARAMS="${DLIO_PARAMS:-${ROOT_DIR}/odom/src/direct_lidar_inertial_odometry/cfg/params.yaml}"
LIDAR5_OUTPUT="${LIDAR5_OUTPUT:-${ROOT_DIR}/slam/config/gimbal_lidar_5.yaml}"
LIDAR3_OUTPUT="${LIDAR3_OUTPUT:-${ROOT_DIR}/slam/config/gimbal_lidar_3.yaml}"
LIVOX_BROADCAST_CODE="${LIVOX_BROADCAST_CODE:-}"

if [[ -z "${BAG_PATH}" ]]; then
  START_DRIVER=1
  LIDAR5_TOPIC="${LIDAR5_TOPIC:-/cali/raw/lidar5}"
  LIDAR3_TOPIC="${LIDAR3_TOPIC:-/cali/raw/lidar3}"
  IMU5_TOPIC="${IMU5_TOPIC:-/cali/raw/imu5}"
  IMU3_TOPIC="${IMU3_TOPIC:-/cali/raw/imu3}"
else
  START_DRIVER=0
  LIDAR5_TOPIC="${LIDAR5_TOPIC:-/sentry/raw/lidar5}"
  LIDAR3_TOPIC="${LIDAR3_TOPIC:-/sentry/raw/lidar3}"
  IMU5_TOPIC="${IMU5_TOPIC:-/sentry/raw/imu5}"
  IMU3_TOPIC="${IMU3_TOPIC:-/sentry/raw/imu3}"
fi
if [[ -z "${RECORD_ROSBAG}" ]]; then
  if [[ "${START_DRIVER}" == "1" ]]; then
    RECORD_ROSBAG=1
  else
    RECORD_ROSBAG=0
  fi
fi
if [[ "${RECORD_ROSBAG}" != "0" && "${RECORD_ROSBAG}" != "1" ]]; then
  echo "[rotate_cali] RECORD_ROSBAG must be 0 or 1." >&2
  exit 1
fi
if [[ "${RECORD_ROSBAG}" == "1" ]]; then
  ROSBAG_OUTPUT="${ROSBAG_OUTPUT:-${ROOT_DIR}/rosbag/$(date +%Y%m%d_%H%M%S)_yaw_cali}"
  if [[ -e "${ROSBAG_OUTPUT}" ]]; then
    echo "[rotate_cali] Rosbag output already exists: ${ROSBAG_OUTPUT}" >&2
    exit 1
  fi
fi

for configuration in "${LIVOX_CONFIG}" "${DLIO_CONFIG}" "${DLIO_PARAMS}" \
                     "${LIDAR5_OUTPUT}" "${LIDAR3_OUTPUT}"; do
  if [[ ! -f "${configuration}" ]]; then
    echo "[rotate_cali] Missing configuration: ${configuration}" >&2
    exit 1
  fi
done

REQUIRED_EXECUTABLES=(
  "single_test single_lidar_odom"
  "cali_ws dual_yaw_circle_cali"
)
if [[ "${START_DRIVER}" == "1" ]]; then
  REQUIRED_EXECUTABLES+=("livox_ros_driver2 livox_ros_driver2_node")
fi
for package_executable in "${REQUIRED_EXECUTABLES[@]}"; do
  read -r package executable <<<"${package_executable}"
  if ! ros2 pkg executables "${package}" | awk '{print $2}' | grep -Fxq "${executable}"; then
    echo "[rotate_cali] ${package}/${executable} is not built or sourced." >&2
    echo "[rotate_cali] Build commands:" >&2
    echo "  cd ${ROOT_DIR}/odom && colcon build --packages-select single_test --symlink-install" >&2
    echo "  cd ${ROOT_DIR}/slam && colcon build --packages-select cali_ws --symlink-install" >&2
    exit 1
  fi
done

if [[ -n "${BAG_PATH}" ]]; then
  if [[ ! -e "${BAG_PATH}" ]]; then
    echo "[rotate_cali] Bag path does not exist: ${BAG_PATH}" >&2
    exit 1
  fi
  BAG_INFO="$(ros2 bag info "${BAG_PATH}")"
  for topic in "${LIDAR5_TOPIC}" "${LIDAR3_TOPIC}"; do
    if ! grep -Fq "Topic: ${topic} | Type: sensor_msgs/msg/PointCloud2" <<<"${BAG_INFO}"; then
      echo "[rotate_cali] ${topic} in the bag is not sensor_msgs/msg/PointCloud2." >&2
      echo "              DLIO cannot consume Livox CustomMsg directly." >&2
      echo "              Record calibration data with this script/driver (xfer_format=0)." >&2
      exit 1
    fi
  done
fi

STALE_PROCESSES="$(
  pgrep -a -f \
    'livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node|single_test/lib/single_test/single_lidar_odom|cali_ws/lib/cali_ws/dual_yaw_circle_cali' \
    || true
)"
if [[ -n "${STALE_PROCESSES}" ]]; then
  echo "[rotate_cali] Calibration/driver processes are already running:" >&2
  echo "${STALE_PROCESSES}" >&2
  echo "[rotate_cali] Stop them before starting a new calibration." >&2
  exit 1
fi

DRIVER_PID=""
DLIO5_PID=""
DLIO3_PID=""
CALI_PID=""
PLAYBACK_PID=""
RECORD_PID=""

stop_process_group() {
  local pid="${1:-}"
  local signal="${2:-TERM}"
  if [[ -z "${pid}" ]]; then
    return
  fi
  if kill -0 -- "-${pid}" 2>/dev/null; then
    kill "-${signal}" -- "-${pid}" 2>/dev/null || true
  elif kill -0 "${pid}" 2>/dev/null; then
    kill "-${signal}" "${pid}" 2>/dev/null || true
  fi
}

process_group_alive() {
  local pid="${1:-}"
  [[ -n "${pid}" ]] && kill -0 -- "-${pid}" 2>/dev/null
}

stop_runtime() {
  local signal="$1"
  stop_process_group "${CALI_PID}" "${signal}"
  stop_process_group "${DLIO5_PID}" "${signal}"
  stop_process_group "${DLIO3_PID}" "${signal}"
  stop_process_group "${PLAYBACK_PID}" "${signal}"
  stop_process_group "${DRIVER_PID}" "${signal}"
}

stop_all() {
  local signal="$1"
  stop_runtime "${signal}"
  stop_process_group "${RECORD_PID}" "${signal}"
}

force_cleanup() {
  local status="$1"
  trap - EXIT
  trap '' INT TERM
  printf '\n[rotate_cali] Forced shutdown; killing all calibration nodes.\n' >&2
  stop_all KILL
  wait 2>/dev/null || true
  exit "${status}"
}

cleanup() {
  local status=$?
  trap - EXIT
  trap 'force_cleanup 130' INT
  trap 'force_cleanup 143' TERM
  printf '\n[rotate_cali] Stopping calibration nodes...\n' >&2
  stop_runtime TERM
  if [[ -n "${RECORD_PID}" ]]; then
    printf '[rotate_cali] Finalizing rosbag: %s\n' "${ROSBAG_OUTPUT}" >&2
    stop_process_group "${RECORD_PID}" TERM
    # ros2 bag record may leave its writer child alive after the CLI leader
    # exits. Wait for the complete session/process group so SQLite can finish
    # its final transaction and checkpoint before any forced kill.
    for _ in {1..200}; do
      if ! process_group_alive "${RECORD_PID}"; then
        break
      fi
      sleep 0.1
    done
  fi
  stop_runtime KILL
  if process_group_alive "${RECORD_PID}"; then
    printf '[rotate_cali] Rosbag did not stop within 20 seconds; forcing shutdown.\n' >&2
    stop_process_group "${RECORD_PID}" KILL
  fi
  wait 2>/dev/null || true
  exit "${status}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "[rotate_cali] Livox SDK: ${SDK_FOUND}"
echo "[rotate_cali] Calibration outputs:"
echo "  lidar5: ${LIDAR5_OUTPUT}"
echo "  lidar3: ${LIDAR3_OUTPUT}"
echo
echo "操作顺序："
echo "  1. 保持底盘和云台完全静止至少 5 秒，让两个 DLIO 完成 IMU 初始化。"
echo "  2. 底盘始终不能移动，只转云台 yaw 轴。"
echo "  3. 以 10-20 deg/s 平滑顺时针转 1-2 圈。"
echo "  4. 再以相同速度平滑逆时针转 1-2 圈，避免突然启停。"
echo "  5. 两路轨迹都通过检查后才会写入 YAML；成功后脚本自动退出。"
echo

if [[ "${START_DRIVER}" == "1" ]]; then
  DRIVER_ARGS=(
    ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args
    -p xfer_format:=0
    -p multi_topic:=1
    -p data_src:=0
    -p publish_freq:=10.0
    -p output_data_type:=0
    -p frame_id:=livox_frame
    -p user_config_path:="${LIVOX_CONFIG}"
    -r "/livox/lidar_192_168_1_5:=${LIDAR5_TOPIC}"
    -r "/livox/lidar_192_168_1_3:=${LIDAR3_TOPIC}"
    -r "/livox/imu_192_168_1_5:=${IMU5_TOPIC}"
    -r "/livox/imu_192_168_1_3:=${IMU3_TOPIC}"
  )
  if [[ -n "${LIVOX_BROADCAST_CODE}" ]]; then
    DRIVER_ARGS+=( -p cmdline_input_bd_code:="${LIVOX_BROADCAST_CODE}" )
  fi
  echo "[rotate_cali] Starting dual MID360 driver (PointCloud2 mode)..."
  setsid "${DRIVER_ARGS[@]}" &
  DRIVER_PID=$!
  sleep "${DRIVER_STARTUP_WAIT:-2}"
  if ! kill -0 "${DRIVER_PID}" 2>/dev/null; then
    echo "[rotate_cali] Livox driver exited during startup." >&2
    wait "${DRIVER_PID}"
  fi
fi

if [[ "${RECORD_ROSBAG}" == "1" ]]; then
  mkdir -p "$(dirname -- "${ROSBAG_OUTPUT}")"
  echo "[rotate_cali] Recording diagnostic rosbag: ${ROSBAG_OUTPUT}"
  setsid ros2 bag record \
    --output "${ROSBAG_OUTPUT}" \
    --storage sqlite3 \
    "${LIDAR5_TOPIC}" "${LIDAR3_TOPIC}" \
    "${IMU5_TOPIC}" "${IMU3_TOPIC}" \
    /cali/lidar5/pose /cali/lidar3/pose \
    /parameter_events /rosout &
  RECORD_PID=$!
  sleep 1
  if ! kill -0 "${RECORD_PID}" 2>/dev/null; then
    echo "[rotate_cali] Rosbag recorder exited during startup." >&2
    wait "${RECORD_PID}"
  fi
fi

start_dlio() {
  local label="$1"
  local namespace="$2"
  local point_topic="$3"
  local imu_topic="$4"
  setsid ros2 run single_test single_lidar_odom --ros-args \
    --params-file "${DLIO_CONFIG}" \
    --params-file "${DLIO_PARAMS}" \
    -r "__ns:=${namespace}" \
    -r "__node:=dlio_${label}" \
    -r "pointcloud:=${point_topic}" \
    -r "imu:=${imu_topic}" \
    -r "/tf:=${namespace}/tf" \
    -r "/tf_static:=${namespace}/tf_static" \
    -p "publish/keyframes:=false" \
    -p "frames/odom:=cali_${label}_odom" \
    -p "frames/baselink:=cali_${label}" \
    -p "frames/lidar:=${label}_lidar" \
    -p "frames/imu:=${label}_imu" &
  STARTED_PID=$!
}

echo "[rotate_cali] Starting independent lidar5 DLIO..."
start_dlio lidar5 /cali/lidar5 "${LIDAR5_TOPIC}" "${IMU5_TOPIC}"
DLIO5_PID="${STARTED_PID}"
echo "[rotate_cali] Starting independent lidar3 DLIO..."
start_dlio lidar3 /cali/lidar3 "${LIDAR3_TOPIC}" "${IMU3_TOPIC}"
DLIO3_PID="${STARTED_PID}"

sleep "${DLIO_STARTUP_WAIT:-1}"
if ! kill -0 "${DLIO5_PID}" 2>/dev/null || ! kill -0 "${DLIO3_PID}" 2>/dev/null; then
  echo "[rotate_cali] A DLIO instance exited during startup." >&2
  exit 1
fi

echo "[rotate_cali] Starting constrained dual-yaw trajectory solver..."
setsid ros2 run cali_ws dual_yaw_circle_cali --ros-args \
  -p "lidar5_pose_topic:=/cali/lidar5/pose" \
  -p "lidar3_pose_topic:=/cali/lidar3/pose" \
  -p "lidar5_output_file:=${LIDAR5_OUTPUT}" \
  -p "lidar3_output_file:=${LIDAR3_OUTPUT}" \
  -p "minimum_direction_degrees:=${MIN_DIRECTION_DEG:-270.0}" \
  -p "maximum_circle_residual:=${MAX_CIRCLE_RESIDUAL:-0.01}" \
  -p "maximum_radius_error:=${MAX_RADIUS_ERROR:-0.03}" \
  -p "equal_height:=true" \
  -p "auto_exit:=true" &
CALI_PID=$!

if [[ -n "${BAG_PATH}" ]]; then
  sleep "${CALIBRATOR_STARTUP_WAIT:-1}"
  echo "[rotate_cali] Replaying ${BAG_PATH} at ${BAG_RATE}x..."
  setsid ros2 bag play "${BAG_PATH}" \
    --rate "${BAG_RATE}" \
    --topics "${LIDAR5_TOPIC}" "${LIDAR3_TOPIC}" \
             "${IMU5_TOPIC}" "${IMU3_TOPIC}" &
  PLAYBACK_PID=$!
fi

WATCH_PIDS=("${DLIO5_PID}" "${DLIO3_PID}" "${CALI_PID}")
if [[ -n "${DRIVER_PID}" ]]; then
  WATCH_PIDS+=("${DRIVER_PID}")
fi
if [[ -n "${PLAYBACK_PID}" ]]; then
  WATCH_PIDS+=("${PLAYBACK_PID}")
fi
if [[ -n "${RECORD_PID}" ]]; then
  WATCH_PIDS+=("${RECORD_PID}")
fi
FINISHED_PID=""
set +e
wait -n -p FINISHED_PID "${WATCH_PIDS[@]}"
CHILD_STATUS=$?
set -e
if [[ "${FINISHED_PID}" != "${CALI_PID}" ]]; then
  echo "[rotate_cali] A required driver/DLIO process exited before calibration completed." >&2
  if [[ "${CHILD_STATUS}" -eq 0 ]]; then
    exit 1
  fi
  exit "${CHILD_STATUS}"
fi
if [[ "${CHILD_STATUS}" -ne 0 ]]; then
  echo "[rotate_cali] Calibration node failed with status ${CHILD_STATUS}." >&2
  exit "${CHILD_STATUS}"
fi
CALI_PID=""

echo "[rotate_cali] Calibration completed successfully:"
echo "  ${LIDAR5_OUTPUT}"
echo "  ${LIDAR3_OUTPUT}"
if [[ "${RECORD_ROSBAG}" == "1" ]]; then
  echo "[rotate_cali] Diagnostic rosbag: ${ROSBAG_OUTPUT}"
fi
