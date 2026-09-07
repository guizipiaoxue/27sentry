#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="${SCRIPT_DIR}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
LIVOX_SETUP="${ROOT_DIR}/livox/install/setup.bash"
SLAM_SETUP="${ROOT_DIR}/slam/install/setup.bash"

for setup_file in "${ROS_SETUP}" "${LIVOX_SETUP}"; do
  if [[ ! -f "${setup_file}" ]]; then
    echo "[use_ground] Setup file not found: ${setup_file}" >&2
    exit 1
  fi
done

set +u
source "${ROS_SETUP}"
source "${LIVOX_SETUP}"
set -u

# Match rotate_cali.sh: Livox SDK2 is installed under /usr/local/lib.
SDK_DIR="${SDK_DIR:-/usr/local/lib}"
SDK_LIBRARY="${SDK_DIR}/liblivox_lidar_sdk_shared.so"
if [[ ! -f "${SDK_LIBRARY}" ]]; then
  echo "[use_ground] Livox SDK library not found: ${SDK_LIBRARY}" >&2
  echo "             Set SDK_DIR to the directory containing liblivox_lidar_sdk_shared.so." >&2
  exit 1
fi
export LD_LIBRARY_PATH="${SDK_DIR}:${LD_LIBRARY_PATH:-}"

CALI_SOURCE="${ROOT_DIR}/slam/src/cali_ws/src/use_ground_cali.cpp"
CALI_EXECUTABLE="${ROOT_DIR}/slam/install/cali_ws/lib/cali_ws/use_ground_cali"
if [[ ! -x "${CALI_EXECUTABLE}" || "${CALI_SOURCE}" -nt "${CALI_EXECUTABLE}" ]]; then
  echo "[use_ground] Building cali_ws..."
  (
    cd "${ROOT_DIR}/slam"
    colcon build --packages-select cali_ws --symlink-install
  )
fi

if [[ ! -f "${SLAM_SETUP}" ]]; then
  echo "[use_ground] SLAM setup file not found after build: ${SLAM_SETUP}" >&2
  exit 1
fi
set +u
source "${SLAM_SETUP}"
set -u

if [[ ! -x "${CALI_EXECUTABLE}" ]]; then
  echo "[use_ground] use_ground_cali was not installed: ${CALI_EXECUTABLE}" >&2
  exit 1
fi

LIVOX_CONFIG="${LIVOX_CONFIG:-${ROOT_DIR}/livox/src/livox_ros_driver2/config/MID360_config_2.json}"
LIDAR1_TOPIC="${LIDAR1_TOPIC:-livox/lidar_192_168_1_5}"
LIDAR2_TOPIC="${LIDAR2_TOPIC:-livox/lidar_192_168_1_3}"
OUTPUT_FILE="${OUTPUT_FILE:-${ROOT_DIR}/slam/ground_z_calibration.yaml}"
MIN_SAMPLES="${MIN_SAMPLES:-20}"
MAX_SAMPLES="${MAX_SAMPLES:-100}"
DRIVER_STARTUP_WAIT="${DRIVER_STARTUP_WAIT:-2}"
LIVOX_BROADCAST_CODE="${LIVOX_BROADCAST_CODE:-}"

if [[ ! -f "${LIVOX_CONFIG}" ]]; then
  echo "[use_ground] Livox config not found: ${LIVOX_CONFIG}" >&2
  exit 1
fi

if pgrep -f '[l]ivox_ros_driver2_node' >/dev/null 2>&1; then
  echo "[use_ground] Another livox_ros_driver2_node is already running." >&2
  echo "             Stop it first; otherwise the SDK UDP ports will report 'bind failed'." >&2
  exit 1
fi

# MID360_config_2.json binds the host side to 192.168.1.50. A missing address
# is the other common cause of the SDK's unqualified "bind failed" message.
if command -v ip >/dev/null 2>&1 && ! ip -4 -o address show 2>/dev/null | grep -qw '192.168.1.50'; then
  echo "[use_ground] Warning: host IP 192.168.1.50 is not configured on a network interface." >&2
  echo "             MID360_config_2.json may fail to bind until the host IP is configured." >&2
fi

DRIVER_PID=""
CALI_PID=""
cleanup() {
  local status=$?
  trap - EXIT INT TERM
  for pid in "${CALI_PID}" "${DRIVER_PID}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
    fi
  done
  sleep 1
  for pid in "${CALI_PID}" "${DRIVER_PID}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -KILL -- "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
    fi
  done
  wait 2>/dev/null || true
  exit "${status}"
}
trap cleanup EXIT INT TERM

DRIVER_ARGS=(
  ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args
  -p xfer_format:=1
  -p multi_topic:=1
  -p data_src:=0
  -p publish_freq:=10.0
  -p output_data_type:=0
  -p frame_id:=livox_frame
  -p user_config_path:="${LIVOX_CONFIG}"
)
if [[ -n "${LIVOX_BROADCAST_CODE}" ]]; then
  DRIVER_ARGS+=( -p cmdline_input_bd_code:="${LIVOX_BROADCAST_CODE}" )
fi

echo "[use_ground] Starting Livox driver with ${LIVOX_CONFIG}"
setsid "${DRIVER_ARGS[@]}" &
DRIVER_PID=$!
sleep "${DRIVER_STARTUP_WAIT}"

if ! kill -0 "${DRIVER_PID}" 2>/dev/null; then
  wait "${DRIVER_PID}" || true
  echo "[use_ground] Livox driver exited during startup." >&2
  exit 1
fi

echo "[use_ground] Starting ground calibration"
echo "[use_ground] Output: ${OUTPUT_FILE}"
setsid ros2 run cali_ws use_ground_cali --ros-args \
  -p lidar1_topic:="${LIDAR1_TOPIC}" \
  -p lidar2_topic:="${LIDAR2_TOPIC}" \
  -p output_file:="${OUTPUT_FILE}" \
  -p min_samples:="${MIN_SAMPLES}" \
  -p max_samples:="${MAX_SAMPLES}" &
CALI_PID=$!

while true; do
  if ! kill -0 "${DRIVER_PID}" 2>/dev/null; then
    wait "${DRIVER_PID}" || true
    echo "[use_ground] Livox driver stopped unexpectedly." >&2
    exit 1
  fi
  if ! kill -0 "${CALI_PID}" 2>/dev/null; then
    wait "${CALI_PID}"
    exit $?
  fi
  sleep 1
done
