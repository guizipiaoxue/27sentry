#!/usr/bin/env bash
# 双 MID360 云台标定启动脚本。
# 脚本可以从任意当前目录执行：bash sentry_test_py/rotate_cali.sh
set -Ee -o pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
LIVOX_WS="${REPO_ROOT}/sentry_test_py/livox"
CALI_WS="${REPO_ROOT}/sentry_test_py/slam"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "[rotate_cali] ROS2 setup not found: ${ROS_SETUP}" >&2
  exit 1
fi
if [[ ! -f "${LIVOX_WS}/install/setup.bash" ]]; then
  echo "[rotate_cali] Livox workspace is not built: ${LIVOX_WS}/install/setup.bash" >&2
  echo "             Build it first with: cd ${LIVOX_WS}/src/livox_ros_driver2 && ./build.sh humble" >&2
  exit 1
fi
if [[ ! -f "${CALI_WS}/install/setup.bash" ]]; then
  echo "[rotate_cali] cali_ws is not built: ${CALI_WS}/install/setup.bash" >&2
  echo "             Build it first with the command shown below." >&2
  exit 1
fi

# ROS/colcon setup 脚本会读取若干可能尚未定义的变量（例如
# AMENT_TRACE_SETUP_FILES），因此 source 时暂时关闭 nounset；环境加载后再恢复。
set +u
source "${ROS_SETUP}"
source "${LIVOX_WS}/install/setup.bash"
source "${CALI_WS}/install/setup.bash"
set -u

# Livox 驱动通过 class_loader 在运行时加载 SDK，必须把 SDK 动态库目录加入
# LD_LIBRARY_PATH；仅 source ROS 工作空间并不会自动设置这个路径。
SDK_DIRS=(
  "/usr/local/lib"
  "${REPO_ROOT}/Livox-SDK2/build/sdk_core"
  "${REPO_ROOT}/slam_source/livox_ws/src/livox_ros_driver2/.livox_sdk/lib"
)
SDK_FOUND=""
for dir in "${SDK_DIRS[@]}"; do
  if [[ -f "${dir}/liblivox_lidar_sdk_shared.so" ]]; then
    SDK_FOUND="${dir}"
    break
  fi
done
if [[ -z "${SDK_FOUND}" ]]; then
  echo "[rotate_cali] liblivox_lidar_sdk_shared.so not found." >&2
  echo "             Install/build Livox-SDK2, or place the .so in /usr/local/lib." >&2
  exit 1
fi
export LD_LIBRARY_PATH="${SDK_FOUND}:${LD_LIBRARY_PATH:-}"

# MID360_config_2.json 中的设备：192.168.1.5 与 192.168.1.3。
# msg_MID360_launch.py 已配置 multi_topic=1，因此话题后缀为下划线形式的 IP。
LIDAR1_TOPIC="${LIDAR1_TOPIC:-livox/lidar_192_168_1_5}"
LIDAR2_TOPIC="${LIDAR2_TOPIC:-livox/lidar_192_168_1_3}"
IMU1_TOPIC="${IMU1_TOPIC:-livox/imu_192_168_1_5}"
IMU2_TOPIC="${IMU2_TOPIC:-livox/imu_192_168_1_3}"
OUTPUT_FILE="${OUTPUT_FILE:-${CALI_WS}/lidar_calibration.yaml}"

DRIVER_PID=""
cleanup() {
  local status=$?
  if [[ -n "${DRIVER_PID}" ]] && kill -0 "${DRIVER_PID}" 2>/dev/null; then
    kill "${DRIVER_PID}" 2>/dev/null || true
    wait "${DRIVER_PID}" 2>/dev/null || true
  fi
  exit "${status}"
}
trap cleanup EXIT INT TERM

# 驱动后台运行，标定节点以前台运行；Ctrl-C 会同时停止驱动。
ros2 launch livox_ros_driver2 msg_MID360_launch.py &
DRIVER_PID=$!
sleep "${DRIVER_STARTUP_WAIT:-2}"

ros2 run cali_ws rotate_cali --ros-args \
  -p lidar1_topic:="${LIDAR1_TOPIC}" \
  -p lidar2_topic:="${LIDAR2_TOPIC}" \
  -p imu1_topic:="${IMU1_TOPIC}" \
  -p imu2_topic:="${IMU2_TOPIC}" \
  -p output_file:="${OUTPUT_FILE}" \
  -p startup_seconds:="${STARTUP_SECONDS:-1.0}"
