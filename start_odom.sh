#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="${SCRIPT_DIR}"

ROS_SETUP="/opt/ros/humble/setup.bash"
LIVOX_SETUP="${ROOT_DIR}/livox/install/setup.bash"
SLAM_SETUP="${ROOT_DIR}/slam/install/setup.bash"
ODOM_SETUP="${ROOT_DIR}/odom/install/setup.bash"

for setup_file in "${ROS_SETUP}" "${LIVOX_SETUP}" "${SLAM_SETUP}" "${ODOM_SETUP}"; do
  if [[ ! -f "${setup_file}" ]]; then
    echo "[start_odom] Missing setup file: ${setup_file}" >&2
    echo "[start_odom] Build the corresponding workspace first." >&2
    exit 1
  fi
done

# ROS setup scripts can reference unset variables, so nounset is temporarily
# disabled while sourcing the three overlays.
set +u
source "${ROS_SETUP}"
source "${LIVOX_SETUP}"
source "${SLAM_SETUP}"
source "${ODOM_SETUP}"
set -u

# Livox ROS Driver 2 links against the SDK shared library. SDK_DIR may be used
# to override the installation directory; /usr/local/lib is used on this host.
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
  echo "[start_odom] liblivox_lidar_sdk_shared.so not found." >&2
  echo "             Set SDK_DIR=/path/to/lib or install it in /usr/local/lib." >&2
  exit 1
fi
export LD_LIBRARY_PATH="${SDK_FOUND}:${LD_LIBRARY_PATH:-}"
if [[ -d "/usr/local/lib" ]]; then
  export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH}"
fi

LIVOX_CONFIG="${LIVOX_CONFIG:-${ROOT_DIR}/livox/src/livox_ros_driver2/config/MID360_config_2.json}"
ODOM_PARAMS="${ODOM_PARAMS:-${ROOT_DIR}/odom/config/odom.yaml}"
LOOP_PARAMS="${LOOP_PARAMS:-${ROOT_DIR}/odom/config/loop.yaml}"
ENABLE_GTSAM="${ENABLE_GTSAM:-1}"
LIVOX_BROADCAST_CODE="${LIVOX_BROADCAST_CODE:-}"
DRIVER_STARTUP_WAIT="${DRIVER_STARTUP_WAIT:-2}"
FUSION_STARTUP_WAIT="${FUSION_STARTUP_WAIT:-1}"
IMU_CALIBRATION_TIMEOUT="${IMU_CALIBRATION_TIMEOUT:-30}"

if [[ "${ENABLE_GTSAM}" != "0" && "${ENABLE_GTSAM}" != "1" ]]; then
  echo "[start_odom] ENABLE_GTSAM must be 0 or 1." >&2
  exit 1
fi

CONFIG_FILES=("${LIVOX_CONFIG}" "${ODOM_PARAMS}")
if [[ "${ENABLE_GTSAM}" == "1" ]]; then
  CONFIG_FILES+=("${LOOP_PARAMS}")
fi
for config_file in "${CONFIG_FILES[@]}"; do
  if [[ ! -f "${config_file}" ]]; then
    echo "[start_odom] Missing configuration file: ${config_file}" >&2
    exit 1
  fi
done

REQUIRED_EXECUTABLES=(
  "livox_ros_driver2 livox_ros_driver2_node"
  "fusion_ws fusion_pcl"
  "odom_ws odom"
)
if [[ "${ENABLE_GTSAM}" == "1" ]]; then
  REQUIRED_EXECUTABLES+=(
    "loop_closure loop_detector"
    "loop_closure pose_graph_backend"
  )
fi
for package_executable in "${REQUIRED_EXECUTABLES[@]}"; do
  read -r package executable <<<"${package_executable}"
  if ! ros2 pkg executables "${package}" | awk '{print $2}' | grep -Fxq "${executable}"; then
    echo "[start_odom] ${package}/${executable} is not built or not sourced." >&2
    exit 1
  fi
done

DRIVER_PID=""
FUSION_PID=""
ODOM_PID=""
LOOP_PID=""
BACKEND_PID=""

stop_process_group() {
  local pid="$1"
  local signal="$2"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    kill "-${signal}" -- "-${pid}" 2>/dev/null ||
      kill "-${signal}" "${pid}" 2>/dev/null || true
  fi
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM

  stop_process_group "${BACKEND_PID}" TERM
  stop_process_group "${ODOM_PID}" TERM
  stop_process_group "${LOOP_PID}" TERM
  stop_process_group "${FUSION_PID}" TERM
  stop_process_group "${DRIVER_PID}" TERM
  sleep 1
  stop_process_group "${BACKEND_PID}" KILL
  stop_process_group "${ODOM_PID}" KILL
  stop_process_group "${LOOP_PID}" KILL
  stop_process_group "${FUSION_PID}" KILL
  stop_process_group "${DRIVER_PID}" KILL
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

echo "[start_odom] SDK: ${SDK_FOUND}"
echo "[start_odom] Starting Livox driver for lidar 5 and lidar 3..."
setsid "${DRIVER_ARGS[@]}" &
DRIVER_PID=$!
sleep "${DRIVER_STARTUP_WAIT}"
if ! kill -0 "${DRIVER_PID}" 2>/dev/null; then
  echo "[start_odom] Livox driver exited during startup." >&2
  wait "${DRIVER_PID}"
fi

echo "[start_odom] Starting point-cloud and IMU fusion..."
setsid ros2 run fusion_ws fusion_pcl &
FUSION_PID=$!
sleep "${FUSION_STARTUP_WAIT}"
if ! kill -0 "${FUSION_PID}" 2>/dev/null; then
  echo "[start_odom] Fusion node exited during startup." >&2
  wait "${FUSION_PID}"
fi

echo "[start_odom] Keep the gimbal stationary; waiting for both IMUs to calibrate..."
if ! timeout "${IMU_CALIBRATION_TIMEOUT}" \
    ros2 topic echo /gimbal/imu_calibrated std_msgs/msg/Bool \
      --once --filter 'm.data' \
      --qos-reliability reliable --qos-durability transient_local \
    | grep -Fq "data: true"; then
  echo "[start_odom] IMU calibration did not complete within ${IMU_CALIBRATION_TIMEOUT}s." >&2
  echo "             Keep both lidars stationary and check both IMU topics." >&2
  exit 1
fi
echo "[start_odom] Both IMUs calibrated."

if [[ "${ENABLE_GTSAM}" == "1" ]]; then
  echo "[start_odom] Starting Scan Context++ loop detector..."
  setsid ros2 run loop_closure loop_detector --ros-args \
    --params-file "${LOOP_PARAMS}" \
    -r keyframe:=/dlio/odom_node/keyframe \
    -r loop_constraint:=/loop_closure/constraint &
  LOOP_PID=$!

  echo "[start_odom] Starting GTSAM iSAM2 mapping backend..."
  setsid ros2 run loop_closure pose_graph_backend --ros-args \
    --params-file "${LOOP_PARAMS}" \
    -r keyframe:=/dlio/odom_node/keyframe \
    -r loop_constraint:=/loop_closure/constraint \
    -r optimized_path:=/mapping/optimized_path \
    -r save_map:=/mapping/save_map &
  BACKEND_PID=$!

  sleep 1
  if ! kill -0 "${LOOP_PID}" 2>/dev/null; then
    echo "[start_odom] Loop detector exited during startup." >&2
    wait "${LOOP_PID}"
  fi
  if ! kill -0 "${BACKEND_PID}" 2>/dev/null; then
    echo "[start_odom] GTSAM mapping backend exited during startup." >&2
    wait "${BACKEND_PID}"
  fi
fi

echo "[start_odom] Starting fused DLIO odometry..."
setsid ros2 run odom_ws odom --ros-args \
  --params-file "${ODOM_PARAMS}" \
  -r pointcloud:=/gimbal/cloud_fused \
  -r imu:=/gimbal/imu_fused \
  -r path:=/path \
  -r pose:=/dlio/odom_node/pose \
  -r odom:=/dlio/odom_node/odom \
  -r kf_pose:=/dlio/odom_node/keyframes \
  -r kf_cloud:=/dlio/odom_node/pointcloud/keyframe \
  -r deskewed:=/fusion_pcl \
  -r keyframe:=/dlio/odom_node/keyframe \
  -p imu/calibration:=false \
  -p pointcloud/deskew:=false \
  -p odom/computeTimeOffset:=false \
  -p publish/pose_odom:=true \
  -p publish/keyframes:=true \
  -p frames/odom:=odom \
  -p frames/baselink:=gimbal \
  -p frames/lidar:=fusion_lidar \
  -p frames/imu:=fusion_imu &
ODOM_PID=$!

sleep 1
if ! kill -0 "${ODOM_PID}" 2>/dev/null; then
  echo "[start_odom] DLIO odometry exited during startup." >&2
  wait "${ODOM_PID}"
fi

echo "[start_odom] DLIO keyframe cloud: /dlio/odom_node/pointcloud/keyframe"
if [[ "${ENABLE_GTSAM}" == "1" ]]; then
  echo "[start_odom] GTSAM enabled. Optimized map is written only on save."
  echo "[start_odom] Topics: /mapping/optimized_path, /loop_closure/constraint, /tf"
  echo "[start_odom] Save map: ros2 service call /mapping/save_map std_srvs/srv/Trigger '{}'"
else
  echo "[start_odom] GTSAM disabled. DLIO Fixed Frame: odom"
fi
echo "[start_odom] Press Ctrl+C to stop all nodes."

# Returning when any child exits prevents a partially running pipeline.
PIPELINE_PIDS=("${DRIVER_PID}" "${FUSION_PID}" "${ODOM_PID}")
if [[ "${ENABLE_GTSAM}" == "1" ]]; then
  PIPELINE_PIDS+=("${LOOP_PID}" "${BACKEND_PID}")
fi
wait -n "${PIPELINE_PIDS[@]}"
