#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="${SCRIPT_DIR}"

usage() {
  cat <<'EOF'
Usage: ./start_odom.sh [OPTIONS]

Start the dual-Livox, dual-IMU fusion pipeline with the selected odometry.

Options:
  -a, --algorithm ALGORITHM  Odometry algorithm: dlio or plio (default: dlio)
      --record-bag           Record diagnostic topics (default)
      --no-record-bag        Disable rosbag recording
      --bag-output PATH      Set the rosbag output directory
  -h, --help                 Show this help message

Environment defaults: ODOM_ALGORITHM=dlio|plio, RECORD_ROSBAG=0|1,
ROSBAG_OUTPUT=/path/to/bag, FUSION_CLOUD_VOXEL_LEAF_SIZE=0.08.
EOF
}

ODOM_ALGORITHM="${ODOM_ALGORITHM:-dlio}"
RECORD_ROSBAG="${RECORD_ROSBAG:-1}"
ROSBAG_OUTPUT="${ROSBAG_OUTPUT:-}"
while [[ $# -gt 0 ]]; do
  case "$1" in
    -a|--algorithm)
      if [[ $# -lt 2 ]]; then
        echo "[start_odom] $1 requires an algorithm name." >&2
        usage >&2
        exit 2
      fi
      ODOM_ALGORITHM="$2"
      shift 2
      ;;
    --algorithm=*)
      ODOM_ALGORITHM="${1#*=}"
      shift
      ;;
    --record-bag)
      RECORD_ROSBAG="1"
      shift
      ;;
    --no-record-bag)
      RECORD_ROSBAG="0"
      shift
      ;;
    --bag-output)
      if [[ $# -lt 2 ]]; then
        echo "[start_odom] $1 requires an output path." >&2
        usage >&2
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
    --)
      shift
      break
      ;;
    *)
      echo "[start_odom] Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ $# -gt 0 ]]; then
  echo "[start_odom] Unexpected positional argument: $1" >&2
  usage >&2
  exit 2
fi

case "${ODOM_ALGORITHM,,}" in
  dlio)
    ODOM_ALGORITHM="dlio"
    ODOM_NAME="DLIO"
    ;;
  plio|point-lio|point_lio)
    ODOM_ALGORITHM="plio"
    ODOM_NAME="Point-LIO"
    ;;
  *)
    echo "[start_odom] Unsupported algorithm: ${ODOM_ALGORITHM}" >&2
    echo "[start_odom] Choose dlio or plio." >&2
    exit 2
    ;;
esac

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
if [[ "${ODOM_ALGORITHM}" == "dlio" ]]; then
  ODOM_PARAMS="${ODOM_PARAMS:-${ROOT_DIR}/odom/config/odom.yaml}"
else
  ODOM_PARAMS="${ODOM_PARAMS:-${ROOT_DIR}/odom/src/plio/config/point_lio.yaml}"
fi
LOOP_PARAMS="${LOOP_PARAMS:-${ROOT_DIR}/odom/config/loop.yaml}"
ENABLE_GTSAM="${ENABLE_GTSAM:-1}"
LIVOX_BROADCAST_CODE="${LIVOX_BROADCAST_CODE:-}"
DRIVER_STARTUP_WAIT="${DRIVER_STARTUP_WAIT:-2}"
FUSION_STARTUP_WAIT="${FUSION_STARTUP_WAIT:-1}"
IMU_CALIBRATION_TIMEOUT="${IMU_CALIBRATION_TIMEOUT:-30}"
FUSION_CLOUD_VOXEL_LEAF_SIZE="${FUSION_CLOUD_VOXEL_LEAF_SIZE:-0.08}"
RAW_LIDAR5_TOPIC="/sentry/raw/lidar5"
RAW_LIDAR3_TOPIC="/sentry/raw/lidar3"
RAW_IMU5_TOPIC="/sentry/raw/imu5"
RAW_IMU3_TOPIC="/sentry/raw/imu3"

if [[ "${RECORD_ROSBAG}" != "0" && "${RECORD_ROSBAG}" != "1" ]]; then
  echo "[start_odom] RECORD_ROSBAG must be 0 or 1." >&2
  exit 1
fi
if [[ "${RECORD_ROSBAG}" == "1" ]]; then
  if [[ -z "${ROSBAG_OUTPUT}" ]]; then
    ROSBAG_OUTPUT="${ROOT_DIR}/rosbag/$(date +%Y%m%d_%H%M%S)_${ODOM_ALGORITHM}"
  fi
  if [[ -e "${ROSBAG_OUTPUT}" ]]; then
    echo "[start_odom] Rosbag output already exists: ${ROSBAG_OUTPUT}" >&2
    exit 1
  fi
fi

if [[ "${ENABLE_GTSAM}" != "0" && "${ENABLE_GTSAM}" != "1" ]]; then
  echo "[start_odom] ENABLE_GTSAM must be 0 or 1." >&2
  exit 1
fi
if [[ "${ODOM_ALGORITHM}" == "plio" && "${ENABLE_GTSAM}" == "1" ]]; then
  echo "[start_odom] Point-LIO does not publish the DLIO keyframe topics required by GTSAM." >&2
  echo "[start_odom] GTSAM is disabled for this run." >&2
  ENABLE_GTSAM="0"
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
)
if [[ "${ODOM_ALGORITHM}" == "dlio" ]]; then
  REQUIRED_EXECUTABLES+=("odom_ws odom")
else
  REQUIRED_EXECUTABLES+=("plio point_lio")
fi
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

# Starting a second pipeline can mix old and new publishers on the same ROS
# graph and can associate multiple message types with one Livox topic.
STALE_PIPELINE_PROCESSES="$(
  pgrep -a -f \
    'livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node|fusion_ws/lib/fusion_ws/fusion_pcl|plio/lib/plio/point_lio|odom_ws/lib/odom_ws/odom' \
    || true
)"
if [[ -n "${STALE_PIPELINE_PROCESSES}" ]]; then
  echo "[start_odom] Another odometry pipeline is still running:" >&2
  echo "${STALE_PIPELINE_PROCESSES}" >&2
  echo "[start_odom] Stop those processes before starting a new run." >&2
  exit 1
fi

DRIVER_PID=""
FUSION_PID=""
ODOM_PID=""
LOOP_PID=""
BACKEND_PID=""
BAG_PID=""

stop_process_group() {
  local pid="$1"
  local signal="$2"
  if [[ -z "${pid}" ]]; then
    return
  fi

  if kill -0 -- "-${pid}" 2>/dev/null; then
    kill "-${signal}" -- "-${pid}" 2>/dev/null || true
  elif kill -0 "${pid}" 2>/dev/null; then
    kill "-${signal}" "${pid}" 2>/dev/null || true
  fi
}

stop_pipeline_processes() {
  local signal="$1"
  stop_process_group "${BACKEND_PID}" "${signal}"
  stop_process_group "${ODOM_PID}" "${signal}"
  stop_process_group "${LOOP_PID}" "${signal}"
  stop_process_group "${FUSION_PID}" "${signal}"
  stop_process_group "${DRIVER_PID}" "${signal}"
}

stop_all_processes() {
  local signal="$1"
  stop_pipeline_processes "${signal}"
  stop_process_group "${BAG_PID}" "${signal}"
}

force_cleanup() {
  local status="$1"
  trap - EXIT
  trap '' INT TERM
  printf '\n[start_odom] Forced shutdown requested; killing all nodes.\n' >&2
  stop_all_processes KILL
  wait 2>/dev/null || true
  exit "${status}"
}

cleanup() {
  local status=$?
  trap - EXIT
  trap 'force_cleanup 130' INT
  trap 'force_cleanup 143' TERM

  printf '\n[start_odom] Stopping all nodes...\n' >&2
  stop_pipeline_processes TERM
  # Point-LIO may be inside a long point-wise update and not return to the ROS
  # executor promptly. Stop the processing groups before waiting for bag flush.
  sleep 0.5
  stop_pipeline_processes KILL
  if [[ -n "${BAG_PID}" ]]; then
    printf '[start_odom] Finalizing rosbag: %s\n' "${ROSBAG_OUTPUT}" >&2
    stop_process_group "${BAG_PID}" TERM
    for _ in {1..30}; do
      if ! kill -0 "${BAG_PID}" 2>/dev/null; then
        break
      fi
      sleep 0.1
    done
    stop_process_group "${BAG_PID}" KILL
  fi
  wait 2>/dev/null || true
  exit "${status}"
}
trap cleanup EXIT
# Each node runs in its own session, so terminal SIGINT only reaches this
# supervisor. The first Ctrl+C performs a graceful shutdown so rosbag can flush;
# cleanup installs force_cleanup as the second-Ctrl+C fallback.
trap 'exit 130' INT
trap 'exit 143' TERM

DRIVER_ARGS=(
  ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args
  -p xfer_format:=1
  -p multi_topic:=1
  -p data_src:=0
  -p publish_freq:=10.0
  -p output_data_type:=0
  -p frame_id:=livox_frame
  -p user_config_path:="${LIVOX_CONFIG}"
  -r "/livox/lidar_192_168_1_5:=${RAW_LIDAR5_TOPIC}"
  -r "/livox/lidar_192_168_1_3:=${RAW_LIDAR3_TOPIC}"
  -r "/livox/imu_192_168_1_5:=${RAW_IMU5_TOPIC}"
  -r "/livox/imu_192_168_1_3:=${RAW_IMU3_TOPIC}"
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

if [[ "${RECORD_ROSBAG}" == "1" ]]; then
  ROSBAG_TOPICS=(
    "${RAW_LIDAR5_TOPIC}"
    "${RAW_LIDAR3_TOPIC}"
    "${RAW_IMU5_TOPIC}"
    "${RAW_IMU3_TOPIC}"
    /gimbal/cloud_fused
    /gimbal/imu_fused
    /gimbal/imu_calibrated
    /point_lio/odom
    /dlio/odom_node/odom
    /path
    /cloud_registered
    /cloud_registered_body
    /fusion_pcl
    /tf
    /tf_static
    /parameter_events
    /rosout
  )
  mkdir -p "$(dirname -- "${ROSBAG_OUTPUT}")"
  echo "[start_odom] Recording diagnostic rosbag: ${ROSBAG_OUTPUT}"
  setsid ros2 bag record \
    --output "${ROSBAG_OUTPUT}" \
    --storage sqlite3 \
    "${ROSBAG_TOPICS[@]}" &
  BAG_PID=$!
  sleep 1
  if ! kill -0 "${BAG_PID}" 2>/dev/null; then
    echo "[start_odom] Rosbag recorder exited during startup." >&2
    wait "${BAG_PID}"
  fi
fi

echo "[start_odom] Starting point-cloud and IMU fusion..."
setsid ros2 run fusion_ws fusion_pcl --ros-args \
  -p imu_accel_unit:=auto \
  -p "cloud_voxel_leaf_size:=${FUSION_CLOUD_VOXEL_LEAF_SIZE}" \
  -p "lidar5_topic:=${RAW_LIDAR5_TOPIC}" \
  -p "lidar3_topic:=${RAW_LIDAR3_TOPIC}" \
  -p "imu5_topic:=${RAW_IMU5_TOPIC}" \
  -p "imu3_topic:=${RAW_IMU3_TOPIC}" &
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

echo "[start_odom] Starting fused ${ODOM_NAME} odometry..."
if [[ "${ODOM_ALGORITHM}" == "dlio" ]]; then
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
else
  setsid ros2 run plio point_lio --ros-args \
    --params-file "${ODOM_PARAMS}" \
    -r pointcloud:=/gimbal/cloud_fused \
    -r imu:=/gimbal/imu_fused \
    -r odom:=/point_lio/odom \
    -r path:=/path \
    -r registered:=/cloud_registered \
    -r registered_body:=/cloud_registered_body &
fi
ODOM_PID=$!

sleep 1
if ! kill -0 "${ODOM_PID}" 2>/dev/null; then
  echo "[start_odom] ${ODOM_NAME} odometry exited during startup." >&2
  wait "${ODOM_PID}"
fi

if [[ "${ODOM_ALGORITHM}" == "dlio" ]]; then
  echo "[start_odom] DLIO keyframe cloud: /dlio/odom_node/pointcloud/keyframe"
else
  echo "[start_odom] Point-LIO topics: /point_lio/odom, /path, /cloud_registered"
fi
if [[ "${ENABLE_GTSAM}" == "1" ]]; then
  echo "[start_odom] GTSAM enabled. Optimized map is written only on save."
  echo "[start_odom] Topics: /mapping/optimized_path, /loop_closure/constraint, /tf"
  echo "[start_odom] Save map: ros2 service call /mapping/save_map std_srvs/srv/Trigger '{}'"
else
  echo "[start_odom] GTSAM disabled. ${ODOM_NAME} Fixed Frame: odom"
fi
echo "[start_odom] Press Ctrl+C to stop all nodes."

# Returning when any child exits prevents a partially running pipeline.
PIPELINE_PIDS=("${DRIVER_PID}" "${FUSION_PID}" "${ODOM_PID}")
if [[ "${RECORD_ROSBAG}" == "1" ]]; then
  PIPELINE_PIDS+=("${BAG_PID}")
fi
if [[ "${ENABLE_GTSAM}" == "1" ]]; then
  PIPELINE_PIDS+=("${LOOP_PID}" "${BACKEND_PID}")
fi
wait -n "${PIPELINE_PIDS[@]}"
