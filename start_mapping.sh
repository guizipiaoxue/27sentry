#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="${SCRIPT_DIR}"

ROS_SETUP="/opt/ros/humble/setup.bash"
SLAM_SETUP="${ROOT_DIR}/slam/install/setup.bash"
MAP_PARAMS="${MAP_PARAMS:-${ROOT_DIR}/slam/config/map.yaml}"
ENABLE_GTSAM="${ENABLE_GTSAM:-1}"
export ENABLE_GTSAM

for required_file in "${ROS_SETUP}" "${SLAM_SETUP}" "${MAP_PARAMS}"; do
  if [[ ! -f "${required_file}" ]]; then
    echo "[start_mapping] Missing file: ${required_file}" >&2
    exit 1
  fi
done

set +u
source "${ROS_SETUP}"
source "${SLAM_SETUP}"
set -u

if ! ros2 pkg executables map_ws | awk '{print $2}' | grep -Fxq kdtree_map; then
  echo "[start_mapping] map_ws/kdtree_map is not built or sourced." >&2
  echo "[start_mapping] Build the slam workspace first." >&2
  exit 1
fi

MAP_PID=""
PIPELINE_PID=""

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
  stop_process_group "${PIPELINE_PID}" TERM
  stop_process_group "${MAP_PID}" TERM
  sleep 1
  stop_process_group "${PIPELINE_PID}" KILL
  stop_process_group "${MAP_PID}" KILL
  wait 2>/dev/null || true
  exit "${status}"
}
trap cleanup EXIT INT TERM

echo "[start_mapping] Starting DLIO KD-tree map builder..."
setsid ros2 run map_ws kdtree_map --ros-args \
  --params-file "${MAP_PARAMS}" \
  -r keyframe:=/dlio/odom_node/pointcloud/keyframe \
  -r map:=/dlio/kdtree_map \
  -r save_map:=/dlio/save_kdtree_map \
  -r clear_map:=/dlio/clear_kdtree_map &
MAP_PID=$!

sleep 1
if ! kill -0 "${MAP_PID}" 2>/dev/null; then
  echo "[start_mapping] KD-tree map builder exited during startup." >&2
  wait "${MAP_PID}"
fi

if [[ "${ENABLE_GTSAM}" == "1" ]]; then
  echo "[start_mapping] Starting DLIO and GTSAM pipeline..."
else
  echo "[start_mapping] Starting DLIO pipeline without GTSAM..."
fi
setsid "${ROOT_DIR}/start_odom.sh" "$@" &
PIPELINE_PID=$!

echo "[start_mapping] Raw DLIO map: /dlio/kdtree_map"
if [[ "${ENABLE_GTSAM}" == "1" ]]; then
  echo "[start_mapping] GTSAM optimized map: /mapping/map"
fi
echo "[start_mapping] Save KD map: ros2 service call /dlio/save_kdtree_map std_srvs/srv/Trigger '{}'"
echo "[start_mapping] Clear KD map: ros2 service call /dlio/clear_kdtree_map std_srvs/srv/Trigger '{}'"

wait -n "${MAP_PID}" "${PIPELINE_PID}"
