#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="${SCRIPT_DIR}"
cd "${ROOT_DIR}"

ROS_SETUP="/opt/ros/humble/setup.bash"
SLAM_SETUP="${ROOT_DIR}/slam/install/setup.bash"
MAP_PARAMS="${MAP_PARAMS:-${ROOT_DIR}/slam/config/map.yaml}"
ENABLE_GTSAM="${ENABLE_GTSAM:-1}"
AUTO_SAVE_MAPS="${AUTO_SAVE_MAPS:-1}"
MAP_SAVE_TIMEOUT="${MAP_SAVE_TIMEOUT:-300}"
export ENABLE_GTSAM

if [[ "${AUTO_SAVE_MAPS}" != "0" && "${AUTO_SAVE_MAPS}" != "1" ]]; then
  echo "[start_mapping] AUTO_SAVE_MAPS must be 0 or 1." >&2
  exit 1
fi

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
SAVE_PID=""
WATCHDOG_PID=""
SAVE_REQUESTED=0

stop_process_group() {
  local pid="$1"
  local signal="$2"
  if [[ -z "${pid}" ]]; then
    return
  fi

  # The session leader may already have exited while other members of its
  # process group are still alive, so probe the group before probing the PID.
  if kill -0 -- "-${pid}" 2>/dev/null; then
    kill "-${signal}" -- "-${pid}" 2>/dev/null || true
  elif kill -0 "${pid}" 2>/dev/null; then
    kill "-${signal}" "${pid}" 2>/dev/null || true
  fi
}

call_save_service() {
  local description="$1"
  local service="$2"

  printf '\n[start_mapping] Saving %s before shutdown...\n' "${description}" >&2
  setsid timeout --signal=TERM --kill-after=5s "${MAP_SAVE_TIMEOUT}" \
    ros2 service call "${service}" std_srvs/srv/Trigger '{}' &
  SAVE_PID=$!
  if wait "${SAVE_PID}"; then
    printf '[start_mapping] Finished saving %s.\n' "${description}" >&2
  else
    printf '[start_mapping] %s save failed or timed out.\n' \
      "${description}" >&2
  fi
  SAVE_PID=""
}

save_maps() {
  call_save_service "KD-tree map" /dlio/save_kdtree_map
  if [[ "${ENABLE_GTSAM}" == "1" ]]; then
    call_save_service "GTSAM optimized map" /mapping/save_map
  fi
}

request_shutdown() {
  printf '\n[start_mapping] Shutdown requested; preparing to save maps.\n' >&2
  SAVE_REQUESTED=1
  exit "$1"
}

force_cleanup() {
  local status="$1"
  trap - EXIT
  trap '' INT TERM
  printf '\n[start_mapping] Forced shutdown requested; stopping all processes.\n' >&2
  stop_process_group "${SAVE_PID}" KILL
  # Let start_odom run its own cleanup so its separately grouped children do
  # not survive the supervisor.
  stop_process_group "${PIPELINE_PID}" TERM
  stop_process_group "${MAP_PID}" KILL
  sleep 2
  stop_process_group "${PIPELINE_PID}" KILL
  stop_watchdog
  wait 2>/dev/null || true
  exit "${status}"
}

supervisor_watchdog() {
  local supervisor_pid="$1"
  local map_pid="$2"
  local pipeline_pid="$3"

  trap '' INT
  trap 'exit 0' TERM
  while kill -0 "${supervisor_pid}" 2>/dev/null; do
    sleep 1
  done

  printf '\n[start_mapping] Supervisor exited unexpectedly; cleaning up child processes.\n' >&2
  stop_process_group "${pipeline_pid}" TERM
  stop_process_group "${map_pid}" TERM
  sleep 2
  stop_process_group "${pipeline_pid}" KILL
  stop_process_group "${map_pid}" KILL
}

stop_watchdog() {
  if [[ -n "${WATCHDOG_PID}" ]] && kill -0 "${WATCHDOG_PID}" 2>/dev/null; then
    kill -TERM "${WATCHDOG_PID}" 2>/dev/null || true
    wait "${WATCHDOG_PID}" 2>/dev/null || true
  fi
  WATCHDOG_PID=""
}

cleanup() {
  local status=$?
  trap - EXIT
  trap 'force_cleanup 130' INT
  trap 'force_cleanup 143' TERM

  if [[ "${SAVE_REQUESTED}" == "1" && "${AUTO_SAVE_MAPS}" == "1" ]]; then
    save_maps
  fi

  printf '\n[start_mapping] Stopping mapping processes...\n' >&2
  stop_process_group "${PIPELINE_PID}" TERM
  stop_process_group "${MAP_PID}" TERM
  # start_odom performs its own one-second graceful shutdown before exiting.
  sleep 2
  stop_process_group "${PIPELINE_PID}" KILL
  stop_process_group "${MAP_PID}" KILL
  stop_watchdog
  wait 2>/dev/null || true
  exit "${status}"
}
trap cleanup EXIT
trap 'request_shutdown 130' INT
trap 'request_shutdown 143' TERM

echo "[start_mapping] Supervisor PID: $$"
if [[ -t 0 ]]; then
  SUPERVISOR_PGID="$(ps -o pgid= -p "$$" | tr -d ' ')"
  TERMINAL_PGID="$(ps -o tpgid= -p "$$" | tr -d ' ')"
  if [[ -n "${SUPERVISOR_PGID}" && "${SUPERVISOR_PGID}" != "${TERMINAL_PGID}" ]]; then
    echo "[start_mapping] Warning: supervisor is not the terminal foreground process." >&2
    echo "[start_mapping] Ctrl+C will not reach it; run this script in the foreground." >&2
  fi
fi

echo "[start_mapping] Starting DLIO KD-tree map builder..."
setsid ros2 run map_ws kdtree_map --ros-args \
  --params-file "${MAP_PARAMS}" \
  -r keyframe:=/dlio/odom_node/pointcloud/keyframe \
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

supervisor_watchdog "$$" "${MAP_PID}" "${PIPELINE_PID}" &
WATCHDOG_PID=$!

echo "[start_mapping] Dense KD-tree map is held in memory until saved."
if [[ "${ENABLE_GTSAM}" == "1" ]]; then
  echo "[start_mapping] Dense GTSAM map is generated only when saved."
fi
echo "[start_mapping] Save KD map: ros2 service call /dlio/save_kdtree_map std_srvs/srv/Trigger '{}'"
echo "[start_mapping] Clear KD map: ros2 service call /dlio/clear_kdtree_map std_srvs/srv/Trigger '{}'"
echo "[start_mapping] Ctrl+C will save available maps before stopping."

wait -n "${MAP_PID}" "${PIPELINE_PID}"
