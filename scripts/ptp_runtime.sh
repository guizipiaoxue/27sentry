#!/usr/bin/env bash
# Shared live-sensor preflight. PTP daemons are managed externally and reused.

ptp_check_host() {
  local root_dir="$1"
  local lidar_config="$2"
  PTP_UTC_OFFSET="${PTP_UTC_OFFSET:-37}"
  PTP_CHECK_TIMEOUT="${PTP_CHECK_TIMEOUT:-30}"
  PTP_LOCK_TIMEOUT="${PTP_LOCK_TIMEOUT:-30}"
  if [[ ! "${PTP_UTC_OFFSET}" =~ ^[0-9]+$ ]]; then
    echo "[ptp] PTP_UTC_OFFSET must be a nonnegative integer (hardware TAI: 37; software UTC: 0)." >&2
    return 1
  fi
  python3 "${root_dir}/scripts/ptp_check.py" \
    --config "${lidar_config}" --utc-offset "${PTP_UTC_OFFSET}" \
    --timeout "${PTP_CHECK_TIMEOUT}"
}

ptp_wait_sensors() {
  echo "[ptp] Waiting for every configured lidar and IMU to report valid PTP/UTC samples..."
  if ! timeout "${PTP_LOCK_TIMEOUT:-30}" \
      ros2 topic echo /livox/ptp_locked std_msgs/msg/Bool \
        --once --filter 'm.data' \
        --qos-reliability reliable --qos-durability transient_local \
      | grep -Fq "data: true"; then
    local helper_dir
    helper_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
    if ! python3 "${helper_dir}/check_sensor_time.py" --diagnose-lock --duration 1; then
      echo "[ptp] Sensor PTP lock wait failed; status diagnosis was unavailable. Check the ROS environment and driver logs." >&2
    fi
    return 1
  fi
}
