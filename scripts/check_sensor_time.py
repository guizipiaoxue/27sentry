#!/usr/bin/env python3
"""Read-only verification of live Livox PTP/UTC headers and stream health."""

import argparse
from collections import deque
import json
import math
import sys
import time


class StreamWindow:
    def __init__(self, max_age_seconds):
        self.max_age_ns = int(max_age_seconds * 1_000_000_000)
        self.last_stamp = 0
        self.stamps = deque(maxlen=4096)
        self.count = 0
        self.errors = {}
        self.age_min_ns = None
        self.age_max_ns = None
        self.last_arrival = None

    def reject(self, reason):
        self.errors[reason] = self.errors.get(reason, 0) + 1

    def observe(self, sec, nanosec, arrival_ns, monotonic_now, timebase=None):
        self.count += 1
        self.last_arrival = monotonic_now
        if sec <= 0 or not 0 <= nanosec < 1_000_000_000:
            self.reject("invalid_header")
            return
        stamp = sec * 1_000_000_000 + nanosec
        if timebase is not None and timebase != stamp:
            self.reject("header_timebase_mismatch")
        if stamp <= self.last_stamp:
            self.reject("non_increasing_stamp")
        self.last_stamp = max(stamp, self.last_stamp)
        age = arrival_ns - stamp
        if abs(age) > self.max_age_ns:
            self.reject("outside_host_utc_window")
        self.age_min_ns = age if self.age_min_ns is None else min(age, self.age_min_ns)
        self.age_max_ns = age if self.age_max_ns is None else max(age, self.age_max_ns)
        self.stamps.append(stamp)

    def report(self, monotonic_now):
        if self.last_arrival is None:
            self.reject("missing_stream")
        elif monotonic_now - self.last_arrival > 2.0:
            self.reject("stale_stream")
        return {
            "samples": self.count,
            "arrival_minus_stamp_ms": None if self.age_min_ns is None else [
                self.age_min_ns / 1e6, self.age_max_ns / 1e6],
            "errors": self.errors,
        }


def paired_deltas(left, right, tolerance_ns):
    """Consume scan pairs once, as the production fusion queue does."""
    left, right = deque(left), deque(right)
    deltas = []
    dropped = [0, 0]
    while left and right:
        delta = left[0] - right[0]
        if abs(delta) <= tolerance_ns:
            deltas.append(delta / 1e6)
            left.popleft()
            right.popleft()
        elif delta < 0:
            left.popleft()
            dropped[0] += 1
        else:
            right.popleft()
            dropped[1] += 1
    return {"pairs": len(deltas), "max_abs_delta_ms": max(map(abs, deltas), default=None),
            "unpaired_samples": [dropped[0] + len(left), dropped[1] + len(right)]}


def lock_diagnostic_message(publisher_count, value):
    if publisher_count == 0:
        return ("No publisher for /livox/ptp_locked was discovered. The loaded Livox driver may be old; "
                "successfully rebuild livox/install and restart with the updated driver. "
                "Also check ROS_DOMAIN_ID and the sourced workspace. Raw lidar/IMU topics alone do not prove PTP lock.")
    if value is False:
        return ("/livox/ptp_locked has a publisher but reports false. Check the driver's per-device timing diagnostics, "
                "time_type=1, UTC offset, and fresh lidar/IMU samples from every configured device.")
    if value is True:
        return ("/livox/ptp_locked became true after the wait failed. Startup remains refused; "
                "retry after checking the driver's lock stability.")
    return ("/livox/ptp_locked has a publisher but no compatible status sample was received. "
            "Check the publisher's health and reliable/transient_local QoS.")


def query_lock_status(duration):
    """Read the DDS graph directly; no ros2 CLI daemon is consulted."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool

    rclpy.init(args=[])
    node = Node("diagnose_livox_ptp_lock")
    status = {"value": None}
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscription = node.create_subscription(
        Bool, "/livox/ptp_locked", lambda message: status.update(value=message.data), qos)
    deadline = time.monotonic() + duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
        return node.count_publishers("/livox/ptp_locked"), status["value"]
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--max-age", type=float, default=2.0,
                        help="maximum absolute host UTC/header difference, seconds")
    parser.add_argument("--max-pair-delta-ms", type=float, default=30.0)
    parser.add_argument("--raw-prefix", default="/sentry/raw")
    parser.add_argument("--include-fused", action="store_true")
    parser.add_argument("--diagnose-lock", action="store_true",
                        help="diagnose a failed lock wait without changing the startup gate")
    args = parser.parse_args()
    if any(not math.isfinite(value) or value <= 0 for value in
           (args.duration, args.max_age, args.max_pair_delta_ms)):
        parser.error("duration and limits must be finite and positive")

    if args.diagnose_lock:
        publishers, value = query_lock_status(args.duration)
        print("[ptp] Sensor PTP lock wait failed: " + lock_diagnostic_message(publishers, value), file=sys.stderr)
        return 0

    import rclpy
    from livox_ros_driver2.msg import CustomMsg
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import Imu, PointCloud2
    from std_msgs.msg import Bool

    rclpy.init(args=[])
    node = Node("check_livox_sensor_time")
    streams = {}
    subscriptions = []
    lock = {"value": False, "arrival": None, "false_samples": 0}
    started = time.monotonic()

    def add_stream(topic, message_type):
        window = StreamWindow(args.max_age)
        streams[topic] = window

        def observe(message):
            window.observe(message.header.stamp.sec, message.header.stamp.nanosec,
                           time.time_ns(), time.monotonic(), getattr(message, "timebase", None))

        subscriptions.append(node.create_subscription(message_type, topic, observe, qos_profile_sensor_data))

    def observe_lock(message):
        lock["value"] = message.data
        lock["arrival"] = time.monotonic()
        if not message.data and time.monotonic() - started > 2.0:
            lock["false_samples"] += 1

    status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions.append(node.create_subscription(Bool, "/livox/ptp_locked", observe_lock, status_qos))
    prefix = args.raw_prefix.rstrip("/")
    for name, message_type in (("lidar5", CustomMsg), ("lidar3", CustomMsg),
                               ("imu5", Imu), ("imu3", Imu)):
        add_stream(f"{prefix}/{name}", message_type)
    if args.include_fused:
        add_stream("/gimbal/cloud_fused", PointCloud2)
        add_stream("/gimbal/imu_fused", Imu)

    try:
        while rclpy.ok() and time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
        now = time.monotonic()
        reports = {topic: window.report(now) for topic, window in streams.items()}
        status_ok = (lock["value"] and lock["arrival"] is not None
                     and now - lock["arrival"] <= 2.0 and not lock["false_samples"])
        pair_report = paired_deltas(streams[f"{prefix}/lidar5"].stamps,
                                    streams[f"{prefix}/lidar3"].stamps,
                                    int(args.max_pair_delta_ms * 1e6))
        passed = (status_ok and all(not report["errors"] for report in reports.values())
                  and pair_report["pairs"] > 0)
        print(json.dumps({"ok": passed, "ptp_locked": bool(status_ok), "streams": reports,
                          "lidar_pairing": pair_report,
                          "note": "Header pairing measures scan phase/coverage; it is not a hardware PTP accuracy measurement."},
                         indent=2))
        return 0 if passed else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
