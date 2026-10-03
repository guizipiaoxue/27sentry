#!/usr/bin/env python3
"""Initialize only the NIC PHC before the project PTP services start.

An already aligned PHC is left untouched, including while sensors run. A
misaligned PHC is stepped only when no known live sensor/LIO process is found.
CLOCK_REALTIME is always a read-only source and is never adjusted here.
"""

import argparse
import json
import math
import os
from pathlib import Path
import re
import time


class BootstrapError(RuntimeError):
    pass


def config_offset(path, interface):
    section = "global"
    values = {"global": {}}
    for raw in Path(path).read_text().splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        if line.startswith("[") and line.endswith("]"):
            section = line[1:-1]
            values.setdefault(section, {})
        else:
            parts = line.split(None, 1)
            if len(parts) != 2:
                raise BootstrapError(f"invalid PTP config line: {raw}")
            values[section][parts[0]] = parts[1]
    if interface not in values:
        raise BootstrapError(f"PTP config has no [{interface}] port")
    settings = {**values["global"], **values[interface]}
    if settings.get("time_stamping", "hardware") != "hardware":
        raise BootstrapError("PHC bootstrap only supports hardware timestamping")
    if settings.get("masterOnly", settings.get("serverOnly", "0")) != "1":
        raise BootstrapError("PHC bootstrap requires a fixed PTP MASTER port")
    offset = int(values["global"].get("utc_offset", "37"))
    if not 0 <= offset <= 255:
        raise BootstrapError("utc_offset must be in 0..255 seconds")
    return offset


def phc_device(interface, wait_seconds):
    base = Path("/sys/class/net") / interface
    deadline = time.monotonic() + wait_seconds
    while True:
        if base.exists():
            try:
                if (base / "carrier").read_text().strip() == "1":
                    clocks = list((base / "device/ptp").glob("ptp[0-9]*"))
                    if len(clocks) != 1:
                        raise BootstrapError(f"{interface}: expected one NIC PHC, found {len(clocks)}")
                    return Path("/dev") / clocks[0].name
            except OSError:
                pass
        if time.monotonic() >= deadline:
            raise BootstrapError(f"{interface}: interface/carrier not ready within {wait_seconds:g} seconds")
        time.sleep(min(0.2, max(0, deadline - time.monotonic())))


def active_acquisition():
    # Examine executable/arguments without invoking, stopping or signalling them.
    names = re.compile(r"(?:^|[/\s])(?:livox_ros_driver2_node|livox_lidar_publisher|"
                       r"fusion_pcl|single_lidar_odom|point_lio|odom|"
                       r"rosbag2_recorder|rosbag2_player)(?:$|[\s/])")
    found = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            arguments = entry.joinpath("cmdline").read_bytes().decode(errors="replace").replace("\0", " ").strip()
            if names.search(arguments) or re.search(r"(?:^|[/\s])ros2\s+bag\s+(?:record|play)(?:\s|$)", arguments):
                found.append(int(entry.name))
        except (FileNotFoundError, ProcessLookupError):
            continue
        except PermissionError as error:
            raise BootstrapError(f"cannot inspect PID {entry.name}; refusing PHC initialization") from error
    return found


def argument_value(arguments, flag):
    for index, argument in enumerate(arguments):
        if argument == flag and index + 1 < len(arguments):
            return arguments[index + 1]
        if argument.startswith(flag) and len(argument) > len(flag):
            return argument[len(flag):]
    return None


def active_clock_writers(device, interface):
    """Reject another process that might write this PHC before a step."""
    found = []

    def same_clock(target):
        if not target or target == "CLOCK_REALTIME":
            return False
        if target == interface or Path(target).resolve() == Path(device).resolve():
            return True
        clock_directory = Path("/sys/class/net") / target / "device/ptp"
        return any(clock.name == Path(device).name for clock in clock_directory.glob("ptp[0-9]*"))

    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            arguments = [part.decode(errors="replace") for part in
                         entry.joinpath("cmdline").read_bytes().split(b"\0") if part]
            if not arguments:
                continue
            executable = Path(arguments[0]).name
            if executable not in ("ptp4l", "phc2sys", "ts2phc", "phc_ctl"):
                continue
            targets = []
            if executable == "phc2sys":
                if any(argument == "-a" or argument.startswith("-ar") for argument in arguments):
                    found.append({"pid": int(entry.name), "program": executable, "target": "automatic"})
                    continue
                targets.append(argument_value(arguments, "-c"))
            elif executable == "phc_ctl":
                targets.extend(argument for argument in arguments[1:] if not argument.startswith("-"))
            else:
                for index, argument in enumerate(arguments):
                    if argument in ("-i", "-p", "-c") and index + 1 < len(arguments):
                        targets.append(arguments[index + 1])
                configuration = argument_value(arguments, "-f")
                if configuration:
                    sections = re.findall(r"^\s*\[([^\]]+)\]", Path(configuration).read_text(), re.MULTILINE)
                    targets.extend(section for section in sections if section != "global")
            if any(same_clock(target) for target in targets) or (executable == "ts2phc" and not targets):
                found.append({"pid": int(entry.name), "program": executable, "target": "same PHC or unknown target"})
        except (FileNotFoundError, ProcessLookupError):
            continue
        except PermissionError as error:
            raise BootstrapError(f"cannot inspect clock writer PID {entry.name}; refusing PHC initialization") from error
    return found


def clock_id(descriptor):
    result = ((~descriptor) << 3) | 3  # Linux FD_TO_CLOCKID.
    if descriptor < 0 or result >= 0:
        raise BootstrapError("invalid dynamic PHC clock id")
    return result


def sample(clock, offset):
    before = time.time_ns()
    hardware = time.clock_gettime_ns(clock)
    after = time.time_ns()
    if after < before:
        raise BootstrapError("CLOCK_REALTIME moved backwards during PHC read")
    return {"phc_minus_utc_ns": hardware - (before + after) // 2,
            "error_ns": hardware - (before + after) // 2 - offset * 1_000_000_000,
            "read_uncertainty_ns": (after - before) // 2}


def precise_sample(clock, offset, tolerance):
    samples = [sample(clock, offset) for _ in range(5)]
    result = min(samples, key=lambda item: item["read_uncertainty_ns"])
    if result["read_uncertainty_ns"] > tolerance // 2:
        raise BootstrapError("PHC read uncertainty is too large to decide whether initialization is safe")
    return result


def initialize(device, offset, tolerance, dry_run=False, interface=None):
    read_descriptor = os.open(device, os.O_RDONLY | os.O_CLOEXEC)
    try:
        initial = precise_sample(clock_id(read_descriptor), offset, tolerance)
    finally:
        os.close(read_descriptor)
    report = {"device": str(device), "utc_offset_seconds": offset,
              "system_clock_changed": False, "before": initial}
    if abs(initial["error_ns"]) + initial["read_uncertainty_ns"] <= tolerance:
        return {**report, "ok": True, "action": "already_aligned_no_step"}
    running = active_acquisition()
    if running:
        raise BootstrapError(f"PHC error {initial['error_ns']} ns exceeds {tolerance} ns while acquisition PIDs {running} run; refusing PHC step")
    writers = active_clock_writers(device, interface)
    if writers:
        raise BootstrapError(f"existing PHC clock writers {writers}; refusing concurrent PHC step")
    if dry_run:
        return {**report, "ok": True, "action": "would_initialize_phc_only"}
    write_descriptor = os.open(device, os.O_RDWR | os.O_CLOEXEC)
    try:
        target_clock = clock_id(write_descriptor)
        # Recheck after obtaining write access; an already aligned PHC stays untouched.
        current = precise_sample(target_clock, offset, tolerance)
        if abs(current["error_ns"]) + current["read_uncertainty_ns"] <= tolerance:
            return {**report, "ok": True, "action": "already_aligned_no_step", "after": current}
        running = active_acquisition()
        if running:
            raise BootstrapError(f"acquisition PIDs {running} started during PHC initialization; refusing step")
        writers = active_clock_writers(device, interface)
        if writers:
            raise BootstrapError(f"PHC clock writers {writers} appeared during initialization; refusing concurrent step")
        # This negative FD clock id refers only to /dev/ptpN, never CLOCK_REALTIME.
        time.clock_settime_ns(target_clock, time.time_ns() + offset * 1_000_000_000)
        final = precise_sample(target_clock, offset, tolerance)
        if abs(final["error_ns"]) + final["read_uncertainty_ns"] > tolerance:
            raise BootstrapError(f"PHC remains outside tolerance after initialization: {final['error_ns']} ns")
        return {**report, "ok": True, "action": "initialized_phc_only", "after": final}
    finally:
        os.close(write_descriptor)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--max-offset-ms", type=float, default=1.0)
    parser.add_argument("--wait-seconds", type=float, default=30.0)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    if not re.fullmatch(r"[A-Za-z0-9_.:-]{1,15}", args.interface):
        parser.error("invalid interface name")
    if not math.isfinite(args.max_offset_ms) or args.max_offset_ms <= 0:
        parser.error("max-offset-ms must be finite and positive")
    if not math.isfinite(args.wait_seconds) or args.wait_seconds < 0:
        parser.error("wait-seconds must be finite and nonnegative")
    try:
        offset = config_offset(args.config, args.interface)
        device = phc_device(args.interface, args.wait_seconds)
        report = initialize(device, offset, round(args.max_offset_ms * 1e6), args.dry_run, args.interface)
    except (BootstrapError, OSError, ValueError) as error:
        print(json.dumps({"ok": False, "error": str(error), "system_clock_changed": False}))
        return 1
    print(json.dumps(report))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
