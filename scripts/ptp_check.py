#!/usr/bin/env python3
"""Read-only PTP grandmaster checks before live MID360 acquisition.

Never starts/stops a service or changes a clock/network interface. If local
management sockets or PHCs are inaccessible, existing systemd configuration
and logs provide explicitly labelled evidence; --strict-pmc requires direct
management and PHC reads instead.
"""

import argparse
import ctypes
import fcntl
import ipaddress
import json
import math
import os
from pathlib import Path
import re
import socket
import subprocess
import tempfile
import time


class CheckError(RuntimeError):
    pass


def run(command, timeout=5):
    result = subprocess.run(command, capture_output=True, text=True,
                            timeout=timeout, env={**os.environ, "LC_ALL": "C"})
    if result.returncode:
        detail = result.stderr.strip() or result.stdout.strip()
        raise CheckError(f"{' '.join(command)}: {detail or 'command failed'}")
    return result.stdout


def option(arguments, names, default=None):
    for index, argument in enumerate(arguments):
        for name in names:
            if argument == name and index + 1 < len(arguments):
                return arguments[index + 1]
            if argument.startswith(name + "="):
                return argument[len(name) + 1:]
            if len(name) == 2 and argument.startswith(name) and len(argument) > 2:
                return argument[2:]
    return default


def read_ptp_config(path):
    sections = {"global": {}}
    section = "global"
    if path is None:
        return sections
    for line in Path(path).read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        if line.startswith("[") and line.endswith("]"):
            section = line[1:-1]
            sections.setdefault(section, {})
            continue
        parts = line.split(None, 1)
        if len(parts) != 2:
            raise CheckError(f"invalid PTP setting in {path}: {line}")
        sections[section][parts[0]] = parts[1].strip()
    return sections


def setting(process, name, default, interface=None):
    arguments, config = process["arguments"], process["config"]
    value = option(arguments, ["--" + name])
    if value is not None:
        return value
    if interface and name in config.get(interface, {}):
        return config[interface][name]
    return config["global"].get(name, default)


def processes():
    result = {"ptp4l": [], "phc2sys": []}
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            arguments = entry.joinpath("cmdline").read_bytes().split(b"\0")
            arguments = [argument.decode() for argument in arguments if argument]
            if not arguments or Path(arguments[0]).name not in result:
                continue
            name = Path(arguments[0]).name
            process = {"pid": int(entry.name), "arguments": arguments,
                       "config": read_ptp_config(option(arguments, ["-f"]))}
            interfaces = list(key for key in process["config"] if key != "global")
            for index, argument in enumerate(arguments):
                if argument == "-i" and index + 1 < len(arguments):
                    interfaces.append(arguments[index + 1])
                elif argument.startswith("-i") and len(argument) > 2:
                    interfaces.append(argument[2:])
            process["interfaces"] = list(dict.fromkeys(interfaces))
            result[name].append(process)
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
    return result


def lidar_routes(path):
    config = json.loads(Path(path).read_text())
    lidars = config.get("lidar_configs", [])
    if not lidars:
        raise CheckError(f"no lidar_configs in {path}")
    hosts = []
    for section in config.values():
        if isinstance(section, dict) and "host_net_info" in section:
            values = section["host_net_info"]
            hosts.extend(values if isinstance(values, list) else [values])
    if not hosts:
        raise CheckError(f"no host_net_info in {path}")
    result = []
    for lidar in lidars:
        address = str(ipaddress.IPv4Address(lidar["ip"]))
        routes = json.loads(run(["ip", "-j", "route", "get", address]))
        if not routes or not routes[0].get("dev"):
            raise CheckError(f"no network route to lidar {address}")
        route = routes[0]
        interface, source = route["dev"], route.get("prefsrc", route.get("src"))
        if route.get("gateway"):
            raise CheckError(f"lidar {address} is routed through a gateway; PTP UDP multicast requires a shared link")
        if not source or not any(all(host.get(key) == source for key in
                                   ("point_data_ip", "imu_data_ip")) for host in hosts):
            raise CheckError(f"lidar {address}: route source {source} does not match configured point/IMU host IP")
        carrier = Path("/sys/class/net") / interface / "carrier"
        if carrier.read_text().strip() != "1":
            raise CheckError(f"lidar {address}: interface {interface} has no carrier")
        result.append({"lidar": address, "interface": interface, "host_ip": source})
    return result


class TimestampInfo(ctypes.Structure):
    _fields_ = [("cmd", ctypes.c_uint32), ("so_timestamping", ctypes.c_uint32),
                ("phc_index", ctypes.c_int32), ("tx_types", ctypes.c_uint32),
                ("tx_reserved", ctypes.c_uint32 * 3), ("rx_filters", ctypes.c_uint32),
                ("rx_reserved", ctypes.c_uint32 * 3)]


class InterfaceRequest(ctypes.Structure):
    _fields_ = [("name", ctypes.c_char * 16), ("data", ctypes.c_void_p),
                ("padding", ctypes.c_char * 16)]


def timestamp_info(interface):
    info = TimestampInfo(cmd=0x41)  # ETHTOOL_GET_TS_INFO: read-only ioctl.
    request = InterfaceRequest(name=interface.encode(), data=ctypes.addressof(info))
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as connection:
        fcntl.ioctl(connection.fileno(), 0x8946, request)  # SIOCETHTOOL.
    return {"phc_index": info.phc_index, "so_timestamping": info.so_timestamping}


def journal(process, count=200):
    cgroup = Path(f"/proc/{process['pid']}/cgroup").read_text()
    units = re.findall(r"/([^/\n]+\.service)(?:/|$)", cgroup, re.MULTILINE)
    if not units:
        raise CheckError(f"PID {process['pid']} has no systemd journal evidence; grant read access to its PTP socket/PHC")
    unit = units[-1]
    state = dict(line.split("=", 1) for line in run([
        "systemctl", "show", unit, "--no-pager", "-p", "ActiveState", "-p", "SubState",
        "-p", "MainPID", "-p", "InvocationID"]).splitlines() if "=" in line)
    if (state.get("ActiveState") != "active" or state.get("SubState") != "running"
            or state.get("MainPID") != str(process["pid"]) or not state.get("InvocationID")):
        raise CheckError(f"PTP service {unit} is not running the inspected PID")
    invocation = state["InvocationID"]
    output = run(["journalctl", "-b", "--no-pager", "--quiet", "-o", "json",
                  "-n", str(count), "_SYSTEMD_INVOCATION_ID=" + invocation])
    entries = [json.loads(line) for line in output.splitlines()]
    return unit, invocation, entries


def management(process, domain):
    address = setting(process, "uds_address", "/var/run/ptp4l")
    if Path(address).exists() and not os.access(address, os.W_OK):
        return None, f"Permission denied: PTP management socket {address}"
    try:
        with tempfile.TemporaryDirectory(prefix="sentry-ptp-") as directory:
            output = run(["pmc", "-u", "-i", directory + "/client", "-s", address,
                          "-d", str(domain), "-b", "0", "GET TIME_PROPERTIES_DATA_SET",
                          "GET PORT_DATA_SET", "GET PORT_PROPERTIES_NP"], timeout=7)
        if "RESPONSE MANAGEMENT TIME_PROPERTIES_DATA_SET" not in output:
            raise CheckError("PTP management socket did not respond")
        return output, None
    except (CheckError, OSError, subprocess.TimeoutExpired) as error:
        return None, str(error)


def check_port(output, interface, port_number):
    # PORT_PROPERTIES_NP names the interface; it provides the exact port state.
    blocks = re.split(r"(?=\S+\s+seq\s+\d+\s+RESPONSE)", output)
    for block in blocks:
        if "PORT_PROPERTIES_NP" in block and re.search(
                r"\binterface\s+" + re.escape(interface) + r"\b", block):
            match = re.search(r"\bportState\s+(\S+)", block)
            if not match or match[1] != "MASTER":
                raise CheckError(f"{interface}: PMC port state is {match[1] if match else 'unknown'}")
            return
    for block in blocks:
        if "PORT_DATA_SET" in block and re.search(r"\bportIdentity\s+\S+-" + str(port_number) + r"\b", block):
            match = re.search(r"\bportState\s+(\S+)", block)
            if not match or match[1] != "MASTER":
                raise CheckError(f"{interface}: PMC port state is {match[1] if match else 'unknown'}")
            return
    raise CheckError(f"{interface}: no matching port in PMC response")


def master_from_journal(process, port_number):
    unit, invocation, entries = journal(process, count=2000)
    pattern = re.compile(r"port " + str(port_number) + r": \S+ to (\S+) on ")
    transitions = [match[1] for entry in entries
                   if (match := pattern.search(entry.get("MESSAGE", "")))]
    if not transitions or transitions[-1] != "MASTER":
        raise CheckError(f"{unit}: latest port {port_number} journal state is {transitions[-1] if transitions else 'unknown'}")
    return invocation


def read_phc(index, expected_seconds, tolerance_ns):
    try:
        # A read-only descriptor plus clock_gettime never adjusts the PHC.
        descriptor = os.open(f"/dev/ptp{index}", os.O_RDONLY)
        try:
            clock = ((~descriptor) << 3) | 3  # FD_TO_CLOCKID.
            before = time.time_ns()
            phc = time.clock_gettime_ns(clock)
            after = time.time_ns()
        finally:
            os.close(descriptor)
    except (OSError, ValueError) as error:
        return None, str(error)
    # Bound the offset rather than treating read latency as a clock error.
    lower, upper = phc - after, phc - before
    expected = expected_seconds * 1_000_000_000
    if upper < expected - tolerance_ns or lower > expected + tolerance_ns:
        raise CheckError(f"PHC ptp{index} - UTC = [{lower}, {upper}] ns; expected {expected} +/- {tolerance_ns} ns")
    return {"offset_ns": phc - (before + after) // 2,
            "read_uncertainty_ns": (after - before) // 2}, None


def servo_for(interface, index, candidates):
    if any("-a" in process["arguments"] for process in candidates):
        raise CheckError(f"{interface}: automatic phc2sys is running; cannot establish a unique fixed-direction PHC servo")
    matches = [process for process in candidates
               if option(process["arguments"], ["-c"]) in (interface, f"/dev/ptp{index}")]
    if len(matches) != 1:
        raise CheckError(f"{interface}: expected exactly one PHC servo, found {len(matches)}")
    process = matches[0]
    arguments = process["arguments"]
    if option(arguments, ["-s"]) != "CLOCK_REALTIME":
        raise CheckError(f"{interface}: phc2sys must synchronize CLOCK_REALTIME -> PHC")
    for names, key, default in [(["-S", "--step_threshold"], "step_threshold", "0"),
                                 (["-F", "--first_step_threshold"], "first_step_threshold", "0.00002")]:
        value = option(arguments, names, process["config"]["global"].get(key, default))
        if float(value) != 0:
            raise CheckError(f"{interface}: phc2sys {key} must be 0 during live acquisition")
    return process


def servo_sample(process, interface, index, utc_offset, tolerance_ns):
    arguments = process["arguments"]
    offset = option(arguments, ["-O"])
    if offset is not None and float(offset) != utc_offset:
        raise CheckError(f"{interface}: phc2sys -O {offset} disagrees with configured UTC offset {utc_offset}")
    if offset is None and "-w" not in arguments:
        raise CheckError(f"{interface}: phc2sys needs -w or explicit -O {utc_offset}")
    unit, invocation, entries = journal(process, count=40)
    target = option(arguments, ["-c"])
    pattern = re.compile(re.escape(target) + r" sys offset\s+([+-]?\d+)\s+s(\d+)\s+freq\s+([+-]?\d+)")
    samples = [(int(entry["__MONOTONIC_TIMESTAMP"]) / 1e6, match)
               for entry in entries if (match := pattern.search(entry.get("MESSAGE", "")))]
    if not samples or not 0 <= time.monotonic() - samples[-1][0] <= 3:
        raise CheckError(f"{unit}: PHC journal sample is missing or older than 3 seconds")
    stamp, sample = samples[-1]
    if int(sample[2]) not in (2, 3) or abs(int(sample[1])) > tolerance_ns:
        raise CheckError(f"{interface}: PHC not locked, state=s{sample[2]} offset={sample[1]} ns")
    frequency_limit = int(option(arguments, ["--max_frequency"],
                                 process["config"]["global"].get("max_frequency", "900000000")))
    hardware_limit = int((Path("/sys/class/ptp") / f"ptp{index}" / "max_adjustment").read_text())
    frequency_limit = min(frequency_limit or hardware_limit, hardware_limit)
    if abs(int(sample[3])) >= frequency_limit:
        raise CheckError(f"{interface}: PHC servo frequency {sample[3]} ppb saturates limit {frequency_limit} ppb")
    return {"source": "systemd journal", "offset_ns": int(sample[1]),
            "servo_state": int(sample[2]), "sample_monotonic": stamp,
            "invocation": invocation, "frequency_ppb": int(sample[3]),
            "frequency_limit_ppb": frequency_limit}


def inspect(path, utc_offset, tolerance_ns, strict_pmc=False):
    routes, running = lidar_routes(path), processes()
    results = []
    for interface in dict.fromkeys(route["interface"] for route in routes):
        matches = [process for process in running["ptp4l"] if interface in process["interfaces"]]
        if len(matches) != 1:
            raise CheckError(f"{interface}: expected exactly one ptp4l, found {len(matches)}; existing services are never started/stopped here")
        master = matches[0]
        port_number = master["interfaces"].index(interface) + 1
        if "-s" in master["arguments"] or setting(master, "slaveOnly", "0") != "0":
            raise CheckError(f"{interface}: ptp4l is configured as a slave")
        if setting(master, "masterOnly", "0", interface) != "1" and setting(master, "serverOnly", "0", interface) != "1":
            raise CheckError(f"{interface}: require masterOnly=1 (or serverOnly=1) to protect host clock authority")
        transport = setting(master, "network_transport", "UDPv4", interface)
        delay = setting(master, "delay_mechanism", "E2E", interface)
        for argument in master["arguments"]:
            transport = {"-2": "L2", "-4": "UDPv4", "-6": "UDPv6"}.get(argument, transport)
            delay = {"-E": "E2E", "-P": "P2P", "-A": "Auto"}.get(argument, delay)
        if transport != "UDPv4":
            raise CheckError(f"{interface}: MID360 checks require UDPv4 PTP")
        if delay != "E2E":
            raise CheckError(f"{interface}: MID360 checks require E2E delay mechanism")
        domain = int(setting(master, "domainNumber", "0"))
        if domain != 0:
            raise CheckError(f"{interface}: MID360 expects PTP domain 0, found {domain}")
        method = setting(master, "time_stamping", "hardware")
        if "-S" in master["arguments"]:
            method = "software"
        elif "-H" in master["arguments"]:
            method = "hardware"
        if method == "software" and utc_offset != 0:
            raise CheckError(f"{interface}: software GM uses UTC; set --utc-offset 0 and driver offset 0. "
                             "Mixed hardware/software profiles require separate driver instances.")
        capability = timestamp_info(interface)
        properties, unavailable = management(master, domain)
        row = {"interface": interface, "timestamping": method,
               "phc_index": capability["phc_index"], "ptp4l_pid": master["pid"],
               "domain": domain, "port_state": "MASTER"}
        if properties:
            check_port(properties, interface, port_number)
            fields = dict(re.findall(r"\b(currentUtcOffset|ptpTimescale|leap61|leap59)\s+(\S+)", properties))
            timescale = int(fields.get("ptpTimescale", "-1"))
            if timescale != (1 if method == "hardware" else 0):
                raise CheckError(f"{interface}: PMC ptpTimescale={timescale} disagrees with {method} timestamping")
            if timescale and int(fields.get("currentUtcOffset", "-1")) != utc_offset:
                raise CheckError(f"{interface}: PMC UTC offset differs from expected {utc_offset} s")
            if fields.get("leap61") == "1" or fields.get("leap59") == "1":
                raise CheckError(f"{interface}: PTP announces a leap second; explicit UTC-offset review is required")
            row["master_evidence"] = "PMC GET"
        elif strict_pmc:
            raise CheckError(f"{interface}: PMC GET unavailable: {unavailable}; grant socket read/write access or run this read-only check as root")
        else:
            row["ptp_invocation"] = master_from_journal(master, port_number)
            row["master_evidence"] = "same-invocation systemd journal + active process config"
            row["pmc_unavailable"] = unavailable
        if method == "hardware":
            if capability["phc_index"] < 0 or capability["so_timestamping"] & 0x45 != 0x45:
                raise CheckError(f"{interface}: hardware timestamping/PHC unavailable")
            if int(setting(master, "utc_offset", "37")) != utc_offset:
                raise CheckError(f"{interface}: ptp4l utc_offset disagrees with expected {utc_offset} s")
            servo = servo_for(interface, capability["phc_index"], running["phc2sys"])
            if "-w" in servo["arguments"]:
                ptp_socket = setting(master, "uds_address", "/var/run/ptp4l")
                servo_socket = option(servo["arguments"], ["-z", "--uds_address"],
                                      servo["config"]["global"].get("uds_address", "/var/run/ptp4l"))
                servo_domain = int(option(servo["arguments"], ["-n", "--domainNumber"],
                                          servo["config"]["global"].get("domainNumber", "0")))
                if Path(ptp_socket).resolve() != Path(servo_socket).resolve() or servo_domain != domain:
                    raise CheckError(f"{interface}: phc2sys -w must use this MASTER's UDS and PTP domain")
            row["phc2sys_pid"] = servo["pid"]
            direct, unreadable = read_phc(capability["phc_index"], utc_offset, tolerance_ns)
            # Journal proves servo lock as well as the configured UTC/TAI direction.
            row["phc_servo"] = servo_sample(servo, interface, capability["phc_index"], utc_offset, tolerance_ns)
            if direct:
                row["phc_read"] = direct
            elif strict_pmc:
                raise CheckError(f"{interface}: direct PHC read unavailable: {unreadable}")
            else:
                row["phc_read_unavailable"] = unreadable
            row["phc_timescale"] = "PTP/TAI"
            row["utc_offset_seconds"] = utc_offset
        elif method == "software":
            if setting(master, "free_running", "0") != "1":
                raise CheckError(f"{interface}: software GM must set free_running=1")
            row["phc_timescale"] = "UTC/CLOCK_REALTIME (no PHC servo)"
            row["utc_offset_seconds"] = 0
        else:
            raise CheckError(f"{interface}: unsupported timestamping method {method}")
        results.append(row)
    return {"routes": routes, "interfaces": results,
            "sensor_lock_verified": False,
            "note": "Host PTP checks only; the driver must verify each lidar/IMU PTP packet independently."}


def wait_ready(args):
    deadline = time.monotonic() + args.timeout
    stable_since = None
    identity = None
    previous = (time.monotonic(), time.time_ns())
    last_error = None
    while True:
        now, realtime = time.monotonic(), time.time_ns()
        if abs((realtime - previous[1]) - (now - previous[0]) * 1e9) > 10_000_000:
            raise CheckError("CLOCK_REALTIME stepped by more than 10 ms during PTP readiness check")
        previous = now, realtime
        try:
            result = inspect(args.config, args.utc_offset,
                             round(args.max_phc_offset_ms * 1e6), args.strict_pmc)
            current = [(row["interface"], row["ptp4l_pid"], row.get("phc2sys_pid"),
                        row.get("ptp_invocation"), row.get("phc_servo", {}).get("invocation"))
                       for row in result["interfaces"]]
            if current != identity:
                stable_since, identity = time.monotonic(), current
            if time.monotonic() - stable_since >= args.settle_seconds:
                return result
        except CheckError as error:
            stable_since, identity = None, None
            if str(error) != last_error and not args.json:
                print(f"[ptp-check] Waiting: {error}", flush=True)
            last_error = str(error)
        if time.monotonic() >= deadline:
            raise CheckError(f"PTP not ready within {args.timeout:g} s: {last_error or 'stability window incomplete'}")
        time.sleep(min(1, max(0, deadline - time.monotonic())))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True, help="Livox JSON used by the live driver")
    parser.add_argument("--utc-offset", type=int, default=37, help="Expected PHC PTP/TAI minus UTC seconds (default: 37)")
    parser.add_argument("--max-phc-offset-ms", type=float, default=1.0)
    parser.add_argument("--timeout", type=float, default=30)
    parser.add_argument("--settle-seconds", type=float, default=3)
    parser.add_argument("--strict-pmc", action="store_true", help="Require PMC GET and direct PHC reads; no journal/config fallback")
    parser.add_argument("--json", action="store_true", help="Output one JSON result")
    args = parser.parse_args(argv)
    for name in ("timeout", "settle_seconds", "max_phc_offset_ms"):
        value = getattr(args, name)
        if not math.isfinite(value) or value < 0 or (name == "max_phc_offset_ms" and value == 0):
            parser.error(f"{name.replace('_', '-')} must be finite and {'positive' if name == 'max_phc_offset_ms' else 'nonnegative'}")
    if args.timeout < args.settle_seconds or args.utc_offset < 0:
        parser.error("timeout must cover settle-seconds and utc-offset must be nonnegative")
    try:
        result = wait_ready(args)
    except (CheckError, OSError, ValueError, KeyError, subprocess.TimeoutExpired) as error:
        if args.json:
            print(json.dumps({"ok": False, "error": str(error)}, ensure_ascii=False))
        else:
            print(f"[ptp-check] REFUSED: {error}", flush=True)
        return 1
    result["ok"] = True
    if args.json:
        print(json.dumps(result, ensure_ascii=False))
    else:
        for route in result["routes"]:
            print(f"[ptp-check] lidar {route['lidar']} -> {route['interface']} ({route['host_ip']})")
        for row in result["interfaces"]:
            print(f"[ptp-check] {row['interface']} MASTER, {row['timestamping']}, {row['master_evidence']}; "
                  f"PHC timescale={row['phc_timescale']}, UTC offset={row['utc_offset_seconds']} s")
            if "phc_servo" in row:
                print(f"[ptp-check] PHC servo locked, offset={row['phc_servo']['offset_ns']} ns")
            if row.get("pmc_unavailable") or row.get("phc_read_unavailable"):
                print("[ptp-check] Direct PMC/PHC unavailable; using current service config/journal evidence. "
                      "Use --strict-pmc with socket/PHC permissions to require direct reads.")
        print("[ptp-check] Host ready. Per-device lidar and IMU PTP lock is checked by the driver.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
