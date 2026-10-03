#!/usr/bin/env bash
# Replace only this project's legacy PTP services. Never set CLOCK_REALTIME.
set -Eeuo pipefail
export LC_ALL=C
umask 077

usage() {
  cat <<'EOF'
Usage: sudo scripts/install_ptp_services.sh [options]
  --interface NAME    Hardware PTP NIC (default: enp86s0)
  --utc-offset SEC    PHC TAI minus system UTC (default: 37; range: 0..255)
  --backup-dir PATH   New absolute backup directory
                      (default: /var/backups/sentry-ptp/<UTC timestamp>)

Installs project PTP units and initializes only the NIC's PHC if necessary.
Stops/removes the four named legacy PTP/clock services after backing them up.
Identical active project services are reused. Changed active installations
must first be stopped during a sensor maintenance window.
EOF
}

fail() { printf '[ptp-install] %s\n' "$*" >&2; exit 1; }
say() { printf '[ptp-install] %s\n' "$*"; }

interface=enp86s0
utc_offset=37
backup_dir=
while (($#)); do
  case "$1" in
    --interface|--utc-offset|--backup-dir)
      (($# >= 2)) || fail "Missing value for $1"
      case "$1" in
        --interface) interface=$2 ;;
        --utc-offset) utc_offset=$2 ;;
        --backup-dir) backup_dir=$2 ;;
      esac
      shift 2
      ;;
    -h|--help) usage; exit 0 ;;
    *) fail "Unknown argument: $1" ;;
  esac
done
[[ $EUID == 0 ]] || fail 'Run as root (sudo); no services have been changed.'
[[ $interface =~ ^[A-Za-z0-9][A-Za-z0-9_.-]{0,14}$ ]] || fail 'Invalid interface name.'
[[ $utc_offset =~ ^[0-9]{1,3}$ ]] || fail '--utc-offset must be an integer in 0..255.'
utc_offset=$((10#$utc_offset))
((utc_offset <= 255)) || fail '--utc-offset must be an integer in 0..255.'
[[ -d /sys/class/net/$interface ]] || fail "Interface $interface does not exist."
[[ -d /run/systemd/system ]] || fail 'A running systemd system manager is required.'
for command in python3 systemctl systemd-analyze install cmp cp flock; do
  command -v "$command" >/dev/null || fail "Required command not found: $command"
done
for binary in /usr/sbin/ptp4l /usr/sbin/phc2sys /usr/sbin/pmc /usr/bin/python3; do
  [[ -x $binary ]] || fail "Required executable not found: $binary"
done
/usr/sbin/ptp4l -v >/dev/null
/usr/sbin/phc2sys -v >/dev/null
systemctl show --property=Version --value >/dev/null

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
repo_dir=$(cd -- "$script_dir/.." && pwd -P)
template=$repo_dir/config/ptp/master-hardware.conf
unit_dir=$repo_dir/config/ptp/systemd
units=(sentry-phc-bootstrap@.service sentry-ptp4l@.service sentry-phc2sys@.service)
legacy_units=(ptp4l-master-enp86s0.service phc2sys-master-enp86s0.service
              odom-phc-bootstrap.service odom-clock-bootstrap.service)
legacy_links=(/etc/systemd/system/multi-user.target.wants/ptp4l-master-enp86s0.service
              /etc/systemd/system/multi-user.target.wants/phc2sys-master-enp86s0.service)
instances=("sentry-phc-bootstrap@$interface.service" "sentry-ptp4l@$interface.service"
           "sentry-phc2sys@$interface.service")
for source in "$template" "$script_dir/ptp_phc_bootstrap.py" "$script_dir/ptp_check.py"; do
  [[ -f $source ]] || fail "Missing deployment source: $source"
done
for unit in "${units[@]}"; do
  [[ -f $unit_dir/$unit ]] || fail "Missing deployment unit: $unit_dir/$unit"
done

# One installer per host. This lock does not stop any sensor/clock process.
exec 9>/run/lock/sentry-ptp-install.lock
flock -n 9 || fail 'Another PTP installation is in progress.'
stage_dir=$(mktemp -d /run/sentry-ptp-install.XXXXXXXX)
mutation_started=false
trap 'rm -rf -- "$stage_dir"' EXIT
trap 'status=$?; if $mutation_started; then printf "[ptp-install] Installation stopped (exit %s). Backup: %s. Inspect service status before restoring legacy clock services.\n" "$status" "$backup_dir" >&2; fi; exit "$status"' ERR

PYTHONDONTWRITEBYTECODE=1 python3 - "$script_dir" "$template" "$stage_dir/master.conf" "$interface" "$utc_offset" <<'PY'
import importlib.util
from pathlib import Path
import re
import stat
import sys

source, template, destination, interface, offset = sys.argv[1:]
for script in ("ptp_check.py", "ptp_phc_bootstrap.py"):
    path = Path(source, script)
    compile(path.read_text(), str(path), "exec")
spec = importlib.util.spec_from_file_location("ptp_check", Path(source, "ptp_check.py"))
ptp = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ptp)
text = Path(template).read_text()
if text.count("[enp86s0]") != 1 or text.count("uds_address /run/ptp4l-enp86s0") != 1:
    raise SystemExit("Unexpected hardware template interface/socket; refusing to guess.")
text = text.replace("enp86s0", interface)
text, count = re.subn(r"(?m)^utc_offset\s+\d+\s*$", "utc_offset " + offset, text)
if count != 1:
    raise SystemExit("Hardware template must have exactly one utc_offset.")
Path(destination).write_text(text)
config = ptp.read_ptp_config(destination)
expected = {"time_stamping": "hardware", "network_transport": "UDPv4",
            "delay_mechanism": "E2E", "domainNumber": "0", "utc_offset": offset,
            "uds_address": "/run/ptp4l-" + interface}
if any(config["global"].get(key) != value for key, value in expected.items()):
    raise SystemExit("Hardware template does not meet this project's PTP contract.")
if set(config) != {"global", interface} or config[interface].get("masterOnly") != "1":
    raise SystemExit("Hardware template must contain one master-only interface.")
capability = ptp.timestamp_info(interface)
phc_index = capability["phc_index"]
if phc_index < 0 or capability["so_timestamping"] & 0x45 != 0x45:
    raise SystemExit(f"{interface}: hardware TX/RX/raw timestamping and a PHC are required.")
phc_path = Path(f"/dev/ptp{phc_index}")
if not stat.S_ISCHR(phc_path.stat().st_mode):
    raise SystemExit(f"{phc_path}: expected a PHC character device.")
print(f"[ptp-install] Validated {interface}, {phc_path}, hardware E2E/domain 0, UTC offset {offset} s.")

# Related unmanaged PTP processes make safe migration impossible. Independent
# PTP instances are left alone. Match the cgroup, never merely the executable.
allowed = {"ptp4l-master-enp86s0.service", "phc2sys-master-enp86s0.service",
           "odom-phc-bootstrap.service", "odom-clock-bootstrap.service",
           *(f"sentry-{name}@{interface}.service" for name in
             ("phc-bootstrap", "ptp4l", "phc2sys"))}
def same_phc(clock):
    if clock in (interface, str(phc_path), phc_path.name):
        return True
    if clock and Path("/sys/class/net", clock).exists():
        return ptp.timestamp_info(clock)["phc_index"] == phc_index
    return False

for kind, processes in ptp.processes().items():
    for process in processes:
        arguments = process["arguments"]
        if kind == "ptp4l":
            related = any(same_phc(item) for item in process["interfaces"])
        else:
            related = (any(item == "-a" or item.startswith("-a") for item in arguments[1:])
                       or same_phc(ptp.option(arguments, ["-c"]))
                       or same_phc(ptp.option(arguments, ["-s"])))
        if not related:
            continue
        try:
            cgroup = Path(f"/proc/{process['pid']}/cgroup").read_text()
        except FileNotFoundError:
            continue
        owners = set(re.findall(r"/([^/\n]+\.service)(?:/|$)", cgroup, re.MULTILINE))
        if not owners & allowed:
            raise SystemExit(f"Conflicting unmanaged {kind} PID {process['pid']} on {interface}/{phc_path}; no services changed.")

# Also detect writers outside linuxptp's ptp4l/phc2sys pair. An already aligned
# bootstrap returns without stepping, so installation must independently reject
# another ts2phc/phc_ctl before starting the project's continuous PHC servo.
bootstrap_spec = importlib.util.spec_from_file_location("ptp_phc_bootstrap", Path(source, "ptp_phc_bootstrap.py"))
bootstrap = importlib.util.module_from_spec(bootstrap_spec)
bootstrap_spec.loader.exec_module(bootstrap)
for writer in bootstrap.active_clock_writers(phc_path, interface):
    try:
        cgroup = Path(f"/proc/{writer['pid']}/cgroup").read_text()
    except FileNotFoundError:
        continue
    owners = set(re.findall(r"/([^/\n]+\.service)(?:/|$)", cgroup, re.MULTILINE))
    if not owners & allowed:
        raise SystemExit(f"Conflicting unmanaged {writer['program']} PID {writer['pid']} on {interface}/{phc_path}; no services changed.")
PY

systemd-analyze verify "${units[@]/#/$unit_dir/}"

# Stopping a Required/BindsTo/PartOf unit can also stop its consumers. Reject
# external active dependencies before touching the legacy installation.
loaded_legacy=()
for unit in "${legacy_units[@]}"; do
  load_state=$(systemctl show "$unit" --property=LoadState --value)
  [[ $load_state != not-found ]] || continue
  loaded_legacy+=("$unit")
  dependency_text=$(systemctl show "$unit" --property=RequiredBy --property=BoundBy --property=ConsistsOf)
  while IFS= read -r line; do
    read -r -a dependents <<< "${line#*=}"
    for dependent in "${dependents[@]}"; do
      allowed=false
      for legacy in "${legacy_units[@]}"; do
        [[ $dependent != "$legacy" ]] || allowed=true
      done
      if ! $allowed; then
        dependent_state=$(systemctl show "$dependent" --property=ActiveState --value)
        if [[ $dependent_state == active || $dependent_state == activating ]]; then
          fail "Stopping $unit would stop $dependent_state dependent $dependent; no services changed."
        fi
      fi
    done
  done <<< "$dependency_text"
done

config_target=/etc/linuxptp/sentry/master-$interface.conf
helper_target=/usr/local/libexec/sentry-ptp/ptp_phc_bootstrap.py
changed=false
shared_changed=false
cmp -s -- "$stage_dir/master.conf" "$config_target" || changed=true
cmp -s -- "$script_dir/ptp_phc_bootstrap.py" "$helper_target" || shared_changed=true
for unit in "${units[@]}"; do
  cmp -s -- "$unit_dir/$unit" "/etc/systemd/system/$unit" || shared_changed=true
done
$shared_changed && changed=true
for instance in "${instances[@]}"; do
  # A per-instance unit or drop-in can replace the fixed servo direction or
  # reintroduce a system-clock bootstrap even when the template files match.
  drop_ins=$(systemctl show "$instance" --property=DropInPaths --value)
  [[ -z $drop_ins ]] || fail "$instance has overrides ($drop_ins); review/remove them before installing."
  fragment=$(systemctl show "$instance" --property=FragmentPath --value)
  expected_fragment=/etc/systemd/system/${instance%%@*}@.service
  [[ -z $fragment || $fragment == "$expected_fragment" ]] || \
    fail "$instance is defined by $fragment instead of the project template; no services changed."
  state=$(systemctl show "$instance" --property=ActiveState --value)
  if $changed && [[ $state == active || $state == activating || $state == deactivating ]]; then
    fail "Installed files differ while $instance is $state. Stop project PTP services in a sensor maintenance window, then rerun; no services changed."
  fi
done
if $shared_changed; then
  running_instances=$(systemctl list-units 'sentry-phc-bootstrap@*.service' 'sentry-ptp4l@*.service' \
    'sentry-phc2sys@*.service' --state=active,activating,deactivating --plain --no-legend --no-pager)
  while read -r instance _; do
    [[ -z $instance ]] || fail "Shared PTP files differ while $instance is running. Use a sensor maintenance window; no services changed."
  done <<< "$running_instances"
fi

if [[ -z $backup_dir ]]; then
  backup_dir=/var/backups/sentry-ptp/$(date -u +%Y%m%dT%H%M%S.%NZ)
fi
[[ $backup_dir == /* && $backup_dir != / ]] || fail '--backup-dir must be a new absolute directory.'
[[ ! -e $backup_dir && ! -L $backup_dir ]] || fail "Backup path already exists: $backup_dir"
mkdir -p -- "$(dirname -- "$backup_dir")"
mkdir -m 0700 -- "$backup_dir"
backup_paths=("${legacy_links[@]}" /etc/linuxptp/ptp4l-master-enp86s0.conf
              /usr/local/libexec/odom-clock "$config_target" "$helper_target")
for unit in "${legacy_units[@]}" "${units[@]}"; do
  backup_paths+=("/etc/systemd/system/$unit")
done
for path in "${backup_paths[@]}"; do
  if [[ -e $path || -L $path ]]; then
    cp -a --parents -- "$path" "$backup_dir/"
  fi
done
systemctl show "${legacy_units[@]}" "${instances[@]}" \
  --property=Id --property=LoadState --property=ActiveState --property=SubState \
  --property=UnitFileState >"$backup_dir/service-state.txt"
printf 'interface=%s\nutc_offset=%s\nrepository=%s\n' \
  "$interface" "$utc_offset" "$repo_dir" >"$backup_dir/installation.txt"
say "Backup complete: $backup_dir"

mutation_started=true
if ((${#loaded_legacy[@]})); then
  systemctl stop "${loaded_legacy[@]}"
fi
for unit in ptp4l-master-enp86s0.service phc2sys-master-enp86s0.service; do
  if [[ -e /etc/systemd/system/$unit || -L /etc/systemd/system/$unit ]]; then
    systemctl disable "$unit"
  fi
done
for path in "${legacy_links[@]}"; do
  rm -f -- "$path"
done
for unit in "${legacy_units[@]}"; do
  rm -f -- "/etc/systemd/system/$unit"
done
rm -f -- /etc/linuxptp/ptp4l-master-enp86s0.conf
rm -rf -- /usr/local/libexec/odom-clock

install -d -m 0755 /etc/linuxptp/sentry /usr/local/libexec/sentry-ptp
install -m 0644 -- "$stage_dir/master.conf" "$config_target"
install -m 0755 -- "$script_dir/ptp_phc_bootstrap.py" "$helper_target"
for unit in "${units[@]}"; do
  install -m 0644 -- "$unit_dir/$unit" "/etc/systemd/system/$unit"
done
systemctl daemon-reload
systemctl enable "sentry-ptp4l@$interface.service" "sentry-phc2sys@$interface.service"
# start reuses active services and their already-completed PHC bootstrap.
systemctl start "sentry-phc2sys@$interface.service"
say "Installed project services on $interface. Identical active services were reused."
systemctl --no-pager --full status "${instances[@]}"
printf '\nRead-only direct PTP verification:\n  sudo python3 %q --config %q --utc-offset %q --strict-pmc --timeout 30\n' \
  "$script_dir/ptp_check.py" \
  "$repo_dir/livox/src/livox_ros_driver2/config/MID360_config_2.json" "$utc_offset"
