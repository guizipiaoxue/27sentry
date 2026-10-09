#include "lidar_health_monitor.h"

#include <arpa/inet.h>
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>

#include "livox_lidar_api.h"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

namespace livox_ros {
namespace {
uint64_t SteadyNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
uint64_t UtcNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
}
std::string Hex(uint32_t value, unsigned width = 8) {
  std::ostringstream out;
  out << "0x" << std::hex << std::setw(width) << std::setfill('0') << value;
  return out.str();
}
const char* HmsLevel(unsigned level) {
  switch (level) {
    case 1: return "info";
    case 2: return "warning";
    case 3: return "error";
    case 4: return "fatal";
    default: return "unknown";
  }
}
const char* HmsReason(uint16_t id) {
  switch (id) {
    case 0x0102: return "environment_temperature_slightly_high";
    case 0x0103: return "environment_temperature_high";
    case 0x0104: return "dirty_window";
    case 0x0105: return "upgrade_failed";
    case 0x0111: case 0x0112: return "internal_component_temperature_abnormal";
    case 0x0113: return "imu_stopped";
    case 0x0114: return "environment_overtemperature";
    case 0x0115: return "thermal_shutdown";
    case 0x0116: return "external_voltage_abnormal";
    case 0x0117: return "lidar_parameters_abnormal";
    case 0x0118: return "internal_component_damaged";
    case 0x0201: return "scan_module_heating";
    case 0x021c: return "scan_code_disk_dirty_or_rotation_abnormal";
    case 0x0304: return "ranging_tia_dc_abnormal";
    case 0x0401: return "communication_link_recovered";
    case 0x0402: return "ptp_stopped_or_time_gap";
    case 0x0403: return "unsupported_ptp_version";
    case 0x0404: return "pps_sync_abnormal";
    case 0x0405: return "time_sync_exception";
    case 0x0406: return "time_sync_accuracy_low";
    case 0x0407: return "gps_signal_lost";
    case 0x0408: return "pps_signal_lost";
    case 0x0409: return "gps_signal_abnormal";
    case 0x040a: return "ptp_gptp_conflict";
    default:
      if (id >= 0x0210 && id <= 0x0219) return "scan_module_abnormal";
      return "unknown_hms_id";
  }
}
const char* WorkState(uint8_t work) {
  switch (work) {
    case 1: return "sampling";
    case 2: return "idle";
    case 4: return "error";
    case 5: return "selfcheck";
    case 6: return "motor_startup";
    case 8: return "upgrade";
    case 9: return "ready";
    default: return "unknown";
  }
}
}  // namespace

const char* LivoxStatusName(livox_status status) {
  switch (status) {
    case 0: return "success";
    case -1: return "failure";
    case -2: return "not_connected";
    case -3: return "not_supported";
    case -4: return "timeout";
    case -5: return "not_enough_memory";
    case -6: return "channel_not_exist";
    case -7: return "invalid_handle";
    case -8: return "handler_not_exist";
    case -9: return "send_failed";
    default: return "unknown_sdk_status";
  }
}

bool LidarCommandSucceeded(livox_status status,
                           const LivoxLidarAsyncControlResponse* response) {
  return status == kLivoxLidarStatusSuccess && response && response->ret_code == 0;
}

std::string DescribeHms(uint32_t code) {
  return "code=" + Hex(code) + " id=" + Hex(code >> 16, 4) +
      " severity=" + HmsLevel(code & 255) + " reason=" + HmsReason(code >> 16);
}

bool DecodeLidarHealth(const LivoxLidarDiagInternalInfoResponse& response,
                       LidarHealthFields& fields) {
  fields = {};
  if (response.ret_code || !response.param_num || response.param_num > 128) return false;
  LidarHealthFields parsed;
  std::set<uint16_t> keys;
  const uint8_t* cursor = response.data;
  for (uint16_t i = 0; i < response.param_num; ++i) {
    uint16_t key = 0, length = 0;
    std::memcpy(&key, cursor, sizeof(key));
    std::memcpy(&length, cursor + sizeof(key), sizeof(length));
    cursor += 4;
    if (length > 256 || !keys.insert(key).second) return false;
    if (key == kKeyCoreTemp) {
      if (length != sizeof(int32_t)) return false;
      std::memcpy(&parsed.temperature, cursor, length);
      parsed.has_temperature = true;
    } else if (key == kKeyCurWorkState || key == kKeyTimeSyncType) {
      if (length != 1) return false;
      if (key == kKeyCurWorkState) { parsed.work = *cursor; parsed.has_work = true; }
      else { parsed.sync = *cursor; parsed.has_sync = true; }
    } else if (key == kKeyLidarDiagStatus) {
      if (length != sizeof(uint16_t)) return false;
      std::memcpy(&parsed.diag, cursor, length);
      parsed.has_diag = true;
    } else if (key == kKeyHmsCode) {
      if (length != 8 * sizeof(uint32_t)) return false;
      std::memcpy(parsed.hms.data(), cursor, length);
      parsed.has_hms = true;
    }
    cursor += length;
  }
  if (!parsed.has_temperature) return false;
  fields = parsed;
  return true;
}

bool ParseLidarHealthPush(const std::string& json, LidarHealthFields& fields) {
  fields = {};
  rapidjson::Document doc;
  doc.Parse(json.c_str());
  if (doc.HasParseError() || !doc.IsObject()) return false;
  LidarHealthFields parsed;
  std::set<std::string> keys;
  for (auto it = doc.MemberBegin(); it != doc.MemberEnd(); ++it) {
    if (!keys.insert(it->name.GetString()).second) return false;
    const auto& value = it->value;
    const std::string key = it->name.GetString();
    if (key == "core_temp") {
      if (!value.IsInt()) return false;
      parsed.temperature = value.GetInt(); parsed.has_temperature = true;
    } else if (key == "lidar_diag_status") {
      if (!value.IsUint() || value.GetUint() > 65535) return false;
      parsed.diag = value.GetUint(); parsed.has_diag = true;
    } else if (key == "cur_work_state" || key == "time_sync_type") {
      if (!value.IsUint() || value.GetUint() > 255) return false;
      if (key == "cur_work_state") { parsed.work = value.GetUint(); parsed.has_work = true; }
      else { parsed.sync = value.GetUint(); parsed.has_sync = true; }
    } else if (key == "hms_code") {
      if (!value.IsArray() || value.Size() != 8) return false;
      for (unsigned i = 0; i < 8; ++i) {
        if (!value[i].IsUint()) return false;
        parsed.hms[i] = value[i].GetUint();
      }
      parsed.has_hms = true;
    }
  }
  fields = parsed;
  return true;  // A push may contain only unrelated configuration fields.
}

std::string FormatHealthFields(const LidarHealthFields& fields) {
  std::ostringstream out;
  if (fields.has_temperature) out << " core_temp_raw=" << fields.temperature
      << " core_temp_c=" << std::fixed << std::setprecision(2) << fields.temperature / 100.0;
  if (fields.has_work) out << " work_state=" << unsigned(fields.work)
      << " work_state_name=" << WorkState(fields.work);
  if (fields.has_sync) out << " time_sync_type=" << unsigned(fields.sync)
      << " time_sync_name=" << (fields.sync == 0 ? "none" : fields.sync == 1 ? "ptp" : fields.sync == 2 ? "gps" : "unknown");
  if (fields.has_diag) out << " lidar_diag_status=" << fields.diag
      << " diag_system=" << (fields.diag & 15) << " diag_scan=" << ((fields.diag >> 4) & 15)
      << " diag_ranging=" << ((fields.diag >> 8) & 15)
      << " diag_communication=" << ((fields.diag >> 12) & 15);
  if (fields.has_hms) {
    out << " hms=[";
    for (unsigned i = 0; i < 8; ++i) out << (i ? "," : "") << Hex(fields.hms[i]);
    out << ']';
  }
  return out.str();
}

bool FormatLidarHealth(const LivoxLidarDiagInternalInfoResponse& response,
                       std::string& payload) {
  payload.clear();
  LidarHealthFields fields;
  if (!DecodeLidarHealth(response, fields)) return false;
  payload = FormatHealthFields(fields);
  return true;
}

void LidarHealthState::Apply(const LidarHealthFields& patch, uint64_t now) {
  if (patch.has_temperature && now >= updated[0]) { fields.temperature = patch.temperature; fields.has_temperature = true; updated[0] = now; }
  if (patch.has_diag && now >= updated[1]) { fields.diag = patch.diag; fields.has_diag = true; updated[1] = now; }
  if (patch.has_hms && now >= updated[2]) { fields.hms = patch.hms; fields.has_hms = true; updated[2] = now; }
  if (patch.has_work && now >= updated[3]) { fields.work = patch.work; fields.has_work = true; updated[3] = now; }
  if (patch.has_sync && now >= updated[4]) { fields.sync = patch.sync; fields.has_sync = true; updated[4] = now; }
}

std::string LidarHealthState::Summary(uint64_t now) const {
  bool stale = false;
  for (auto stamp : updated) if (!stamp || now < stamp || now - stamp > 15000000000ULL) stale = true;
  unsigned level = 0;
  bool unknown = false;
  if (fields.has_work && fields.work == 4) level = 3;
  if (fields.has_work && std::string(WorkState(fields.work)) == "unknown") unknown = true;
  if (fields.has_diag) for (unsigned i = 0; i < 4; ++i) {
    const unsigned nibble = (fields.diag >> (i * 4)) & 15;
    if (nibble > 3) unknown = true;
    level = std::max(level, nibble == 3 ? 4U : nibble == 2 ? 3U : nibble == 1 ? 2U : 0U);
  }
  if (fields.has_hms) for (auto code : fields.hms) if (code) {
    if ((code & 255) < 1 || (code & 255) > 4) unknown = true;
    else level = std::max(level, code & 255);
  }
  const char* status = level >= 2 ? HmsLevel(level) :
      stale || unknown ? "unknown" : "normal";
  std::string out = "device_status=" + std::string(status) +
      " health_fresh=" + (stale ? "0" : "1") + FormatHealthFields(fields);
  const char* names[] = {"temperature", "diag", "hms", "work", "sync"};
  for (unsigned i = 0; i < 5; ++i) out += " " + std::string(names[i]) + "_age_ms=" +
      (updated[i] && now >= updated[i] ? std::to_string((now - updated[i]) / 1000000) : "unknown");
  return out;
}

void LidarHealthMonitor::Start() {
  if (worker_.joinable()) return;
  try {
    logger_ = std::make_unique<odom_logging::RunLogger>("lidar_health");
    std::cout << "LIDAR_HEALTH_LOG file=" << logger_->path().string() << std::endl;
  } catch (const std::exception& error) {
    std::cerr << "RUNLOG_WRITE_FAILED component=lidar_health reason=" << error.what() << std::endl;
  }
  { std::lock_guard<std::mutex> lock(mutex_); stopping_ = false; }
  worker_ = std::thread(&LidarHealthMonitor::Run, this);
  SetLivoxLidarInfoCallback(PushCallback, this);
}

void LidarHealthMonitor::Stop() {
  { std::lock_guard<std::mutex> lock(mutex_); stopping_ = true; retries_.clear(); }
  condition_.notify_all();
  if (worker_.joinable()) worker_.join();
  logger_.reset();  // Flush queued lines before reporting shutdown complete.
  // SDK is uninitialized by the owning LdsLidar before this object is destroyed.
}

void LidarHealthMonitor::SetExpectedDevices(const std::vector<uint32_t>& handles) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto handle : handles) devices_.emplace(handle, false);
}
void LidarHealthMonitor::SetStreamSummary(std::function<std::string(uint32_t)> summary) {
  std::lock_guard<std::mutex> lock(mutex_);
  stream_summary_ = std::move(summary);
}
void LidarHealthMonitor::RetryAfter(std::function<void()> command) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!stopping_) retries_.emplace_back(std::chrono::steady_clock::now() +
      std::chrono::seconds(1), std::move(command));
}
void LidarHealthMonitor::ObserveDevice(uint32_t handle, uint8_t type) {
  if (type != kLivoxLidarTypeMid360) return;
  bool found = false;
  { std::lock_guard<std::mutex> lock(mutex_); found = !devices_[handle]; devices_[handle] = true; }
  if (found) RecordEvent(handle, "LIDAR_DEVICE_FOUND", "device_type=9");
}
void LidarHealthMonitor::ObservePush(uint32_t handle, uint8_t type, const char* info) {
  if (type != kLivoxLidarTypeMid360) return;
  ObserveDevice(handle, type);
  Report report;
  report.handle = handle; report.event = "LIDAR_PUSH";
  if (!info) { report.event = "LIDAR_PUSH_INVALID"; report.payload = "reason=null_info"; }
  else {
    // SDK supplies a NUL-terminated JSON string. Bound copying in its callback.
    const auto size = strnlen(info, 16385);
    if (size > 16384) { report.event = "LIDAR_PUSH_INVALID"; report.payload = "reason=oversize_json"; }
    else report.payload.assign(info, size);
  }
  Enqueue(std::move(report));
}
void LidarHealthMonitor::PushCallback(uint32_t handle, uint8_t type, const char* info, void* data) {
  if (data) static_cast<LidarHealthMonitor*>(data)->ObservePush(handle, type, info);
}
void LidarHealthMonitor::RecordEvent(uint32_t handle, const std::string& event,
                                     const std::string& payload) {
  Report report; report.handle = handle; report.event = event; report.payload = payload;
  Enqueue(std::move(report));
}
void LidarHealthMonitor::RecordSubmit(uint32_t handle, const char* command, livox_status status) {
  if (status != kLivoxLidarStatusSuccess) RecordEvent(handle, "LIDAR_COMMAND_SUBMIT_FAILED",
      "command=" + std::string(command) + " sdk_status=" + std::to_string(status) +
      " sdk_reason=" + LivoxStatusName(status));
}
void LidarHealthMonitor::RecordDriverEvent(const std::string& event, const std::string& payload) {
  const auto start = payload.find("ip=");
  in_addr address{};
  auto details = payload;
  if (start != std::string::npos) {
    const auto end = payload.find(' ', start);
    const auto ip = payload.substr(start + 3, end == std::string::npos ? end : end - start - 3);
    if (inet_pton(AF_INET, ip.c_str(), &address) == 1)
      details.erase(start, end == std::string::npos ? end : end - start + 1);
  }
  RecordEvent(address.s_addr, event, "source=driver " + details);
}
void LidarHealthMonitor::RecordCommand(uint32_t handle, const char* command,
    livox_status status, const LivoxLidarAsyncControlResponse* response) {
  RecordEvent(handle, LidarCommandSucceeded(status, response) ?
      "LIDAR_COMMAND_OK" : "LIDAR_COMMAND_FAILED",
      "command=" + std::string(command) + " sdk_status=" + std::to_string(status) +
      " sdk_reason=" + LivoxStatusName(status) + " ret_code=" +
      (response ? std::to_string(response->ret_code) : "null") + " error_key=" +
      (response ? Hex(response->error_key, 4) : "null"));
}
void LidarHealthMonitor::Enqueue(Report report) {
  report.steady_ns = SteadyNs(); report.utc_ns = UtcNs();
  { std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_) return;
    if (reports_.size() >= 1024) { reports_.pop_front(); ++dropped_; }
    reports_.push_back(std::move(report));
  }
  condition_.notify_one();
}
void LidarHealthMonitor::QueryCallback(livox_status status, uint32_t handle,
    LivoxLidarDiagInternalInfoResponse* response, void* data) {
  if (!data) return;
  auto* monitor = static_cast<LidarHealthMonitor*>(data);
  Report report; report.handle = handle;
  report.health = status == kLivoxLidarStatusSuccess && response &&
      DecodeLidarHealth(*response, report.fields);
  report.event = report.health ? "LIDAR_HEALTH" : "LIDAR_HEALTH_QUERY_FAILED";
  report.payload = report.health ? "source=query" + FormatHealthFields(report.fields) :
      "sdk_status=" + std::to_string(status) + " sdk_reason=" + LivoxStatusName(status) +
      " ret_code=" + (response ? std::to_string(response->ret_code) : "null") +
      " reason=query_failed_or_invalid_or_missing_temperature";
  monitor->Enqueue(std::move(report));
}
void LidarHealthMonitor::Write(uint32_t handle, const std::string& event,
                              const std::string& payload, uint64_t utc_ns) {
  in_addr address{}; address.s_addr = handle;
  char ip[INET_ADDRSTRLEN] = {}; inet_ntop(AF_INET, &address, ip, sizeof(ip));
  const auto line = "ip=" + std::string(ip) + " host_utc_ns=" +
      std::to_string(utc_ns ? utc_ns : UtcNs()) + " " + payload;
  if (logger_) logger_->log(event, line);
  std::cout << event << ' ' << line << std::endl;
}
void LidarHealthMonitor::Run() {
  using Clock = std::chrono::steady_clock;
  auto next_query = Clock::now() + std::chrono::seconds(5);
  auto next_summary = Clock::now() + std::chrono::seconds(1);
  std::map<uint32_t, LidarHealthState> states;
  std::map<uint32_t, uint64_t> query_failures, command_failures;
  std::map<uint32_t, uint64_t> push_failures;
  std::map<uint32_t, bool> query_failed;
  std::map<uint32_t, bool> push_invalid;
  std::map<uint32_t, std::set<std::string>> failed_commands;
  std::map<uint32_t, std::string> last_status;
  for (;;) {
    std::deque<Report> reports;
    std::map<uint32_t, bool> devices;
    std::vector<std::function<void()>> retries;
    std::function<std::string(uint32_t)> streams;
    std::size_t dropped;
    bool stopping;
    { std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait_until(lock, std::min(next_query, next_summary),
          [this] { return stopping_ || !reports_.empty(); });
      reports.swap(reports_); dropped = dropped_; dropped_ = 0;
      stopping = stopping_; devices = devices_; streams = stream_summary_;
      for (auto it = retries_.begin(); it != retries_.end();) {
        if (it->first <= Clock::now()) { retries.push_back(std::move(it->second)); it = retries_.erase(it); }
        else ++it;
      }
    }
    if (dropped) {
      Write(0, "LIDAR_HEALTH_DROPPED", "count=" + std::to_string(dropped));
      for (auto& state : states) state.second.updated.fill(0);
    }
    for (auto& report : reports) {
      if (report.event == "LIDAR_PUSH") {
        report.health = ParseLidarHealthPush(report.payload, report.fields);
        rapidjson::Document doc; doc.Parse(report.payload.c_str());
        if (!doc.HasParseError()) {
          rapidjson::StringBuffer buffer; rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
          doc.Accept(writer); report.payload = "json=" + std::string(buffer.GetString());
        } else {
          rapidjson::StringBuffer buffer; rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
          writer.String(report.payload.c_str(), report.payload.size());
          report.payload = "json_invalid=" + std::string(buffer.GetString());
        }
        if (!report.health) report.event = "LIDAR_PUSH_INVALID";
      }
      Write(report.handle, report.event, report.payload, report.utc_ns);
      if (report.event == "LIDAR_PUSH_INVALID") {
        ++push_failures[report.handle];
        push_invalid[report.handle] = true;
      }
      if (report.health && report.fields.has_diag && report.fields.has_hms)
        push_invalid[report.handle] = false;
      if (report.event == "LIDAR_HEALTH_QUERY_FAILED") { ++query_failures[report.handle]; query_failed[report.handle] = true; }
      if (report.event == "LIDAR_HEALTH") query_failed[report.handle] = false;
      if (report.event == "LIDAR_COMMAND_FAILED" || report.event == "LIDAR_COMMAND_SUBMIT_FAILED" || report.event == "LIDAR_COMMAND_OK") {
        const auto end = report.payload.find(' ');
        const auto command = report.payload.substr(0, end);
        if (report.event == "LIDAR_COMMAND_OK") failed_commands[report.handle].erase(command);
        else { ++command_failures[report.handle]; failed_commands[report.handle].insert(command); }
      }
      if (report.health) {
        auto& state = states[report.handle];
        if (report.fields.has_hms && report.steady_ns >= state.updated[2]) {
          std::set<uint32_t> old_codes(state.fields.hms.begin(), state.fields.hms.end());
          std::set<uint32_t> codes(report.fields.hms.begin(), report.fields.hms.end());
          for (auto code : codes) if (code && !old_codes.count(code)) Write(report.handle, "LIDAR_HMS_ACTIVE", DescribeHms(code), report.utc_ns);
          for (auto code : old_codes) if (code && !codes.count(code)) Write(report.handle, "LIDAR_HMS_CLEARED", DescribeHms(code), report.utc_ns);
        }
        if (report.fields.has_diag && report.steady_ns >= state.updated[1] &&
            (!state.fields.has_diag || state.fields.diag != report.fields.diag))
          Write(report.handle, "LIDAR_DIAG_CHANGED", FormatHealthFields(report.fields), report.utc_ns);
        state.Apply(report.fields, report.steady_ns);
      }
    }
    if (stopping) return;
    for (auto& retry : retries) retry();
    if (Clock::now() >= next_summary) {
      for (const auto& device : devices) {
        std::string commands;
        for (const auto& command : failed_commands[device.first]) commands += (commands.empty() ? "" : ",") + command.substr(8);
        const auto health = states[device.first].Summary(SteadyNs());
        const auto stream_status = streams ? streams(device.first) : "";
        const char* status = !device.second ? "not_discovered" :
            health.find("device_status=fatal") != std::string::npos ? "fatal" :
            health.find("device_status=error") != std::string::npos ? "error" :
            stream_status.find("timeout") != std::string::npos || stream_status.find("rejected") != std::string::npos ||
            !failed_commands[device.first].empty() ? "error" :
            query_failed[device.first] || push_invalid[device.first] || health.find("device_status=unknown") != std::string::npos ||
            stream_status.find("not_received") != std::string::npos ? "unknown" :
            health.find("device_status=warning") != std::string::npos ? "warning" :
            states[device.first].fields.work != 1 ? "not_sampling" : "normal";
        if (last_status[device.first] != status) {
          Write(device.first, "LIDAR_STATUS_CHANGED", "from=" +
              (last_status[device.first].empty() ? std::string("initial") : last_status[device.first]) +
              " to=" + status);
          last_status[device.first] = status;
        }
        Write(device.first, "LIDAR_STATUS", "status=" + std::string(status) +
            " discovered=" + std::to_string(device.second) + " " + health + " query_status=" +
            (query_failed[device.first] ? "failed" : states[device.first].updated[0] ? "available" : "pending") +
            " query_failures_total=" + std::to_string(query_failures[device.first]) +
            " push_invalid=" + std::to_string(push_invalid[device.first]) +
            " push_failures_total=" + std::to_string(push_failures[device.first]) +
            " command_failures_total=" + std::to_string(command_failures[device.first]) +
            " failed_commands=[" + commands + "]" + stream_status);
      }
      next_summary = Clock::now() + std::chrono::seconds(1);
    }
    if (Clock::now() >= next_query) {
      for (const auto& device : devices) if (device.second) {
        auto status = QueryLivoxLidarInternalInfo(device.first, QueryCallback, this);
        if (status != kLivoxLidarStatusSuccess) RecordEvent(device.first, "LIDAR_HEALTH_QUERY_FAILED",
            "submit_status=" + std::to_string(status) + " sdk_reason=" + LivoxStatusName(status));
      }
      next_query = Clock::now() + std::chrono::seconds(5);
    }
  }
}
}  // namespace livox_ros
