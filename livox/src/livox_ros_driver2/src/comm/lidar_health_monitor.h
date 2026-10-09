#pragma once

#include <array>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "livox_lidar_def.h"
#include "run_logger.hpp"

namespace livox_ros {

struct LidarHealthFields {
  bool has_temperature = false, has_diag = false, has_hms = false;
  bool has_work = false, has_sync = false;
  int32_t temperature = 0;
  uint16_t diag = 0;
  uint8_t work = 0, sync = 0;
  std::array<uint32_t, 8> hms{};
};

// SDK query callbacks omit the buffer size. The SDK must supply a complete
// buffer; we validate KV counts, duplicate keys and known field lengths.
bool DecodeLidarHealth(const LivoxLidarDiagInternalInfoResponse& response,
                       LidarHealthFields& fields);
bool ParseLidarHealthPush(const std::string& json, LidarHealthFields& fields);
bool FormatLidarHealth(const LivoxLidarDiagInternalInfoResponse& response,
                       std::string& payload);
std::string DescribeHms(uint32_t code);
std::string FormatHealthFields(const LidarHealthFields& fields);
const char* LivoxStatusName(livox_status status);
bool LidarCommandSucceeded(livox_status status,
                           const LivoxLidarAsyncControlResponse* response);

// Partial pushes update only included fields. Freshness is per field so a
// temperature-only push cannot make an old HMS/diagnostic value look current.
struct LidarHealthState {
  LidarHealthFields fields;
  std::array<uint64_t, 5> updated{};  // temp, diag, HMS, work, sync (steady ns)
  void Apply(const LidarHealthFields& patch, uint64_t now);
  std::string Summary(uint64_t now) const;
};

class LidarHealthMonitor final {
 public:
  ~LidarHealthMonitor() { Stop(); }
  void Start();
  void Stop();  // Stop submitting commands before shutting down the SDK.
  void SetExpectedDevices(const std::vector<uint32_t>& handles);
  void ObserveDevice(uint32_t handle, uint8_t device_type);
  void ObservePush(uint32_t handle, uint8_t device_type, const char* info);
  void RecordEvent(uint32_t handle, const std::string& event,
                   const std::string& payload);
  void RecordDriverEvent(const std::string& event, const std::string& payload);
  void RecordCommand(uint32_t handle, const char* command, livox_status status,
                     const LivoxLidarAsyncControlResponse* response);
  void RecordSubmit(uint32_t handle, const char* command, livox_status status);
  void SetStreamSummary(std::function<std::string(uint32_t)> summary);
  void RetryAfter(std::function<void()> command);  // Used for work-mode retry.

 private:
  struct Report {
    uint32_t handle = 0;
    uint64_t steady_ns = 0, utc_ns = 0;
    std::string event, payload;
    LidarHealthFields fields;
    bool health = false;
  };
  static void PushCallback(uint32_t handle, uint8_t dev_type, const char* info,
                           void* client_data);
  static void QueryCallback(livox_status status, uint32_t handle,
                            LivoxLidarDiagInternalInfoResponse* response,
                            void* client_data);
  void Enqueue(Report report);
  void Run();
  void Write(uint32_t handle, const std::string& event, const std::string& payload,
             uint64_t utc_ns = 0);

  std::mutex mutex_;
  std::condition_variable condition_;
  std::map<uint32_t, bool> devices_;  // configured but undiscovered stays visible
  std::deque<Report> reports_;
  std::size_t dropped_ = 0;
  std::vector<std::pair<std::chrono::steady_clock::time_point,
                        std::function<void()>>> retries_;
  std::function<std::string(uint32_t)> stream_summary_;
  bool stopping_ = false;
  std::thread worker_;
  std::unique_ptr<odom_logging::RunLogger> logger_;
};

}  // namespace livox_ros
