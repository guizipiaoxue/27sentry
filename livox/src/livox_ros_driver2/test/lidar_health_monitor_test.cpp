#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>

#include "comm/lidar_health_monitor.h"

namespace {
struct Response {
  std::vector<uint8_t> bytes{0, 0, 0};
  uint16_t count = 0;
  template <typename T>
  void Add(uint16_t key, const T& value) {
    const uint16_t size = sizeof(value);
    const auto offset = bytes.size();
    bytes.resize(offset + 4 + size);
    std::memcpy(bytes.data() + offset, &key, sizeof(key));
    std::memcpy(bytes.data() + offset + 2, &size, sizeof(size));
    std::memcpy(bytes.data() + offset + 4, &value, size);
    ++count;
    std::memcpy(bytes.data() + 1, &count, sizeof(count));
  }
  const LivoxLidarDiagInternalInfoResponse& Get() const {
    static_assert(offsetof(LivoxLidarDiagInternalInfoResponse, data) == 3);
    return *reinterpret_cast<const LivoxLidarDiagInternalInfoResponse*>(bytes.data());
  }
};
}  // namespace

int main() {
  std::string payload;
  Response nominal;
  nominal.Add(0x1234, uint32_t{123});  // Unrelated SDK fields must not shift decoding.
  nominal.Add(kKeyCurWorkState, uint8_t{1});
  nominal.Add(kKeyCoreTemp, int32_t{6500});
  nominal.Add(kKeyLidarDiagStatus, uint16_t{0x3210});
  const uint32_t hms[8] = {0x01020002, 0x01150004, 0, 0, 0, 0, 0, 0};
  nominal.Add(kKeyHmsCode, hms);
  nominal.Add(kKeyTimeSyncType, uint8_t{1});
  assert(livox_ros::FormatLidarHealth(nominal.Get(), payload));
  assert(payload.find("core_temp_raw=6500 core_temp_c=65.00") != std::string::npos);
  assert(payload.find("work_state=1") != std::string::npos);
  assert(payload.find("diag_system=0 diag_scan=1 diag_ranging=2 diag_communication=3") != std::string::npos);
  assert(payload.find("hms=[0x01020002,0x01150004,0x00000000") != std::string::npos);
  assert(payload.find("time_sync_type=1") != std::string::npos);

  Response cold;
  cold.Add(kKeyCoreTemp, int32_t{-1234});
  assert(livox_ros::FormatLidarHealth(cold.Get(), payload));
  assert(payload.find("core_temp_c=-12.34") != std::string::npos);
  Response missing;
  missing.Add(kKeyCurWorkState, uint8_t{1});
  assert(!livox_ros::FormatLidarHealth(missing.Get(), payload));
  assert(payload.empty());  // Do not carry an earlier temperature into a failure.
  Response short_temperature;
  short_temperature.Add(kKeyCoreTemp, uint16_t{65});
  assert(!livox_ros::FormatLidarHealth(short_temperature.Get(), payload));
  Response short_hms;
  short_hms.Add(kKeyCoreTemp, int32_t{5000});
  short_hms.Add(kKeyHmsCode, uint32_t{0});
  assert(!livox_ros::FormatLidarHealth(short_hms.Get(), payload));
  Response duplicate;
  duplicate.Add(kKeyCoreTemp, int32_t{5000});
  duplicate.Add(kKeyCoreTemp, int32_t{6000});
  assert(!livox_ros::FormatLidarHealth(duplicate.Get(), payload));
  Response failed;
  failed.bytes[0] = 1;
  assert(!livox_ros::FormatLidarHealth(failed.Get(), payload));

  livox_ros::LidarHealthFields patch;
  assert(livox_ros::ParseLidarHealthPush(R"({"core_temp":6500,"cur_work_state":1,"time_sync_type":1,"lidar_diag_status":0,"hms_code":[0,0,0,0,0,0,0,0],"future_field":123})", patch));
  livox_ros::LidarHealthState state;
  constexpr uint64_t start = 1000000000ULL;
  state.Apply(patch, start);
  assert(state.Summary(start).find("device_status=normal health_fresh=1") != std::string::npos);
  assert(livox_ros::ParseLidarHealthPush(R"({"core_temp":7000})", patch));
  state.Apply(patch, start + 16000000000ULL);
  assert(state.fields.has_hms && state.fields.hms[0] == 0);
  assert(state.Summary(start + 16000000000ULL).find("device_status=unknown health_fresh=0") != std::string::npos);
  state.Apply(patch, start);  // An older callback must not overwrite a newer field.
  assert(state.updated[0] == start + 16000000000ULL);
  assert(livox_ros::ParseLidarHealthPush(R"({"hms_code":[18153476,0,0,0,0,0,0,0]})", patch));
  state.Apply(patch, start + 17000000000ULL);  // 0x01150004 thermal shutdown
  assert(state.Summary(start + 17000000000ULL).find("device_status=fatal") != std::string::npos);
  assert(livox_ros::DescribeHms(0x01150004).find("severity=fatal reason=thermal_shutdown") != std::string::npos);
  assert(livox_ros::DescribeHms(0xabcd0003).find("unknown_hms_id") != std::string::npos);
  for (uint16_t id : {0x0102,0x0103,0x0104,0x0105,0x0111,0x0112,0x0113,0x0114,
                     0x0115,0x0116,0x0117,0x0118,0x0201,0x0210,0x0219,0x021c,
                     0x0304,0x0401,0x0402,0x0403,0x0404,0x0405,0x0406,0x0407,
                     0x0408,0x0409,0x040a})
    assert(livox_ros::DescribeHms((uint32_t(id) << 16) | 2).find("unknown_hms_id") == std::string::npos);
  assert(!livox_ros::ParseLidarHealthPush(R"({"core_temp":1,"hms_code":[0]})", patch));
  assert(!patch.has_temperature);  // Invalid partial JSON must not update state.
  assert(!livox_ros::ParseLidarHealthPush(R"({"core_temp":1,"core_temp":2})", patch));
  assert(!livox_ros::ParseLidarHealthPush("not JSON", patch));
  assert(!livox_ros::ParseLidarHealthPush(R"({"lidar_diag_status":65536})", patch));
  assert(!livox_ros::ParseLidarHealthPush(R"({"hms_code":[-1,0,0,0,0,0,0,0]})", patch));
  assert(livox_ros::ParseLidarHealthPush(R"({"cur_work_state":4})", patch));
  livox_ros::LidarHealthState work_error;
  work_error.Apply(patch, start);
  assert(work_error.Summary(start).find("device_status=error") != std::string::npos);
  LivoxLidarAsyncControlResponse response{};
  assert(livox_ros::LidarCommandSucceeded(0, &response));
  response.ret_code = 1;
  assert(!livox_ros::LidarCommandSucceeded(0, &response));
  assert(!livox_ros::LidarCommandSucceeded(-4, nullptr));
  assert(!livox_ros::LidarCommandSucceeded(0, nullptr));

  char temp_directory[] = "/tmp/sentry-health-test-XXXXXX";
  assert(mkdtemp(temp_directory));
  const char* prior_env = std::getenv("SENTRY_RUNLOG_DIR");
  const bool had_env = prior_env != nullptr;
  const std::string prior = prior_env ? prior_env : "";
  setenv("SENTRY_RUNLOG_DIR", temp_directory, 1);
  {
    livox_ros::LidarHealthMonitor live;
    live.SetExpectedDevices({1, 2});
    live.SetStreamSummary([](uint32_t) { return " cloud_status=receiving imu_status=receiving"; });
    live.Start();  // Does not initialize/start a SDK instance or sensor sockets.
    const char* hot = R"({"core_temp":6500,"cur_work_state":1,"time_sync_type":1,"lidar_diag_status":0,"hms_code":[18153476,0,0,0,0,0,0,0]})";
    live.ObservePush(1, kLivoxLidarTypeMid360, hot);
    live.ObservePush(1, kLivoxLidarTypeMid360, hot);  // One ACTIVE per transition.
    live.ObservePush(1, kLivoxLidarTypeMid360, R"({"hms_code":[0,0,0,0,0,0,0,0]})");
    live.ObservePush(1, kLivoxLidarTypeMid360, "broken\nJSON");
    live.RecordCommand(1, "imu_enable", -4, nullptr);
    response.ret_code = 0;
    live.RecordCommand(1, "imu_enable", 0, &response);
    live.RecordSubmit(1, "work_mode", -9);
    live.RecordDriverEvent("PTP_REJECT", "ip=1.0.0.0 stream=1 reason=not_ptp");
    std::this_thread::sleep_for(std::chrono::milliseconds(1200));
    live.Stop();
  }
  std::ifstream log(std::string(temp_directory) + "/lidar_health.log");
  const std::string contents((std::istreambuf_iterator<char>(log)), {});
  const auto active = contents.find("LIDAR_HMS_ACTIVE");
  assert(active != std::string::npos && contents.find("LIDAR_HMS_ACTIVE", active + 1) == std::string::npos);
  assert(contents.find("LIDAR_HMS_CLEARED") != std::string::npos);
  assert(contents.find("LIDAR_PUSH_INVALID") != std::string::npos);
  assert(contents.find("broken\\nJSON") != std::string::npos);
  assert(contents.find("LIDAR_COMMAND_FAILED") != std::string::npos);
  assert(contents.find("sdk_reason=send_failed") != std::string::npos);
  assert(contents.find("status=not_discovered") != std::string::npos);
  assert(contents.find("failed_commands=[work_mode]") != std::string::npos);
  assert(contents.find("source=driver stream=1 reason=not_ptp") != std::string::npos);
  assert(contents.find("LIDAR_STATUS_CHANGED") != std::string::npos);

  // Stop must wake the timer immediately without waiting for five seconds.
  livox_ros::LidarHealthMonitor monitor;
  const auto begin = std::chrono::steady_clock::now();
  monitor.Start();
  monitor.Stop();
  monitor.Stop();
  assert(std::chrono::steady_clock::now() - begin < std::chrono::seconds(1));
  if (had_env) setenv("SENTRY_RUNLOG_DIR", prior.c_str(), 1);
  else unsetenv("SENTRY_RUNLOG_DIR");
  std::filesystem::remove_all(temp_directory);
  std::cout << "lidar health decoding and shutdown checks passed\n";
}
