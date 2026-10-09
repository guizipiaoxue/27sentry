#ifndef LIVOX_DRIVER_PTP_TIME_H_
#define LIVOX_DRIVER_PTP_TIME_H_

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <functional>
#include <map>
#include <stdexcept>
#include <vector>

namespace livox_ros {

enum class SensorStream : std::size_t { kCloud = 0, kImu = 1 };

struct TimestampConfig {
  bool require_ptp_sync = true;
  std::int64_t ptp_utc_offset_seconds = 37;
  double ptp_max_host_skew_seconds = 2.0;
  double ptp_stream_timeout_seconds = 0.5;
};

enum class TimestampError {
  kNone, kUnconfiguredDevice, kInvalidPacket, kNotPtp, kInvalidTimestamp,
  kHostSkew, kNotIncreasing
};

inline const char* TimestampErrorName(TimestampError error) {
  switch (error) {
    case TimestampError::kNone: return "none";
    case TimestampError::kUnconfiguredDevice: return "device not in lidar_configs";
    case TimestampError::kInvalidPacket: return "invalid data packet";
    case TimestampError::kNotPtp: return "packet time_type is not PTP (1)";
    case TimestampError::kInvalidTimestamp: return "invalid timestamp or UTC offset";
    case TimestampError::kHostSkew: return "UTC timestamp exceeds allowed host skew";
    case TimestampError::kNotIncreasing: return "timestamp duplicated or moved backwards";
  }
  return "unknown";
}

// The wire format is little-endian integer nanoseconds. No receiving-clock
// fallback or fitted first-packet offset is permitted.
inline bool DecodeTimestamp(const std::uint8_t* bytes, std::size_t size,
                            std::uint64_t& timestamp) {
  if (!bytes || size != sizeof(timestamp)) return false;
  timestamp = 0;
  for (std::size_t i = 0; i < size; ++i) {
    timestamp |= static_cast<std::uint64_t>(bytes[i]) << (8 * i);
  }
  return timestamp != 0;
}

struct TimestampResult {
  bool accepted = false;
  std::uint64_t utc_ns = 0;
  std::uint64_t generation = 0;
  TimestampError error = TimestampError::kNone;
};

// Caller serializes access. Each device generation invalidates queued data;
// last sample times are independent for the asynchronous cloud and IMU flows.
class TimestampGuard {
 public:
  // Called before expiration resets a device; observer must not reenter guard.
  using ExpiryObserver = std::function<void(std::uint32_t, SensorStream,
      std::uint64_t, std::uint64_t, std::uint64_t, std::uint64_t)>;
  void SetExpiryObserver(ExpiryObserver observer) { expiry_observer_ = std::move(observer); }

  void Configure(const TimestampConfig& config) {
    if (config.ptp_utc_offset_seconds < 0 ||
        config.ptp_utc_offset_seconds > 3600 ||
        !std::isfinite(config.ptp_max_host_skew_seconds) ||
        config.ptp_max_host_skew_seconds <= 0.0 ||
        config.ptp_max_host_skew_seconds > 3600.0 ||
        !std::isfinite(config.ptp_stream_timeout_seconds) ||
        config.ptp_stream_timeout_seconds <= 0.0 ||
        config.ptp_stream_timeout_seconds > 3600.0) {
      throw std::invalid_argument("invalid PTP UTC offset, host skew, or stream timeout");
    }
    config_ = config;
    devices_.clear();
  }

  void SetExpectedLidars(const std::vector<std::uint32_t>& handles) {
    devices_.clear();
    for (const auto handle : handles) devices_.emplace(handle, Device{});
  }

  TimestampResult Invalidate(std::uint32_t handle, TimestampError error,
                             std::uint64_t steady_ns) {
    TimestampResult result;
    result.error = error;
    const auto found = devices_.find(handle);
    if (found != devices_.end()) {
      Reset(found->second, steady_ns);
      result.generation = found->second.generation;
    }
    return result;
  }

  TimestampResult Accept(std::uint32_t handle, SensorStream stream,
                         std::uint8_t time_type, const std::uint8_t* bytes,
                         std::size_t size, std::uint64_t host_utc_ns,
                         std::uint64_t steady_ns) {
    Expire(steady_ns);
    const auto found = devices_.find(handle);
    if (found == devices_.end()) {
      return Invalidate(handle, TimestampError::kUnconfiguredDevice, steady_ns);
    }
    if ((config_.require_ptp_sync && time_type != 1) || time_type > 2) {
      return Invalidate(handle, TimestampError::kNotPtp, steady_ns);
    }
    std::uint64_t stamp = 0;
    if (!DecodeTimestamp(bytes, size, stamp)) {
      return Invalidate(handle, TimestampError::kInvalidTimestamp, steady_ns);
    }
    const auto offset_ns = time_type == 1 ?
        static_cast<std::uint64_t>(config_.ptp_utc_offset_seconds) * 1000000000ULL : 0;
    // ROS2 header seconds are signed int32. Keep the conversion integral.
    const auto max_ros_ns =
        static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max()) *
        1000000000ULL + 999999999ULL;
    if (stamp <= offset_ns || stamp - offset_ns > max_ros_ns) {
      return Invalidate(handle, TimestampError::kInvalidTimestamp, steady_ns);
    }
    stamp -= offset_ns;
    const auto skew = stamp > host_utc_ns ? stamp - host_utc_ns : host_utc_ns - stamp;
    if (config_.require_ptp_sync &&
        skew > static_cast<std::uint64_t>(config_.ptp_max_host_skew_seconds * 1e9)) {
      return Invalidate(handle, TimestampError::kHostSkew, steady_ns);
    }
    auto& device = found->second;
    auto& flow = device.flows[static_cast<std::size_t>(stream)];
    if (flow.last_ns != 0 &&
        (stamp <= flow.last_ns || (flow.seen && time_type != flow.time_type))) {
      return Invalidate(handle, TimestampError::kNotIncreasing, steady_ns);
    }
    flow.seen = true;
    flow.ptp = time_type == 1;
    flow.time_type = time_type;
    flow.last_ns = stamp;
    flow.arrival_ns = steady_ns;
    TimestampResult result;
    result.accepted = true;
    result.utc_ns = stamp;
    result.generation = device.generation;
    return result;
  }

  bool IsPtpLocked(std::uint64_t steady_ns) {
    Expire(steady_ns);
    if (devices_.empty()) return false;
    for (const auto& device : devices_) {
      if (steady_ns < device.second.relock_after_ns) return false;
      for (const auto& flow : device.second.flows) {
        if (!flow.seen || !flow.ptp) return false;
      }
    }
    return true;
  }

  bool IsCurrent(std::uint32_t handle, SensorStream stream,
                 std::uint64_t generation, std::uint64_t steady_ns) {
    Expire(steady_ns);
    const auto found = devices_.find(handle);
    return found != devices_.end() && found->second.generation == generation &&
        steady_ns >= found->second.relock_after_ns &&
        found->second.flows[static_cast<std::size_t>(stream)].seen;
  }

  bool IsFreshUtc(std::uint64_t stamp, std::uint64_t host_utc_ns) const {
    const auto skew = stamp > host_utc_ns ? stamp - host_utc_ns : host_utc_ns - stamp;
    return !config_.require_ptp_sync ||
        skew <= static_cast<std::uint64_t>(config_.ptp_max_host_skew_seconds * 1e9);
  }

 private:
  struct Flow {
    bool seen = false;
    bool ptp = false;
    std::uint8_t time_type = 0;
    std::uint64_t last_ns = 0;
    std::uint64_t arrival_ns = 0;
  };
  struct Device {
    std::uint64_t generation = 1;
    std::uint64_t relock_after_ns = 0;
    std::array<Flow, 2> flows;
  };
  static void Reset(Device& device, std::uint64_t steady_ns) {
    ++device.generation;
    // Hold loss long enough for two 500 ms status heartbeats. Retain timestamp
    // high-water marks so rejected duplicate/backward packets cannot restart
    // an older timeline by clearing their own monotonicity check.
    device.relock_after_ns = steady_ns + 1000000000ULL;
    for (auto& flow : device.flows) {
      flow.seen = false;
      flow.ptp = false;
      flow.arrival_ns = 0;
    }
  }
  void Expire(std::uint64_t steady_ns) {
    const auto timeout_ns = static_cast<std::uint64_t>(
        config_.ptp_stream_timeout_seconds * 1e9);
    for (auto& item : devices_) {
      for (std::size_t index = 0; index < item.second.flows.size(); ++index) {
        const auto& flow = item.second.flows[index];
        if (flow.seen && (steady_ns < flow.arrival_ns ||
                         steady_ns - flow.arrival_ns >= timeout_ns)) {
          if (expiry_observer_) expiry_observer_(item.first,
              static_cast<SensorStream>(index), flow.last_ns, flow.arrival_ns,
              steady_ns, item.second.generation);
          Reset(item.second, steady_ns);
          break;
        }
      }
    }
  }
  ExpiryObserver expiry_observer_;
  TimestampConfig config_;
  std::map<std::uint32_t, Device> devices_;
};

// Detect a crossed interval even when no packet lands on its exact boundary.
class FrameBoundary {
 public:
  bool Advance(std::uint64_t stamp, std::uint64_t interval) {
    if (interval == 0) throw std::invalid_argument("zero publish interval");
    const auto bin = stamp / interval;
    const bool crossed = initialized_ && bin != bin_;
    bin_ = bin;
    initialized_ = true;
    return crossed;
  }
 private:
  bool initialized_ = false;
  std::uint64_t bin_ = 0;
};

}  // namespace livox_ros
#endif  // LIVOX_DRIVER_PTP_TIME_H_
