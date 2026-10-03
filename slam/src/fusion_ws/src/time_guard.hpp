#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace fusion_timing
{

// Keep acquisition clocks separate from the steady clock used to expire PTP
// status. A replay explicitly skips live status/host-age checks, never the
// message's own timestamp and monotonicity checks.
class TimeGuard final
{
public:
  using Clock = std::chrono::steady_clock;
  static constexpr std::size_t kStreamCount = 4;

  TimeGuard(bool require_ptp_sync, double maximum_host_skew_seconds)
  : require_ptp_sync_(require_ptp_sync),
    maximum_host_skew_seconds_(maximum_host_skew_seconds)
  {
    if (!std::isfinite(maximum_host_skew_seconds_) ||
      maximum_host_skew_seconds_ <= 0.0)
    {
      throw std::invalid_argument("ptp_max_host_skew_seconds must be finite and positive");
    }
  }

  void updateLock(bool locked, Clock::time_point received)
  {
    status_received_ = received;
    status_seen_ = true;
    locked_ = locked;
  }

  std::string readiness(Clock::time_point current) const
  {
    if (!require_ptp_sync_) {
      return {};
    }
    if (!status_seen_) {return "waiting for /livox/ptp_locked";}
    if (!locked_) {return "driver reports PTP unlocked";}
    if (current - status_received_ >= std::chrono::seconds(2)) {
      return "PTP lock status expired (2 seconds without refresh)";
    }
    return {};
  }

  std::string validate(
    std::size_t stream, std::int32_t seconds,
    std::uint32_t nanoseconds, std::int64_t host_nanoseconds,
    const std::uint64_t * timebase = nullptr)
  {
    if (stream >= kStreamCount) {throw std::out_of_range("unknown sensor stream");}
    if (seconds <= 0 || nanoseconds >= 1000000000U) {
      return "invalid header stamp (positive sec and nanosec < 1e9 required)";
    }
    const std::int64_t stamp = static_cast<std::int64_t>(seconds) * 1000000000LL +
      static_cast<std::int64_t>(nanoseconds);
    if (timebase && *timebase != static_cast<std::uint64_t>(stamp)) {
      return "CustomMsg timebase differs from header stamp";
    }
    if (require_ptp_sync_ &&
      std::abs(static_cast<long double>(stamp) - host_nanoseconds) >
      static_cast<long double>(maximum_host_skew_seconds_) * 1.0e9L)
    {
      return "acquisition stamp exceeds ptp_max_host_skew_seconds from host UTC";
    }
    if (last_stamp_[stream] != 0 && stamp <= last_stamp_[stream]) {
      return "non-increasing acquisition stamp (clock jump or duplicate)";
    }
    last_stamp_[stream] = stamp;
    return {};
  }

  void resetTimeline() {last_stamp_.fill(0);}

private:
  bool require_ptp_sync_;
  double maximum_host_skew_seconds_;
  bool status_seen_ = false;
  bool locked_ = false;
  Clock::time_point status_received_{};
  std::array<std::int64_t, kStreamCount> last_stamp_{};
};

}  // namespace fusion_timing
