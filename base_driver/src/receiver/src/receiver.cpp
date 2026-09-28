#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <receiver/msg/chassis_state.hpp>
#include <receiver/msg/game_state.hpp>
#include <receiver/msg/gimbal_dynamics.hpp>
#include <receiver/msg/gimbal_state.hpp>
#include <receiver/msg/position_pair.hpp>
#include <receiver/msg/uplink_frame.hpp>

namespace {

using SteadyClock = std::chrono::steady_clock;
constexpr size_t kFrameSize = 15;
constexpr uint8_t kFrameHead = 0x21;

speed_t baud_constant(int baud) {
  switch (baud) {
    case 9600: return B9600;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: throw std::invalid_argument("unsupported baud_rate");
  }
}

// Uplink frames use the reflected CRC8 variant from the firmware protocol.
uint8_t uplink_crc8(const uint8_t* data, size_t length) {
  uint8_t crc = 0xff;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) ? static_cast<uint8_t>((crc >> 1) ^ 0x8c) : crc >> 1;
    }
  }
  return crc;
}

uint16_t read_u16(const uint8_t* bytes) {
  return static_cast<uint16_t>(bytes[0] | (static_cast<uint16_t>(bytes[1]) << 8));
}

int16_t read_i16(const uint8_t* bytes) {
  return static_cast<int16_t>(read_u16(bytes));
}

uint32_t read_u32(const uint8_t* bytes) {
  return static_cast<uint32_t>(bytes[0]) |
         (static_cast<uint32_t>(bytes[1]) << 8) |
         (static_cast<uint32_t>(bytes[2]) << 16) |
         (static_cast<uint32_t>(bytes[3]) << 24);
}

float read_f32(const uint8_t* bytes) {
  const uint32_t bits = read_u32(bytes);
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

class Receiver : public rclcpp::Node {
 public:
  Receiver() : Node("receiver") {
    static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
                  "protocol requires IEEE 754 float32");
    device_ = declare_parameter<std::string>("device", "/dev/ttyACM0");
    baud_ = baud_constant(declare_parameter<int>("baud_rate", 115200));
    const int poll_ms = declare_parameter<int>("poll_period_ms", 5);
    reconnect_ms_ = declare_parameter<int>("reconnect_period_ms", 1000);
    frame_id_ = declare_parameter<std::string>("frame_id", "gimbal");
    if (poll_ms < 1 || reconnect_ms_ < 1) {
      throw std::invalid_argument("poll and reconnect periods must be positive");
    }

    // Publish every valid frame in raw form, then expose common fields separately.
    raw_pub_ = create_publisher<receiver::msg::UplinkFrame>("/base/uplink/raw", 50);
    gimbal_pub_ = create_publisher<receiver::msg::GimbalState>("/base/gimbal/state", 20);
    chassis_pub_ = create_publisher<receiver::msg::ChassisState>("/base/chassis/state", 20);
    game_pub_ = create_publisher<receiver::msg::GameState>("/base/game/state", 10);
    position_pub_ = create_publisher<receiver::msg::PositionPair>("/base/position/pair", 10);
    dynamics_pub_ = create_publisher<receiver::msg::GimbalDynamics>("/base/gimbal/dynamics", 20);
    timer_ = create_wall_timer(std::chrono::milliseconds(poll_ms), [this] { poll_serial(); });
  }

  ~Receiver() override { close_serial(); }

 private:
  void close_serial() {
    if (fd_ >= 0) {
      close(fd_);
      fd_ = -1;
    }
    buffer_.clear();
  }

  // Reopen after USB disconnect without blocking the ROS executor.
  bool ensure_serial() {
    if (fd_ >= 0) return true;
    const auto now = SteadyClock::now();
    if (now < next_reconnect_) return false;
    next_reconnect_ = now + std::chrono::milliseconds(reconnect_ms_);
    const int fd = open(device_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), reconnect_ms_,
                           "Cannot open %s: %s", device_.c_str(), std::strerror(errno));
      return false;
    }
    termios settings{};
    if (tcgetattr(fd, &settings) != 0) {
      close(fd);
      return false;
    }
    cfmakeraw(&settings);
    cfsetispeed(&settings, baud_);
    cfsetospeed(&settings, baud_);
    settings.c_cflag |= CLOCAL | CREAD;
    if (tcsetattr(fd, TCSANOW, &settings) != 0) {
      close(fd);
      return false;
    }
    fd_ = fd;
    RCLCPP_INFO(get_logger(), "Receiving from %s", device_.c_str());
    return true;
  }

  // Read all available bytes and retain any incomplete frame for the next poll.
  void poll_serial() {
    if (!ensure_serial()) return;
    std::array<uint8_t, 512> chunk{};
    while (true) {
      const ssize_t count = read(fd_, chunk.data(), chunk.size());
      if (count > 0) {
        buffer_.insert(buffer_.end(), chunk.begin(), chunk.begin() + count);
      } else if (count < 0 && errno == EINTR) {
        continue;
      } else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        RCLCPP_WARN(get_logger(), "Serial read failed: %s", std::strerror(errno));
        close_serial();
        return;
      } else {
        break;
      }
    }

    // A bad CRC advances one byte so the next possible header is retained.
    while (buffer_.size() >= kFrameSize) {
      const auto head = std::find(buffer_.begin(), buffer_.end(), kFrameHead);
      buffer_.erase(buffer_.begin(), head);
      if (buffer_.size() < kFrameSize) break;
      if (buffer_[1] > 11 || uplink_crc8(buffer_.data(), 14) != buffer_[14]) {
        buffer_.erase(buffer_.begin());
        continue;
      }
      publish_frame(buffer_.data());
      buffer_.erase(buffer_.begin(), buffer_.begin() + kFrameSize);
    }
    if (buffer_.size() > 4096) buffer_.erase(buffer_.begin(), buffer_.end() - 14);
  }

  void publish_frame(const uint8_t* frame) {
    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = frame_id_;
    receiver::msg::UplinkFrame raw;
    raw.header = header;
    raw.type_id = frame[1];
    std::copy(frame + 2, frame + 14, raw.payload.begin());
    raw_pub_->publish(raw);

    // Decode fields used by navigation and common status consumers.
    switch (frame[1]) {
      case 0: {
        receiver::msg::GimbalState state;
        state.header = header;
        state.yaw_deg = read_f32(frame + 2);
        state.pitch_deg = read_f32(frame + 6);
        if (!std::isfinite(state.yaw_deg) || !std::isfinite(state.pitch_deg)) break;
        state.remain_bullet = read_u16(frame + 10);
        state.shoot_state = frame[12] & 0x03;
        state.capacitor_voltage_v = static_cast<float>(frame[13]) * 2.0f;
        gimbal_pub_->publish(state);
        break;
      }
      case 1: {
        receiver::msg::GameState state;
        state.header = header;
        const uint16_t flags = read_u16(frame + 2);
        state.game_started = flags & 0x01;
        state.red_team = flags & 0x04;
        state.enemy_outpost_hp = (flags >> 3) & 0x3f;
        state.self_outpost_hp = (flags >> 9) & 0x3f;
        state.bullet_remaining_17mm = read_u16(frame + 4);
        state.stage_remaining_sec = read_u16(frame + 6) * 0.5f;
        state.self_hp = read_u16(frame + 8);
        state.event_data = read_u32(frame + 10);
        game_pub_->publish(state);
        break;
      }
      case 5: {
        receiver::msg::PositionPair pair;
        pair.header = header;
        pair.friend_type = frame[2];
        pair.friend_x_m = read_i16(frame + 3) * 0.01f;
        pair.friend_y_m = read_i16(frame + 5) * 0.01f;
        pair.enemy_type = frame[7];
        pair.enemy_x_m = read_i16(frame + 8) * 0.01f;
        pair.enemy_y_m = read_i16(frame + 10) * 0.01f;
        pair.bullet_speed_mps = read_u16(frame + 12) * 0.01f;
        position_pub_->publish(pair);
        break;
      }
      case 6: {
        receiver::msg::ChassisState state;
        state.header = header;
        state.uwb_yaw_deg = read_u16(frame + 2);
        state.damage_difference = read_i16(frame + 4);
        state.steer_angle_deg = read_i16(frame + 6) * 0.1f;
        state.yaw_rate_radps = read_i16(frame + 8) * 0.01f;
        state.velocity_x_mps = read_i16(frame + 10) * 0.01f;
        state.velocity_y_mps = read_i16(frame + 12) * 0.01f;
        chassis_pub_->publish(state);
        break;
      }
      case 11: {
        receiver::msg::GimbalDynamics state;
        state.header = header;
        state.yaw_rate_degps = read_i16(frame + 2) * 0.1f;
        state.pitch_rate_degps = read_i16(frame + 4) * 0.1f;
        state.yaw_accel_degps2 = read_i16(frame + 6);
        state.pitch_accel_degps2 = read_i16(frame + 8);
        state.sample_tick_ms = read_u32(frame + 10);
        dynamics_pub_->publish(state);
        break;
      }
      default: break;
    }
  }

  std::string device_;
  std::string frame_id_;
  speed_t baud_{};
  int reconnect_ms_{};
  int fd_ = -1;
  SteadyClock::time_point next_reconnect_{};
  std::vector<uint8_t> buffer_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<receiver::msg::UplinkFrame>::SharedPtr raw_pub_;
  rclcpp::Publisher<receiver::msg::GimbalState>::SharedPtr gimbal_pub_;
  rclcpp::Publisher<receiver::msg::ChassisState>::SharedPtr chassis_pub_;
  rclcpp::Publisher<receiver::msg::GameState>::SharedPtr game_pub_;
  rclcpp::Publisher<receiver::msg::PositionPair>::SharedPtr position_pub_;
  rclcpp::Publisher<receiver::msg::GimbalDynamics>::SharedPtr dynamics_pub_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Receiver>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("receiver"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
