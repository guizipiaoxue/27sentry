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
#include <poll.h>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <receiver/msg/gimbal_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

namespace {

using SteadyClock = std::chrono::steady_clock;
constexpr uint8_t kHead = 0x21;

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

void write_u16(uint8_t* dest, uint16_t value) {
  dest[0] = static_cast<uint8_t>(value);
  dest[1] = static_cast<uint8_t>(value >> 8);
}

void write_u32(uint8_t* dest, uint32_t value) {
  for (int i = 0; i < 4; ++i) dest[i] = static_cast<uint8_t>(value >> (8 * i));
}

void write_f32(uint8_t* dest, float value) {
  uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  write_u32(dest, bits);
}

// Coordinates and path fragments have distinct firmware CRC variants.
uint8_t coordinate_crc8(const uint8_t* data, size_t length) {
  uint8_t crc = 0xff;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x31)
                         : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

uint16_t path_crc16(const uint8_t* data, size_t length) {
  uint16_t crc = 0xffff;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) ? static_cast<uint16_t>((crc >> 1) ^ 0x8408)
                       : static_cast<uint16_t>(crc >> 1);
    }
  }
  return crc;
}

class Sender : public rclcpp::Node {
 public:
  Sender() : Node("sender") {
    static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
                  "protocol requires IEEE 754 float32");
    device_ = declare_parameter<std::string>("device", "/dev/ttyACM0");
    baud_ = baud_constant(declare_parameter<int>("baud_rate", 115200));
    const int period_ms = declare_parameter<int>("control_period_ms", 20);
    reconnect_ms_ = declare_parameter<int>("reconnect_period_ms", 1000);
    write_timeout_ms_ = declare_parameter<int>("write_timeout_ms", 20);
    command_timeout_ms_ = declare_parameter<int>("command_timeout_ms", 150);
    feedback_timeout_ms_ = declare_parameter<int>("feedback_timeout_ms", 500);
    coordinate_max_age_ms_ = declare_parameter<int>("coordinate_max_age_ms", 2000);
    coordinate_min_period_ms_ = declare_parameter<int>("coordinate_min_period_ms", 100);
    max_speed_mps_ = declare_parameter<double>("max_speed_mps", 2.4);
    if (period_ms < 1 || period_ms >= 200 || reconnect_ms_ < 1 ||
        write_timeout_ms_ < 1 || command_timeout_ms_ < 1 ||
        feedback_timeout_ms_ < 1 || coordinate_max_age_ms_ < 1 ||
        coordinate_min_period_ms_ < 1 ||
        max_speed_mps_ <= 0.0 || max_speed_mps_ > 2.54) {
      throw std::invalid_argument("invalid sender timing or speed parameter");
    }

    // The sender is the only writer; each subscription maps to one downlink type.
    velocity_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, [this](geometry_msgs::msg::Twist::SharedPtr msg) {
          if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y)) return;
          velocity_x_ = msg->linear.x;
          velocity_y_ = msg->linear.y;
          last_velocity_ = SteadyClock::now();
        });
    feedback_sub_ = create_subscription<receiver::msg::GimbalState>(
        "/base/gimbal/state", 10, [this](receiver::msg::GimbalState::SharedPtr msg) {
          if (!std::isfinite(msg->yaw_deg) || !std::isfinite(msg->pitch_deg)) return;
          feedback_yaw_ = msg->yaw_deg;
          feedback_pitch_ = msg->pitch_deg;
          last_feedback_ = SteadyClock::now();
        });
    angles_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
        "/base/target_angles", 10, [this](geometry_msgs::msg::Vector3::SharedPtr msg) {
          if (!std::isfinite(msg->x) || !std::isfinite(msg->y)) return;
          target_yaw_ = msg->x;
          target_pitch_ = msg->y;
          angles_received_ = true;
        });
    fire_sub_ = create_subscription<std_msgs::msg::UInt8>(
        "/base/fire_code", 10, [this](std_msgs::msg::UInt8::SharedPtr msg) {
          fire_code_ = msg->data;
        });
    sentry_sub_ = create_subscription<std_msgs::msg::UInt32>(
        "/base/sentry_command", 10, [this](std_msgs::msg::UInt32::SharedPtr msg) {
          if (msg->data & 0xfe000000U) return;
          std::array<uint8_t, 6> frame{kHead, 0x01};
          write_u32(frame.data() + 2, msg->data);
          send_frame(frame);
        });
    path_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
        "/base/map_path_payload", 2, [this](std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
          send_map_path(msg->data);
        });
    custom_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
        "/base/custom_info_payload", 2, [this](std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
          if (msg->data.size() != 34) {
            RCLCPP_WARN(get_logger(), "custom info requires exactly 34 bytes");
            return;
          }
          std::array<uint8_t, 36> frame{};
          frame[0] = kHead;
          frame[1] = 0x03;
          std::copy(msg->data.begin(), msg->data.end(), frame.begin() + 2);
          send_frame(frame);
        });
    coordinate_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        "/base/sentry_coordinate", 10,
        [this](geometry_msgs::msg::PointStamped::SharedPtr msg) { send_coordinate(*msg); });
    trajectory_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "/base/gimbal_trajectory", 10,
        [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) { send_trajectory(msg->data); });
    timer_ = create_wall_timer(std::chrono::milliseconds(period_ms),
                               [this] { send_control(); });
  }

  ~Sender() override {
    if (fd_ >= 0) {
      send_control(true);
      close(fd_);
    }
  }

 private:
  bool ensure_serial() {
    if (fd_ >= 0) return true;
    const auto now = SteadyClock::now();
    if (now < next_reconnect_) return false;
    next_reconnect_ = now + std::chrono::milliseconds(reconnect_ms_);
    const int fd = open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
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
    RCLCPP_INFO(get_logger(), "Sending to %s", device_.c_str());
    return true;
  }

  // Complete each frame before another ROS callback can write to the port.
  template <size_t N>
  bool send_frame(const std::array<uint8_t, N>& frame) {
    if (!ensure_serial()) return false;
    size_t offset = 0;
    const auto deadline = SteadyClock::now() + std::chrono::milliseconds(write_timeout_ms_);
    while (offset < N) {
      const ssize_t count = write(fd_, frame.data() + offset, N - offset);
      if (count > 0) {
        offset += static_cast<size_t>(count);
        continue;
      }
      if (count < 0 && errno == EINTR) continue;
      if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK) &&
          SteadyClock::now() < deadline) {
        pollfd wait{fd_, POLLOUT, 0};
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - SteadyClock::now()).count();
        if (poll(&wait, 1, std::max(1, static_cast<int>(remaining))) > 0 &&
            !(wait.revents & (POLLERR | POLLHUP | POLLNVAL))) continue;
      }
      RCLCPP_WARN(get_logger(), "Serial write failed or timed out");
      close(fd_);
      fd_ = -1;
      return false;
    }
    return true;
  }

  // The heartbeat frame continues at a fixed rate, with stale velocity set to zero.
  void send_control(bool force_stop = false) {
    const auto now = SteadyClock::now();
    const bool feedback_fresh = now - last_feedback_ <
                                std::chrono::milliseconds(feedback_timeout_ms_);
    const bool command_fresh = now - last_velocity_ <
                               std::chrono::milliseconds(command_timeout_ms_);
    const double vx = (!force_stop && feedback_fresh && command_fresh) ? velocity_x_ : 0.0;
    const double vy = (!force_stop && feedback_fresh && command_fresh) ? velocity_y_ : 0.0;
    std::array<uint8_t, 13> frame{};
    frame[0] = kHead;
    frame[1] = 0x00;
    frame[2] = static_cast<uint8_t>(static_cast<int8_t>(std::lround(
        std::clamp(vx, -max_speed_mps_, max_speed_mps_) * 50.0)));
    frame[3] = static_cast<uint8_t>(static_cast<int8_t>(std::lround(
        std::clamp(vy, -max_speed_mps_, max_speed_mps_) * 50.0)));
    write_f32(frame.data() + 4, static_cast<float>(angles_received_ ? target_yaw_ : feedback_yaw_));
    write_f32(frame.data() + 8, static_cast<float>(angles_received_ ? target_pitch_ : feedback_pitch_));
    frame[12] = force_stop ? 0 : fire_code_;
    send_frame(frame);
  }

  // Split the 105-byte logical map payload into the two required 64-byte frames.
  void send_map_path(const std::vector<uint8_t>& payload) {
    if (payload.size() != 105 || payload[0] < 1 || payload[0] > 3) {
      RCLCPP_WARN(get_logger(), "map path requires 105 bytes and intention 1..3");
      return;
    }
    if (!ensure_serial()) return;
    const uint8_t sequence = path_sequence_++;
    for (uint8_t fragment = 0; fragment < 2; ++fragment) {
      std::array<uint8_t, 64> frame{};
      frame[0] = kHead;
      frame[1] = 0x02;
      frame[2] = sequence;
      frame[3] = fragment;
      frame[4] = 2;
      frame[5] = fragment == 0 ? 56 : 49;
      const size_t start = fragment == 0 ? 0 : 56;
      std::copy(payload.begin() + start, payload.begin() + start + frame[5],
                frame.begin() + 6);
      write_u16(frame.data() + 62, path_crc16(frame.data(), 62));
      if (!send_frame(frame)) return;
    }
  }

  // Reject stale or out-of-range coordinates before centimeter quantization.
  void send_coordinate(const geometry_msgs::msg::PointStamped& point) {
    if (!std::isfinite(point.point.x) || !std::isfinite(point.point.y)) return;
    const auto current = SteadyClock::now();
    if (current - last_coordinate_sent_ <
        std::chrono::milliseconds(coordinate_min_period_ms_)) return;
    if (point.header.stamp.sec != 0 || point.header.stamp.nanosec != 0) {
      const auto age = now() - rclcpp::Time(point.header.stamp);
      if (age.nanoseconds() < 0 || age > rclcpp::Duration::from_seconds(
          coordinate_max_age_ms_ / 1000.0)) return;
    }
    const long x = std::lround(point.point.x * 100.0);
    const long y = std::lround(point.point.y * 100.0);
    if (x < INT16_MIN || x > INT16_MAX || y < INT16_MIN || y > INT16_MAX) return;
    std::array<uint8_t, 17> frame{};
    frame[0] = kHead;
    frame[1] = 0x04;
    write_u16(frame.data() + 2, static_cast<uint16_t>(static_cast<int16_t>(x)));
    write_u16(frame.data() + 4, static_cast<uint16_t>(static_cast<int16_t>(y)));
    frame[16] = coordinate_crc8(frame.data(), 16);
    if (send_frame(frame)) last_coordinate_sent_ = current;
  }

  void send_trajectory(const std::vector<float>& values) {
    if (values.size() != 6 ||
        !std::all_of(values.begin(), values.end(), [](float v) { return std::isfinite(v); })) {
      RCLCPP_WARN(get_logger(), "trajectory requires six finite float32 values");
      return;
    }
    std::array<uint8_t, 26> frame{};
    frame[0] = kHead;
    frame[1] = 0x05;
    for (size_t i = 0; i < values.size(); ++i) write_f32(frame.data() + 2 + i * 4, values[i]);
    send_frame(frame);
  }

  std::string device_;
  speed_t baud_{};
  int reconnect_ms_{};
  int write_timeout_ms_{};
  int command_timeout_ms_{};
  int feedback_timeout_ms_{};
  int coordinate_max_age_ms_{};
  int coordinate_min_period_ms_{};
  double max_speed_mps_{};
  int fd_ = -1;
  uint8_t path_sequence_ = 0;
  uint8_t fire_code_ = 0;
  bool angles_received_ = false;
  double velocity_x_ = 0.0;
  double velocity_y_ = 0.0;
  double feedback_yaw_ = 0.0;
  double feedback_pitch_ = 0.0;
  double target_yaw_ = 0.0;
  double target_pitch_ = 0.0;
  SteadyClock::time_point next_reconnect_{};
  SteadyClock::time_point last_velocity_{};
  SteadyClock::time_point last_feedback_{};
  SteadyClock::time_point last_coordinate_sent_{};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
  rclcpp::Subscription<receiver::msg::GimbalState>::SharedPtr feedback_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr angles_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr fire_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr sentry_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr custom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr coordinate_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr trajectory_sub_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Sender>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("sender"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
