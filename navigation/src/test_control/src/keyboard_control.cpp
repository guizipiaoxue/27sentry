#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <limits>
#include <poll.h>
#include <signal.h>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;
constexpr auto kSendPeriod = std::chrono::milliseconds(50);
constexpr auto kKeyTimeout = std::chrono::milliseconds(180);
constexpr auto kFeedbackTimeout = std::chrono::milliseconds(500);
volatile sig_atomic_t stop_requested = 0;

void handle_signal(int) { stop_requested = 1; }

class Terminal {
 public:
  Terminal() {
    if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &old_) != 0) {
      throw std::runtime_error("keyboard input requires a terminal");
    }
    termios raw = old_;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
      throw std::runtime_error("cannot configure terminal");
    }
  }

  ~Terminal() { tcsetattr(STDIN_FILENO, TCSANOW, &old_); }
  Terminal(const Terminal&) = delete;
  Terminal& operator=(const Terminal&) = delete;

 private:
  termios old_{};
};

class SerialPort {
 public:
  explicit SerialPort(const std::string& path) {
    fd_ = open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      throw std::runtime_error("cannot open " + path + ": " + std::strerror(errno));
    }
    termios settings{};
    if (tcgetattr(fd_, &settings) != 0) {
      const std::string error = std::strerror(errno);
      close(fd_);
      throw std::runtime_error("cannot configure " + path + ": " + error);
    }
    cfmakeraw(&settings);
    cfsetispeed(&settings, B115200);
    cfsetospeed(&settings, B115200);
    settings.c_cflag |= CLOCAL | CREAD;
    if (tcsetattr(fd_, TCSANOW, &settings) != 0) {
      const std::string error = std::strerror(errno);
      close(fd_);
      throw std::runtime_error("cannot configure " + path + ": " + error);
    }
  }

  ~SerialPort() { close(fd_); }
  SerialPort(const SerialPort&) = delete;
  SerialPort& operator=(const SerialPort&) = delete;
  int fd() const { return fd_; }

  void send(const std::array<uint8_t, 13>& frame) const {
    size_t sent = 0;
    while (sent < frame.size()) {
      const ssize_t count = write(fd_, frame.data() + sent, frame.size() - sent);
      if (count > 0) {
        sent += static_cast<size_t>(count);
      } else if (count < 0 && errno == EINTR) {
        continue;
      } else if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        pollfd wait{fd_, POLLOUT, 0};
        if (poll(&wait, 1, 50) <= 0 || (wait.revents & (POLLERR | POLLHUP | POLLNVAL))) {
          throw std::runtime_error("serial write timed out");
        }
      } else {
        throw std::runtime_error("serial write failed");
      }
    }
  }

 private:
  int fd_ = -1;
};

uint8_t crc8_uplink(const uint8_t* data, size_t length) {
  uint8_t crc = 0xFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1) ? static_cast<uint8_t>((crc >> 1) ^ 0x8C) : crc >> 1;
    }
  }
  return crc;
}

float read_float_le(const uint8_t* bytes) {
  const uint32_t bits = static_cast<uint32_t>(bytes[0]) |
                        (static_cast<uint32_t>(bytes[1]) << 8) |
                        (static_cast<uint32_t>(bytes[2]) << 16) |
                        (static_cast<uint32_t>(bytes[3]) << 24);
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

void write_float_le(uint8_t* bytes, float value) {
  uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  for (int i = 0; i < 4; ++i) {
    bytes[i] = static_cast<uint8_t>(bits >> (8 * i));
  }
}

struct Attitude {
  float yaw = 0;
  float pitch = 0;
  Clock::time_point received{};
  bool valid = false;
};

void receive_attitude(int fd, std::vector<uint8_t>& buffer, Attitude& attitude,
                      float& yaw_target) {
  std::array<uint8_t, 256> incoming{};
  while (true) {
    const ssize_t count = read(fd, incoming.data(), incoming.size());
    if (count > 0) {
      buffer.insert(buffer.end(), incoming.begin(), incoming.begin() + count);
    } else if (count < 0 && errno == EINTR) {
      continue;
    } else if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      break;
    } else {
      throw std::runtime_error("serial connection closed");
    }
  }

  while (buffer.size() >= 15) {
    if (buffer.front() != 0x21 || crc8_uplink(buffer.data(), 14) != buffer[14]) {
      buffer.erase(buffer.begin());
      continue;
    }
    if (buffer[1] == 0) {
      const float yaw = read_float_le(buffer.data() + 2);
      const float pitch = read_float_le(buffer.data() + 6);
      if (std::isfinite(yaw) && std::isfinite(pitch)) {
        if (!attitude.valid) yaw_target = yaw;
        attitude = {yaw, pitch, Clock::now(), true};
      }
    }
    buffer.erase(buffer.begin(), buffer.begin() + 15);
  }
}

struct Axis {
  int direction = 0;
  Clock::time_point last_key{};

  int value(Clock::time_point now) const {
    return now - last_key < kKeyTimeout ? direction : 0;
  }

  void set(int new_direction) {
    direction = new_direction;
    last_key = Clock::now();
  }

  void clear() { direction = 0; }
};

float parse_positive(const char* text, const char* name, float maximum) {
  size_t parsed = 0;
  const float value = std::stof(text, &parsed);
  if (text[parsed] != '\0' || !std::isfinite(value) || value <= 0 || value > maximum) {
    throw std::runtime_error(std::string(name) + " must be in (0, " +
                             std::to_string(maximum) + "]");
  }
  return value;
}

}  // namespace

int main(int argc, char** argv) {
  static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
                "protocol requires IEEE 754 float32");
  if (argc < 2 || argc > 4) {
    std::cerr << "Usage: keyboard_control SERIAL_DEVICE [speed_mps] [yaw_rate_deg_s]\n";
    return 2;
  }

  try {
    const float speed = argc >= 3 ? parse_positive(argv[2], "speed", 2.54f) : 0.8f;
    const float yaw_rate = argc >= 4 ? parse_positive(argv[3], "yaw rate", 360.0f) : 60.0f;
    const int8_t speed_raw = static_cast<int8_t>(std::lround(speed * 50.0f));
    SerialPort serial(argv[1]);
    Terminal terminal;
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    std::cout << "W/S: forward/back, A/D: left/right, Q/E: yaw left/right, "
                 "Space: stop, Ctrl+C: exit\n"
              << "Waiting for a valid gimbal attitude frame...\n";
    std::vector<uint8_t> receive_buffer;
    Attitude attitude;
    float yaw_target = 0;
    Axis forward, lateral, turn;
    int previous_turn = 0;
    auto next_send = Clock::now();
    auto last_send = next_send;

    while (!stop_requested) {
      pollfd fds[2]{{STDIN_FILENO, POLLIN, 0}, {serial.fd(), POLLIN, 0}};
      const int ready = poll(fds, 2, 10);
      if (ready < 0 && errno != EINTR) throw std::runtime_error("poll failed");
      if (fds[1].revents & (POLLERR | POLLHUP | POLLNVAL)) {
        throw std::runtime_error("serial connection lost");
      }
      if (fds[1].revents & POLLIN) {
        receive_attitude(serial.fd(), receive_buffer, attitude, yaw_target);
      }
      if (fds[0].revents & POLLIN) {
        char keys[32];
        const ssize_t count = read(STDIN_FILENO, keys, sizeof(keys));
        for (ssize_t i = 0; i < count; ++i) {
          switch (keys[i]) {
            case 'w': case 'W': forward.set(1); break;
            case 's': case 'S': forward.set(-1); break;
            case 'a': case 'A': lateral.set(1); break;
            case 'd': case 'D': lateral.set(-1); break;
            case 'q': case 'Q': turn.set(1); break;
            case 'e': case 'E': turn.set(-1); break;
            case ' ': forward.clear(); lateral.clear(); turn.clear(); break;
            default: break;
          }
        }
      }

      const auto now = Clock::now();
      if (now < next_send) continue;
      next_send = now + kSendPeriod;
      if (!attitude.valid) continue;

      const bool fresh = now - attitude.received < kFeedbackTimeout;
      const float dt = std::min(std::chrono::duration<float>(now - last_send).count(),
                                std::chrono::duration<float>(kSendPeriod).count());
      last_send = now;
      const int turn_direction = fresh ? turn.value(now) : 0;
      if (turn_direction != 0) {
        yaw_target += turn_direction * yaw_rate * dt;
      } else if (previous_turn != 0 && fresh) {
        yaw_target = attitude.yaw;
      }
      previous_turn = turn_direction;

      std::array<uint8_t, 13> frame{};
      frame[0] = 0x21;
      frame[1] = 0x00;
      frame[2] = static_cast<uint8_t>(fresh ? forward.value(now) * speed_raw : 0);
      frame[3] = static_cast<uint8_t>(fresh ? lateral.value(now) * speed_raw : 0);
      write_float_le(frame.data() + 4, yaw_target);
      write_float_le(frame.data() + 8, attitude.pitch);
      frame[12] = 0;
      serial.send(frame);
    }

    if (attitude.valid) {
      std::array<uint8_t, 13> stop{};
      stop[0] = 0x21;
      const bool fresh = Clock::now() - attitude.received < kFeedbackTimeout;
      write_float_le(stop.data() + 4, fresh ? attitude.yaw : yaw_target);
      write_float_le(stop.data() + 8, attitude.pitch);
      serial.send(stop);
    }
  } catch (const std::exception& error) {
    std::cerr << "keyboard_control: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
