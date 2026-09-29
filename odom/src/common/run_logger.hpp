#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace odom_logging {

// A small asynchronous file logger for high-rate IMU and LiDAR callbacks.
class RunLogger final {
 public:
  RunLogger(std::string component, std::string directory = "runlog")
      : component_(std::move(component)) {
    if (!directory.empty() && directory.front() == '@') directory.erase(0, 1);
    if (directory.empty()) directory = "runlog";

    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_time{};
#ifdef _WIN32
    localtime_s(&local_time, &now_time);
#else
    localtime_r(&now_time, &local_time);
#endif
    std::ostringstream folder_name;
    folder_name << std::put_time(&local_time, "%Y%m%d_%H%M%S");

    directory_ = std::filesystem::path(directory) / folder_name.str();
    std::error_code error;
    std::filesystem::create_directories(directory_, error);
    if (error) {
      throw std::runtime_error("cannot create odom runlog directory " +
                               directory_.string() + ": " + error.message());
    }
    file_path_ = directory_ / (component_ + ".log");
    file_.open(file_path_, std::ios::out | std::ios::app);
    if (!file_) {
      throw std::runtime_error("cannot open odom runlog file " +
                               file_path_.string());
    }
    worker_ = std::thread(&RunLogger::run, this);
    log("LOGGER_START", "file=" + file_path_.string());
  }

  ~RunLogger() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    condition_.notify_one();
    if (worker_.joinable()) worker_.join();
    file_.flush();
    file_.close();
  }

  RunLogger(const RunLogger &) = delete;
  RunLogger &operator=(const RunLogger &) = delete;

  void log(const std::string &event, const std::string &payload = {}) {
    const auto now = std::chrono::system_clock::now();
    const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch());
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_time{};
#ifdef _WIN32
    localtime_s(&local_time, &now_time);
#else
    localtime_r(&now_time, &local_time);
#endif
    const auto millisecond = millis.count() % 1000;
    std::ostringstream line;
    line << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S") << '.'
         << std::setfill('0') << std::setw(3) << millisecond
         << " [" << component_ << "] " << event;
    if (!payload.empty()) line << ' ' << payload;
    line << '\n';

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_) return;
      // Keep the sensor callback bounded if the storage device stalls.
      if (pending_.size() >= maximum_pending_lines_) pending_.pop_front();
      pending_.push_back(line.str());
    }
    condition_.notify_one();
  }

  const std::filesystem::path &path() const { return file_path_; }

 private:
  void run() {
    for (;;) {
      std::deque<std::string> batch;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] { return stopping_ || !pending_.empty(); });
        batch.swap(pending_);
        if (stopping_ && batch.empty()) break;
      }
      for (const auto &line : batch) file_ << line;
      file_.flush();
    }
  }

  static constexpr std::size_t maximum_pending_lines_ = 20000;
  std::string component_;
  std::filesystem::path directory_;
  std::filesystem::path file_path_;
  std::ofstream file_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<std::string> pending_;
  bool stopping_ = false;
  std::thread worker_;
};

}  // namespace odom_logging
