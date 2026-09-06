#include <cmath>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class CircleCaliDlio final : public rclcpp::Node {
 public:
  CircleCaliDlio() : Node("circle_cali_dlio") {
    pose_topic_ = declare_parameter<std::string>("pose_topic", "pose");
    output_file_ = declare_parameter<std::string>("output_file", "gimbal_lidar.yaml");
    min_samples_ = declare_parameter<int>("min_samples", 100);
    max_position_jump_ = declare_parameter<double>("max_position_jump", 0.50);
    max_angular_jump_ = declare_parameter<double>("max_angular_jump", 0.35);
    max_linear_speed_ = declare_parameter<double>("max_linear_speed", 5.0);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed", 6.0);
    fit_window_ = declare_parameter<int>("fit_window", 2000);
    residual_threshold_ = declare_parameter<double>("residual_threshold", 0.05);
    sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic_, rclcpp::SensorDataQoS(),
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {
          std::lock_guard<std::mutex> l(mutex_);
          Eigen::Quaterniond q(m->pose.orientation.w, m->pose.orientation.x,
                               m->pose.orientation.y, m->pose.orientation.z);
          if (q.norm() < 1e-9) return;
          q.normalize();
          Sample s{m->header.stamp.sec + 1e-9 * m->header.stamp.nanosec, q,
                   Eigen::Vector3d(m->pose.position.x, m->pose.position.y,
                                   m->pose.position.z)};
          // Motion is continuous: reject time reversals and implausible jumps
          // before they can contaminate the fit.
          if (!poses_.empty()) {
            const Sample &last = poses_.back();
            const double dt = s.t - last.t;
            if (dt <= 0.0) return;
            const double dp = (s.p - last.p).norm();
            const double da = Eigen::AngleAxisd(last.q.inverse() * s.q).angle();
            const bool normal_interval = dt < 0.2;
            if (dp > max_position_jump_ || da > max_angular_jump_ ||
                (normal_interval && (dp / dt > max_linear_speed_ ||
                                     da / dt > max_angular_speed_))) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                                   "discard discontinuous pose: dp=%.3f da=%.3f dt=%.3f",
                                   dp, da, dt);
              return;
            }
          }
          poses_.push_back(s);
        });
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { report(); });
  }
 private:
  void report() {
    std::lock_guard<std::mutex> l(mutex_);
    const size_t begin = fit_window_ > 0 && poses_.size() > static_cast<size_t>(fit_window_)
                             ? poses_.size() - static_cast<size_t>(fit_window_) : 0;
    const size_t count = poses_.size() - begin;
    if (count < static_cast<size_t>(min_samples_)) return;
    std::vector<bool> inlier(count, true);
    Eigen::Vector3d c = Eigen::Vector3d::Zero();
    // Iterative sigma clipping limits the influence of isolated bad samples.
    for (int iter = 0; iter < 5; ++iter) {
      Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
      Eigen::Vector3d b = Eigen::Vector3d::Zero();
      size_t n = 0;
      for (size_t j = 0; j < count; ++j) if (inlier[j]) {
        const size_t i = begin + j;
        const auto R = poses_[i].q.toRotationMatrix();
        const Eigen::Matrix3d A = Eigen::Matrix3d::Identity() - R;
        H += A.transpose() * A;
        b += A.transpose() * poses_[i].p;
        ++n;
      }
      if (n < static_cast<size_t>(min_samples_)) return;
      c = H.ldlt().solve(b);
      if (!c.allFinite()) return;
      std::vector<double> residuals;
      residuals.reserve(n);
      for (size_t j = 0; j < count; ++j) if (inlier[j]) {
        const size_t i = begin + j;
        const auto R = poses_[i].q.toRotationMatrix();
        residuals.push_back((poses_[i].p - (c - R * c)).norm());
      }
      std::nth_element(residuals.begin(), residuals.begin() + residuals.size()/2,
                       residuals.end());
      const double median = residuals[residuals.size()/2];
      const double cutoff = std::max(residual_threshold_, 3.0 * median);
      bool changed = false;
      for (size_t j = 0; j < count; ++j) if (inlier[j]) {
        const size_t i = begin + j;
        const auto R = poses_[i].q.toRotationMatrix();
        if ((poses_[i].p - (c - R * c)).norm() > cutoff) { inlier[j] = false; changed = true; }
      }
      if (!changed) break;
    }
    size_t used = 0;
    for (bool v : inlier) used += v;
    if (used < static_cast<size_t>(min_samples_)) return;
    double e = 0.0;
    for (size_t j = 0; j < count; ++j) if (inlier[j]) {
      const size_t i = begin + j;
      Eigen::Vector3d d = poses_[i].p - (c - poses_[i].q.toRotationMatrix() * c);
      e += d.squaredNorm();
    }
    const double rms = std::sqrt(e / used);
    std::ofstream out(output_file_);
    out << std::fixed << std::setprecision(9)
        << "circle_center: [" << c.x() << ", " << c.y() << ", " << c.z() << "]\n"
        << "circle_residual: " << rms << "\n"
        << "samples: " << used << "\n"
        << "rejected: " << (count - used) << "\n";
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                         "samples=%zu center=(%.4f %.4f %.4f) residual=%.4f m",
                         used, c.x(), c.y(), c.z(), rms);
  }
  std::string pose_topic_, output_file_;
  int min_samples_;
  struct Sample { double t; Eigen::Quaterniond q; Eigen::Vector3d p; };
  std::vector<Sample> poses_;
  double max_position_jump_, max_angular_jump_, max_linear_speed_, max_angular_speed_, residual_threshold_;
  int fit_window_;
  std::mutex mutex_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CircleCaliDlio>());
  rclcpp::shutdown();
  return 0;
}
