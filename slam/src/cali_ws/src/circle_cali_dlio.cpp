#include <cmath>
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
    sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic_, rclcpp::SensorDataQoS(),
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {
          std::lock_guard<std::mutex> l(mutex_);
          Eigen::Quaterniond q(m->pose.orientation.w, m->pose.orientation.x,
                               m->pose.orientation.y, m->pose.orientation.z);
          if (q.norm() < 1e-9) return;
          q.normalize();
          poses_.push_back({q, Eigen::Vector3d(m->pose.position.x,
                                               m->pose.position.y,
                                               m->pose.position.z)});
        });
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { report(); });
  }
 private:
  void report() {
    std::lock_guard<std::mutex> l(mutex_);
    if (poses_.size() < static_cast<size_t>(min_samples_)) return;
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();
    for (const auto &p : poses_) {
      Eigen::Matrix3d A = Eigen::Matrix3d::Identity() - p.first.toRotationMatrix();
      H += A.transpose() * A;
      b += A.transpose() * p.second;
    }
    Eigen::Vector3d c = H.ldlt().solve(b);
    if (!c.allFinite()) return;
    double e = 0.0;
    for (const auto &p : poses_) {
      Eigen::Vector3d d = p.second - (c - p.first.toRotationMatrix() * c);
      e += d.squaredNorm();
    }
    const double rms = std::sqrt(e / poses_.size());
    std::ofstream out(output_file_);
    out << std::fixed << std::setprecision(9)
        << "circle_center: [" << c.x() << ", " << c.y() << ", " << c.z() << "]\n"
        << "circle_residual: " << rms << "\n"
        << "samples: " << poses_.size() << "\n";
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                         "samples=%zu center=(%.4f %.4f %.4f) residual=%.4f m",
                         poses_.size(), c.x(), c.y(), c.z(), rms);
  }
  std::string pose_topic_, output_file_;
  int min_samples_;
  std::vector<std::pair<Eigen::Quaterniond, Eigen::Vector3d>> poses_;
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
