#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

class GroundZCalibration final : public rclcpp::Node {
 public:
  using Msg = livox_ros_driver2::msg::CustomMsg;
  using Point = pcl::PointXYZ;
  using Cloud = pcl::PointCloud<Point>;

  GroundZCalibration() : Node("use_ground_cali") {
    topics_[0] = declare_parameter<std::string>(
        "lidar1_topic", "livox/lidar_192_168_1_5");
    topics_[1] = declare_parameter<std::string>(
        "lidar2_topic", "livox/lidar_192_168_1_3");
    output_file_ = declare_parameter<std::string>(
        "output_file", "ground_z_calibration.yaml");
    min_points_ = declare_parameter<int>("min_points", 80);
    min_samples_ = declare_parameter<int>("min_samples", 20);
    max_samples_ = declare_parameter<int>("max_samples", 100);
    ransac_distance_ = declare_parameter<double>("ransac_distance", 0.04);
    min_ground_normal_z_ = declare_parameter<double>("min_ground_normal_z", 0.85);
    min_range_ = declare_parameter<double>("min_range", 0.5);
    max_range_ = declare_parameter<double>("max_range", 30.0);
    z_min_ = declare_parameter<double>("z_min", -5.0);
    z_max_ = declare_parameter<double>("z_max", 5.0);
    sync_tolerance_ = declare_parameter<double>("sync_tolerance", 0.10);

    auto qos = rclcpp::SensorDataQoS();
    subscriptions_[0] = create_subscription<Msg>(
        topics_[0], qos, [this](Msg::ConstSharedPtr msg) { cloudCallback(0, msg); });
    subscriptions_[1] = create_subscription<Msg>(
        topics_[1], qos, [this](Msg::ConstSharedPtr msg) { cloudCallback(1, msg); });
    publishers_[0] = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/ground_cali/cloud_lidar1", 10);
    publishers_[1] = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/ground_cali/cloud_lidar2", 10);
    fused_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/ground_cali/cloud_fused", 10);
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { report(); });

    RCLCPP_INFO(get_logger(), "ground calibration topics: %s and %s",
                topics_[0].c_str(), topics_[1].c_str());
  }

 private:
  struct GroundEstimate {
    bool valid = false;
    double height = std::numeric_limits<double>::quiet_NaN();
    std::size_t inliers = 0;
  };

  static Cloud::Ptr toCloud(const Msg &msg) {
    auto cloud = std::make_shared<Cloud>();
    cloud->reserve(msg.points.size());
    for (const auto &p : msg.points) {
      if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
        cloud->push_back(Point{p.x, p.y, p.z});
      }
    }
    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
  }

  GroundEstimate estimateGround(const Cloud::Ptr &cloud) const {
    Cloud::Ptr candidates(new Cloud);
    const double min_range_sq = min_range_ * min_range_;
    const double max_range_sq = max_range_ * max_range_;
    for (const auto &p : cloud->points) {
      const double range_sq = static_cast<double>(p.x) * p.x +
                              static_cast<double>(p.y) * p.y;
      if (range_sq >= min_range_sq && range_sq <= max_range_sq &&
          p.z >= z_min_ && p.z <= z_max_) {
        candidates->push_back(p);
      }
    }
    if (candidates->size() < static_cast<std::size_t>(min_points_)) {
      return {};
    }

    pcl::SACSegmentation<Point> segmentation;
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(ransac_distance_);
    segmentation.setMaxIterations(150);
    segmentation.setInputCloud(candidates);
    pcl::PointIndices inliers;
    pcl::ModelCoefficients coefficients;
    segmentation.segment(inliers, coefficients);
    if (inliers.indices.size() < static_cast<std::size_t>(min_points_) ||
        coefficients.values.size() < 4) {
      return {};
    }
    const double a = coefficients.values[0];
    const double b = coefficients.values[1];
    const double c = coefficients.values[2];
    const double d = coefficients.values[3];
    const double normal_norm = std::sqrt(a * a + b * b + c * c);
    if (normal_norm < 1e-9 || std::abs(c) / normal_norm < min_ground_normal_z_) {
      return {};
    }
    GroundEstimate result;
    result.valid = true;
    result.height = -d / c;
    result.inliers = inliers.indices.size();
    return result;
  }

  static double median(std::vector<double> values) {
    if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
    const auto middle = values.begin() + values.size() / 2;
    std::nth_element(values.begin(), middle, values.end());
    double result = *middle;
    if (values.size() % 2 == 0) {
      const auto lower = std::max_element(values.begin(), middle);
      result = (*lower + result) * 0.5;
    }
    return result;
  }

  void cloudCallback(int index, Msg::ConstSharedPtr msg) {
    const auto cloud = toCloud(*msg);
    if (cloud->size() < static_cast<std::size_t>(min_points_)) return;
    const GroundEstimate ground = estimateGround(cloud);
    std::lock_guard<std::mutex> lock(mutex_);
    latest_cloud_[index] = cloud;
    latest_stamp_[index] = rclcpp::Time(msg->header.stamp);
    if (ground.valid) {
      latest_ground_[index] = ground.height;
      latest_inliers_[index] = ground.inliers;
    }
    if (latest_ground_[0] == latest_ground_[0] && latest_ground_[1] == latest_ground_[1] &&
        std::abs((latest_stamp_[0] - latest_stamp_[1]).seconds()) <= sync_tolerance_) {
      ground_differences_.push_back(latest_ground_[0] - latest_ground_[1]);
      while (ground_differences_.size() > static_cast<std::size_t>(max_samples_)) {
        ground_differences_.pop_front();
      }
      if (ground_differences_.size() >= static_cast<std::size_t>(min_samples_)) {
        std::vector<double> samples(ground_differences_.begin(), ground_differences_.end());
        z_offset_ = median(std::move(samples));
        calibrated_ = std::isfinite(z_offset_);
        if (calibrated_ && !saved_) saveCalibration();
      }
    }
    publishLatest();
  }

  void publishLatest() {
    if (!latest_cloud_[0] || !latest_cloud_[1]) return;
    Cloud aligned[2];
    aligned[0] = *latest_cloud_[0];
    aligned[1] = *latest_cloud_[1];
    if (calibrated_) {
      for (auto &p : aligned[1].points) p.z += static_cast<float>(z_offset_);
    }
    for (int i = 0; i < 2; ++i) {
      sensor_msgs::msg::PointCloud2 output;
      pcl::toROSMsg(aligned[i], output);
      output.header.frame_id = i == 0 ? "lidar1" : "lidar2_ground_aligned";
      output.header.stamp = latest_stamp_[i];
      publishers_[i]->publish(output);
    }
    Cloud fused = aligned[0];
    fused += aligned[1];
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(fused, output);
    output.header.frame_id = "lidar1_ground_aligned";
    output.header.stamp = std::max(latest_stamp_[0], latest_stamp_[1]);
    fused_publisher_->publish(output);
  }

  void saveCalibration() {
    std::ofstream output(output_file_);
    if (!output) {
      RCLCPP_WARN(get_logger(), "cannot write calibration file: %s", output_file_.c_str());
      return;
    }
    output << "# Ground based z calibration (meters)\n"
           << "lidar1_ground_height: " << latest_ground_[0] << "\n"
           << "lidar2_ground_height: " << latest_ground_[1] << "\n"
           << "z_offset_lidar2_to_lidar1: " << z_offset_ << "\n";
    saved_ = true;
    RCLCPP_INFO(get_logger(), "z calibration saved: lidar2 += %.6f m -> %s",
                z_offset_, output_file_.c_str());
  }

  void report() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (latest_ground_[0] == latest_ground_[0] && latest_ground_[1] == latest_ground_[1]) {
      RCLCPP_INFO(get_logger(), "ground heights: lidar1=%.4f m (%zu), lidar2=%.4f m (%zu), samples=%zu, z_offset=%.4f m%s",
                  latest_ground_[0], latest_inliers_[0], latest_ground_[1], latest_inliers_[1],
                  ground_differences_.size(), z_offset_, calibrated_ ? " [calibrated]" : "");
    } else {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                           "waiting for valid ground planes from both lidars");
    }
  }

  std::string topics_[2];
  std::string output_file_;
  int min_points_ = 80;
  int min_samples_ = 20;
  int max_samples_ = 100;
  double ransac_distance_ = 0.04;
  double min_ground_normal_z_ = 0.85;
  double min_range_ = 0.5;
  double max_range_ = 30.0;
  double z_min_ = -5.0;
  double z_max_ = 5.0;
  double sync_tolerance_ = 0.10;
  double latest_ground_[2] = {std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::quiet_NaN()};
  std::size_t latest_inliers_[2] = {0, 0};
  rclcpp::Time latest_stamp_[2]{rclcpp::Time(0, 0, RCL_ROS_TIME),
                                rclcpp::Time(0, 0, RCL_ROS_TIME)};
  Cloud::Ptr latest_cloud_[2];
  std::deque<double> ground_differences_;
  double z_offset_ = 0.0;
  bool calibrated_ = false;
  bool saved_ = false;
  std::mutex mutex_;
  rclcpp::Subscription<Msg>::SharedPtr subscriptions_[2];
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publishers_[2];
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fused_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundZCalibration>());
  rclcpp::shutdown();
  return 0;
}
