#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <regex>
#include <string>
#include <vector>

#include <Eigen/Geometry>
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
  using Transform = Eigen::Isometry3d;

  GroundZCalibration() : Node("use_ground_cali") {
    topics_[0] = declare_parameter<std::string>(
        "lidar1_topic", "livox/lidar_192_168_1_5");
    topics_[1] = declare_parameter<std::string>(
        "lidar2_topic", "livox/lidar_192_168_1_3");
    output_file_ = declare_parameter<std::string>(
        "output_file", "ground_z_calibration.yaml");
    lidar3_calibration_file_ = declare_parameter<std::string>(
        "lidar3_calibration_file", "slam/config/gimbal_lidar_3.yaml");
    lidar5_calibration_file_ = declare_parameter<std::string>(
        "lidar5_calibration_file", "slam/config/gimbal_lidar_5.yaml");
    min_points_ = declare_parameter<int>("min_points", 80);
    min_samples_ = declare_parameter<int>("min_samples", 20);
    max_samples_ = declare_parameter<int>("max_samples", 100);
    ransac_distance_ = declare_parameter<double>("ransac_distance", 0.04);
    min_ground_normal_z_ = declare_parameter<double>("min_ground_normal_z", 0.85);
    min_range_ = declare_parameter<double>("min_range", 0.5);
    max_range_ = declare_parameter<double>("max_range", 30.0);
    z_min_ = declare_parameter<double>("z_min", -5.0);
    z_max_ = declare_parameter<double>("z_max", 0.0);
    ground_z_margin_ = declare_parameter<double>("ground_z_margin", 0.02);
    ground_must_be_negative_ = declare_parameter<bool>("ground_must_be_negative", true);
    sync_tolerance_ = declare_parameter<double>("sync_tolerance", 0.10);

    lidar3_transform_loaded_ = loadLidarToGimbal(
        lidar3_calibration_file_, lidar3_to_gimbal_);
    lidar5_transform_loaded_ = loadLidarToGimbal(
        lidar5_calibration_file_, lidar5_to_gimbal_);
    if (!lidar3_transform_loaded_ || !lidar5_transform_loaded_) {
      RCLCPP_WARN(
          get_logger(),
          "existing gimbal calibration YAML not fully loaded; ground heights "
          "will still be saved, but transform matrices will be omitted");
    }

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

  static std::vector<double> numbersInLine(const std::string &line) {
    static const std::regex number_pattern(
        R"([-+]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][-+]?[0-9]+)?)");
    std::vector<double> values;
    for (std::sregex_iterator it(line.begin(), line.end(), number_pattern), end;
         it != end; ++it) {
      values.push_back(std::stod(it->str()));
    }
    return values;
  }

  static bool loadLidarToGimbal(
      const std::string &file_name, Transform &transform) {
    std::ifstream input(file_name);
    if (!input) {
      return false;
    }
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Zero();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    int section = 0;
    std::vector<double> rotation_values;
    std::vector<double> translation_values;
    std::string line;
    while (std::getline(input, line)) {
      if (line.find("lidar_to_gimbal_rotation:") != std::string::npos) {
        section = 1;
        rotation_values.clear();
        continue;
      }
      if (line.find("lidar_to_gimbal_translation:") != std::string::npos) {
        section = 2;
        translation_values.clear();
        continue;
      }
      const std::vector<double> values = numbersInLine(line);
      if (section == 1 && rotation_values.size() < 9) {
        rotation_values.insert(rotation_values.end(), values.begin(), values.end());
      } else if (section == 2 && translation_values.size() < 3) {
        translation_values.insert(translation_values.end(), values.begin(), values.end());
      }
    }
    if (rotation_values.size() < 9 || translation_values.size() < 3) {
      return false;
    }
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        rotation(row, col) =
            rotation_values[static_cast<std::size_t>(row * 3 + col)];
      }
      translation(row) = translation_values[static_cast<std::size_t>(row)];
    }
    if (!rotation.allFinite() || !translation.allFinite() ||
        std::abs(rotation.determinant()) < 1e-6) {
      return false;
    }
    transform = Transform::Identity();
    transform.linear() = rotation;
    transform.translation() = translation;
    return true;
  }

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
          p.z >= z_min_ && p.z <= z_max_ &&
          (!ground_must_be_negative_ || p.z < -ground_z_margin_)) {
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
    if (ground_must_be_negative_ && result.height >= -ground_z_margin_) {
      return {};
    }
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
    output << std::setprecision(12)
           << "# Ground based z calibration (meters)\n"
           << "lidar1_ground_height: " << latest_ground_[0] << "\n"
           << "lidar2_ground_height: " << latest_ground_[1] << "\n"
           << "# lidar1=lidar5, lidar2=lidar3; ground is expected on negative lidar Z\n"
           << "lidar5_ground_height: " << latest_ground_[0] << "\n"
           << "lidar3_ground_height: " << latest_ground_[1] << "\n"
           << "gimbal_height_above_ground: " << -latest_ground_[1] << "\n"
           << "z_offset_lidar3_to_lidar5: " << z_offset_ << "\n"
           << "z_translation_lidar5_to_lidar3: "
           << (latest_ground_[1] - latest_ground_[0]) << "\n";
    if (lidar3_transform_loaded_ && lidar5_transform_loaded_) {
      writeGroundAlignedTransforms(output);
    }
    saved_ = true;
    RCLCPP_INFO(get_logger(), "z calibration saved: lidar3 += %.6f m to lidar5 -> %s",
                z_offset_, output_file_.c_str());
    if (lidar3_transform_loaded_ && lidar5_transform_loaded_) {
      RCLCPP_INFO(
          get_logger(),
          "ground-aligned transforms saved: T_gimbal_lidar3, "
          "T_gimbal_lidar5, T_lidar3_lidar5, T_lidar5_lidar3");
    }
  }

  static void writeMatrix(
      std::ofstream &output, const char *name, const Transform &transform) {
    output << name << ":\n";
    for (int row = 0; row < 4; ++row) {
      output << "  [";
      for (int col = 0; col < 4; ++col) {
        output << transform.matrix()(row, col)
               << (col == 3 ? "]\n" : ", ");
      }
    }
  }

  void writeGroundAlignedTransforms(std::ofstream &output) const {
    // The gimbal Z origin is defined at lidar3 height. The measured ground
    // heights then determine the lidar5-to-lidar3 vertical translation.
    Transform lidar3_to_gimbal = lidar3_to_gimbal_;
    Transform lidar5_to_gimbal = lidar5_to_gimbal_;
    lidar3_to_gimbal.translation().z() = 0.0;
    lidar5_to_gimbal.translation().z() =
        latest_ground_[1] - latest_ground_[0];
    const Transform lidar5_to_lidar3 =
        lidar3_to_gimbal.inverse() * lidar5_to_gimbal;
    const Transform lidar3_to_lidar5 =
        lidar5_to_gimbal.inverse() * lidar3_to_gimbal;

    output << std::setprecision(12)
           << "# Compatibility aliases for pcl_publish: lidar1=lidar5, lidar2=lidar3.\n";
    writeMatrix(output, "T_gimbal_lidar1", lidar5_to_gimbal);
    writeMatrix(output, "T_gimbal_lidar2", lidar3_to_gimbal);
    writeMatrix(output, "T_lidar1_lidar2", lidar3_to_lidar5);
    output << "# Explicit lidar-number names and both relative directions.\n"
           << "# Matrices map a point from the frame in the name suffix "
              "to the frame in the name prefix.\n"
           << "# p_gimbal = T_gimbal_lidar3 * p_lidar3\n"
           << "# p_gimbal = T_gimbal_lidar5 * p_lidar5\n"
           << "# p_lidar3 = T_lidar3_lidar5 * p_lidar5\n"
           << "# p_lidar5 = T_lidar5_lidar3 * p_lidar3\n";
    writeMatrix(output, "T_gimbal_lidar3", lidar3_to_gimbal);
    writeMatrix(output, "T_gimbal_lidar5", lidar5_to_gimbal);
    writeMatrix(output, "T_lidar3_lidar5", lidar5_to_lidar3);
    writeMatrix(output, "T_lidar5_lidar3", lidar3_to_lidar5);
  }

  void report() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (latest_ground_[0] == latest_ground_[0] && latest_ground_[1] == latest_ground_[1]) {
      RCLCPP_INFO(get_logger(), "ground heights: lidar5=%.4f m (%zu), lidar3=%.4f m (%zu), samples=%zu, lidar3_to_lidar5_z_offset=%.4f m%s",
                  latest_ground_[0], latest_inliers_[0], latest_ground_[1], latest_inliers_[1],
                  ground_differences_.size(), z_offset_, calibrated_ ? " [calibrated]" : "");
    } else {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                           "waiting for valid ground planes from both lidars");
    }
  }

  std::string topics_[2];
  std::string output_file_;
  std::string lidar3_calibration_file_;
  std::string lidar5_calibration_file_;
  int min_points_ = 80;
  int min_samples_ = 20;
  int max_samples_ = 100;
  double ransac_distance_ = 0.04;
  double min_ground_normal_z_ = 0.85;
  double min_range_ = 0.5;
  double max_range_ = 30.0;
  double z_min_ = -5.0;
  double z_max_ = 0.0;
  double ground_z_margin_ = 0.02;
  double sync_tolerance_ = 0.10;
  bool ground_must_be_negative_ = true;
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
  bool lidar3_transform_loaded_ = false;
  bool lidar5_transform_loaded_ = false;
  Transform lidar3_to_gimbal_ = Transform::Identity();
  Transform lidar5_to_gimbal_ = Transform::Identity();
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
