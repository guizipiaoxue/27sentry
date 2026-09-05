#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <cstdlib>

#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

class PclPublish final : public rclcpp::Node {
 public:
  using Msg = livox_ros_driver2::msg::CustomMsg;
  using Point = pcl::PointXYZ;
  using Cloud = pcl::PointCloud<Point>;

  PclPublish() : Node("pcl_publish") {
    const std::string yaml_file = declare_parameter<std::string>(
        "calibration_file", "slam/config/lidar_calibration.yaml");
    const std::string lidar3_topic = declare_parameter<std::string>(
        "lidar3_topic", "livox/lidar_192_168_1_3");
    const std::string lidar5_topic = declare_parameter<std::string>(
        "lidar5_topic", "livox/lidar_192_168_1_5");
    const int queue_size = declare_parameter<int>("queue_size", 10);

    if (!loadCalibration(yaml_file)) {
      throw std::runtime_error("failed to load calibration file: " + yaml_file);
    }

    auto qos = rclcpp::SensorDataQoS();
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/test/pcl", queue_size);
    sub_lidar3_ = create_subscription<Msg>(
        lidar3_topic, qos,
        [this](Msg::ConstSharedPtr msg) { cloudCallback(1, msg); });
    sub_lidar5_ = create_subscription<Msg>(
        lidar5_topic, qos,
        [this](Msg::ConstSharedPtr msg) { cloudCallback(0, msg); });

    RCLCPP_INFO(get_logger(), "lidar3 topic: %s -> T_gimbal_lidar2", lidar3_topic.c_str());
    RCLCPP_INFO(get_logger(), "lidar5 topic: %s -> T_gimbal_lidar1", lidar5_topic.c_str());
    RCLCPP_INFO(get_logger(), "publishing fused cloud on /test/pcl, frame=gimbal");
  }

 private:
  static bool parseMatrixRow(const std::string &line, Eigen::Vector4d &row) {
    const std::size_t begin = line.find('[');
    const std::size_t end = line.find(']', begin);
    if (begin == std::string::npos || end == std::string::npos) {
      return false;
    }
    std::string values = line.substr(begin + 1, end - begin - 1);
    for (char &ch : values) {
      if (ch == ',') {
        ch = ' ';
      }
    }
    std::istringstream stream(values);
    return static_cast<bool>(stream >> row[0] >> row[1] >> row[2] >> row[3]);
  }

  bool loadCalibration(const std::string &file_name) {
    std::string expanded_file_name = file_name;
    if (expanded_file_name.rfind("~/", 0) == 0) {
      const char *home = std::getenv("HOME");
      if (home != nullptr) {
        expanded_file_name = std::string(home) + expanded_file_name.substr(1);
      }
    }

    std::ifstream input(expanded_file_name);
    if (!input) {
      RCLCPP_ERROR(
          get_logger(), "cannot open calibration file: %s",
          expanded_file_name.c_str());
      return false;
    }

    Eigen::Matrix4d *current = nullptr;
    bool loaded[2] = {false, false};
    int row_index = 0;
    std::string line;
    while (std::getline(input, line)) {
      if (line.find("T_gimbal_lidar1:") != std::string::npos) {
        current = &calibration_[0];
        row_index = 0;
        loaded[0] = false;
        continue;
      }
      if (line.find("T_gimbal_lidar2:") != std::string::npos) {
        loaded[0] = (row_index == 4);
        current = &calibration_[1];
        row_index = 0;
        loaded[1] = false;
        continue;
      }
      if (current == nullptr || row_index >= 4) {
        continue;
      }
      Eigen::Vector4d row;
      if (parseMatrixRow(line, row)) {
        current->row(row_index++) = row.transpose();
      }
    }

    loaded[1] = (row_index == 4);
    if (!loaded[0] || !loaded[1] || calibration_[0](3, 3) == 0.0 ||
        calibration_[1](3, 3) == 0.0) {
      RCLCPP_ERROR(
          get_logger(), "invalid calibration matrices in %s",
          expanded_file_name.c_str());
      return false;
    }
    transform_[0] = calibration_[0].cast<float>();
    transform_[1] = calibration_[1].cast<float>();
    RCLCPP_INFO(
        get_logger(),
        "loaded T_gimbal_lidar1 and T_gimbal_lidar2 from %s",
        expanded_file_name.c_str());
    return true;
  }

  static Cloud::Ptr toCloud(const Msg &msg) {
    auto cloud = std::make_shared<Cloud>();
    cloud->reserve(msg.points.size());
    for (const auto &point : msg.points) {
      if (std::isfinite(point.x) && std::isfinite(point.y) &&
          std::isfinite(point.z)) {
        cloud->push_back(Point{point.x, point.y, point.z});
      }
    }
    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
  }

  void cloudCallback(int lidar_index, Msg::ConstSharedPtr msg) {
    Cloud transformed;
    pcl::transformPointCloud(
        *toCloud(*msg), transformed, transform_[lidar_index]);

    std::lock_guard<std::mutex> lock(mutex_);
    latest_[lidar_index] = std::make_shared<Cloud>(std::move(transformed));
    latest_stamp_[lidar_index] = rclcpp::Time(msg->header.stamp);
    publishFused();
  }

  void publishFused() {
    if (!latest_[0] || !latest_[1] || latest_[0]->empty() || latest_[1]->empty()) {
      return;
    }
    auto fused = std::make_shared<Cloud>();
    *fused = *latest_[0];
    *fused += *latest_[1];

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*fused, output);
    output.header.frame_id = "gimbal";
    output.header.stamp = std::max(latest_stamp_[0], latest_stamp_[1]);
    pub_->publish(output);
  }

  Eigen::Matrix4d calibration_[2] = {
      Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity()};
  Eigen::Matrix4f transform_[2] = {
      Eigen::Matrix4f::Identity(), Eigen::Matrix4f::Identity()};
  Cloud::Ptr latest_[2];
  rclcpp::Time latest_stamp_[2]{rclcpp::Time(0, 0, RCL_ROS_TIME),
                                rclcpp::Time(0, 0, RCL_ROS_TIME)};
  std::mutex mutex_;
  rclcpp::Subscription<Msg>::SharedPtr sub_lidar3_;
  rclcpp::Subscription<Msg>::SharedPtr sub_lidar5_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PclPublish>());
  rclcpp::shutdown();
  return 0;
}
