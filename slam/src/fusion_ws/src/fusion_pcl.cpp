// Use the calibrated lidar-to-gimbal transforms to fuse lidar 5 and lidar 3.
// Cloud pairs are published at the lidar rate (about 10 Hz), while synchronized
// IMU pairs are rotated into the gimbal frame, averaged, and published at the
// IMU rate (about 200 Hz). ROS header stamps always use sec + nanosec; the
// different device packet units must not be applied to header.stamp.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Core>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <yaml-cpp/yaml.h>

class FusionPcl final : public rclcpp::Node {
 public:
  using CustomMsg = livox_ros_driver2::msg::CustomMsg;
  using Imu = sensor_msgs::msg::Imu;
  using Cloud = pcl::PointCloud<pcl::PointXYZI>;

  FusionPcl() : Node("fusion_pcl") {
    const std::string package_share =
        ament_index_cpp::get_package_share_directory("fusion_ws");

    cloud_topics_[0] = declare_parameter<std::string>(
        "lidar5_topic", "livox/lidar_192_168_1_5");
    cloud_topics_[1] = declare_parameter<std::string>(
        "lidar3_topic", "livox/lidar_192_168_1_3");
    imu_topics_[0] = declare_parameter<std::string>(
        "imu5_topic", "livox/imu_192_168_1_5");
    imu_topics_[1] = declare_parameter<std::string>(
        "imu3_topic", "livox/imu_192_168_1_3");
    cloud_output_topic_ = declare_parameter<std::string>(
        "cloud_output_topic", "/gimbal/cloud_fused");
    imu_output_topic_ = declare_parameter<std::string>(
        "imu_output_topic", "/gimbal/imu_fused");
    frame_id_ = declare_parameter<std::string>("frame_id", "gimbal");
    cloud_sync_tolerance_ = declare_parameter<double>(
        "cloud_sync_tolerance", 0.03);
    imu_sync_tolerance_ = declare_parameter<double>(
        "imu_sync_tolerance", 0.004);
    max_queue_size_ = static_cast<std::size_t>(std::max<std::int64_t>(
        2, declare_parameter<std::int64_t>("max_queue_size", 100)));

    const std::string lidar5_calibration = declare_parameter<std::string>(
        "lidar5_calibration",
        package_share + "/config/gimbal_lidar_5.yaml");
    const std::string lidar3_calibration = declare_parameter<std::string>(
        "lidar3_calibration",
        package_share + "/config/gimbal_lidar_3.yaml");
    transforms_[0] = loadTransform(lidar5_calibration);
    transforms_[1] = loadTransform(lidar3_calibration);

    const auto qos = rclcpp::SensorDataQoS();
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        cloud_output_topic_, qos);
    imu_pub_ = create_publisher<Imu>(imu_output_topic_, qos);

    for (std::size_t i = 0; i < 2; ++i) {
      cloud_subs_[i] = create_subscription<CustomMsg>(
          cloud_topics_[i], qos,
          [this, i](CustomMsg::ConstSharedPtr msg) {
            cloudCallback(i, std::move(msg));
          });
      imu_subs_[i] = create_subscription<Imu>(
          imu_topics_[i], qos,
          [this, i](Imu::ConstSharedPtr msg) {
            imuCallback(i, std::move(msg));
          });
    }

    RCLCPP_INFO(
        get_logger(),
        "fusing lidar5/lidar3 -> %s and imu5/imu3 -> %s (frame: %s)",
        cloud_output_topic_.c_str(), imu_output_topic_.c_str(),
        frame_id_.c_str());
  }

 private:
  template <typename MessageT>
  struct TimedMessage {
    rclcpp::Time stamp;
    std::shared_ptr<const MessageT> message;
  };

  static Eigen::Matrix4d loadTransform(const std::string &file_name) {
    const YAML::Node config = YAML::LoadFile(file_name);
    const YAML::Node rotation = config["lidar_to_gimbal_rotation"];
    const YAML::Node translation = config["lidar_to_gimbal_translation"];
    if (!rotation || rotation.size() != 3 || !translation ||
        translation.size() != 3) {
      throw std::runtime_error(
          "invalid lidar-to-gimbal calibration: " + file_name);
    }

    Eigen::Matrix3d rotation_matrix;
    Eigen::Vector3d translation_vector;
    for (std::size_t row = 0; row < 3; ++row) {
      if (!rotation[row].IsSequence() || rotation[row].size() != 3) {
        throw std::runtime_error("invalid rotation matrix: " + file_name);
      }
      for (std::size_t col = 0; col < 3; ++col) {
        rotation_matrix(
            static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(col)) =
            rotation[row][col].as<double>();
      }
      translation_vector(static_cast<Eigen::Index>(row)) =
          translation[row].as<double>();
    }

    if (!rotation_matrix.allFinite() || !translation_vector.allFinite() ||
        std::abs(rotation_matrix.determinant() - 1.0) > 1e-3 ||
        !(rotation_matrix.transpose() * rotation_matrix).isIdentity(1e-3)) {
      throw std::runtime_error("non-rigid calibration transform: " + file_name);
    }

    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = rotation_matrix;
    transform.block<3, 1>(0, 3) = translation_vector;
    return transform;
  }

  static Cloud::Ptr transformCloud(
      const CustomMsg &message, const Eigen::Matrix4d &transform) {
    Cloud::Ptr input(new Cloud);
    input->reserve(message.points.size());
    for (const auto &point : message.points) {
      const double range_squared =
          point.x * point.x + point.y * point.y + point.z * point.z;
      if (std::isfinite(point.x) && std::isfinite(point.y) &&
          std::isfinite(point.z) && range_squared > 1e-8) {
        pcl::PointXYZI output;
        output.x = point.x;
        output.y = point.y;
        output.z = point.z;
        output.intensity = static_cast<float>(point.reflectivity);
        input->push_back(output);
      }
    }
    input->width = static_cast<std::uint32_t>(input->size());
    input->height = 1;
    input->is_dense = true;

    Cloud::Ptr output(new Cloud);
    pcl::transformPointCloud(*input, *output, transform.cast<float>());
    return output;
  }

  template <typename MessageT>
  void limitQueue(std::deque<TimedMessage<MessageT>> &queue) {
    while (queue.size() > max_queue_size_) {
      queue.pop_front();
    }
  }

  void cloudCallback(std::size_t index, CustomMsg::ConstSharedPtr message) {
    cloud_queues_[index].push_back(
        {rclcpp::Time(message->header.stamp), std::move(message)});
    limitQueue(cloud_queues_[index]);
    synchronizeClouds();
  }

  void synchronizeClouds() {
    while (!cloud_queues_[0].empty() && !cloud_queues_[1].empty()) {
      const double dt =
          (cloud_queues_[0].front().stamp - cloud_queues_[1].front().stamp)
              .seconds();
      if (std::abs(dt) <= cloud_sync_tolerance_) {
        auto lidar5 = std::move(cloud_queues_[0].front());
        auto lidar3 = std::move(cloud_queues_[1].front());
        cloud_queues_[0].pop_front();
        cloud_queues_[1].pop_front();
        publishCloudPair(lidar5, lidar3);
      } else {
        const std::size_t older = dt < 0.0 ? 0 : 1;
        cloud_queues_[older].pop_front();
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "dropping unsynchronized lidar%zu cloud (delta %.3f s)",
            older == 0 ? 5UL : 3UL, std::abs(dt));
      }
    }
  }

  void publishCloudPair(
      const TimedMessage<CustomMsg> &lidar5,
      const TimedMessage<CustomMsg> &lidar3) {
    Cloud::Ptr fused = transformCloud(*lidar5.message, transforms_[0]);
    const Cloud::Ptr cloud3 = transformCloud(*lidar3.message, transforms_[1]);
    *fused += *cloud3;

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*fused, output);
    output.header.frame_id = frame_id_;
    output.header.stamp =
        lidar5.stamp >= lidar3.stamp ? lidar5.stamp : lidar3.stamp;
    cloud_pub_->publish(output);
  }

  void imuCallback(std::size_t index, Imu::ConstSharedPtr message) {
    imu_queues_[index].push_back(
        {rclcpp::Time(message->header.stamp), std::move(message)});
    limitQueue(imu_queues_[index]);
    synchronizeImus();
  }

  void synchronizeImus() {
    while (!imu_queues_[0].empty() && !imu_queues_[1].empty()) {
      const double dt =
          (imu_queues_[0].front().stamp - imu_queues_[1].front().stamp)
              .seconds();
      if (std::abs(dt) <= imu_sync_tolerance_) {
        auto imu5 = std::move(imu_queues_[0].front());
        auto imu3 = std::move(imu_queues_[1].front());
        imu_queues_[0].pop_front();
        imu_queues_[1].pop_front();
        publishImuPair(imu5, imu3);
      } else {
        const std::size_t older = dt < 0.0 ? 0 : 1;
        imu_queues_[older].pop_front();
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "dropping unsynchronized lidar%zu IMU sample (delta %.4f s)",
            older == 0 ? 5UL : 3UL, std::abs(dt));
      }
    }
  }

  static Eigen::Vector3d angularVelocity(const Imu &imu) {
    return {imu.angular_velocity.x, imu.angular_velocity.y,
            imu.angular_velocity.z};
  }

  static Eigen::Vector3d linearAcceleration(const Imu &imu) {
    return {imu.linear_acceleration.x, imu.linear_acceleration.y,
            imu.linear_acceleration.z};
  }

  void publishImuPair(
      const TimedMessage<Imu> &imu5, const TimedMessage<Imu> &imu3) {
    const Eigen::Vector3d gyro5 =
        transforms_[0].block<3, 3>(0, 0) * angularVelocity(*imu5.message);
    const Eigen::Vector3d gyro3 =
        transforms_[1].block<3, 3>(0, 0) * angularVelocity(*imu3.message);
    const Eigen::Vector3d accel5 =
        transforms_[0].block<3, 3>(0, 0) * linearAcceleration(*imu5.message);
    const Eigen::Vector3d accel3 =
        transforms_[1].block<3, 3>(0, 0) * linearAcceleration(*imu3.message);

    const Eigen::Vector3d gyro = 0.5 * (gyro5 + gyro3);
    const Eigen::Vector3d accel = 0.5 * (accel5 + accel3);
    if (!gyro.allFinite() || !accel.allFinite()) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "dropping fused IMU sample containing non-finite values");
      return;
    }

    Imu output;
    output.header.frame_id = frame_id_;
    output.header.stamp = imu5.stamp >= imu3.stamp ? imu5.stamp : imu3.stamp;
    // Livox IMUs do not provide orientation; -1 explicitly marks it absent.
    output.orientation_covariance[0] = -1.0;
    output.angular_velocity.x = gyro.x();
    output.angular_velocity.y = gyro.y();
    output.angular_velocity.z = gyro.z();
    output.linear_acceleration.x = accel.x();
    output.linear_acceleration.y = accel.y();
    output.linear_acceleration.z = accel.z();
    imu_pub_->publish(output);
  }

  std::array<std::string, 2> cloud_topics_;
  std::array<std::string, 2> imu_topics_;
  std::string cloud_output_topic_;
  std::string imu_output_topic_;
  std::string frame_id_;
  double cloud_sync_tolerance_ = 0.03;
  double imu_sync_tolerance_ = 0.004;
  std::size_t max_queue_size_ = 100;
  std::array<Eigen::Matrix4d, 2> transforms_ = {
      Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity()};

  std::array<std::deque<TimedMessage<CustomMsg>>, 2> cloud_queues_;
  std::array<std::deque<TimedMessage<Imu>>, 2> imu_queues_;
  std::array<rclcpp::Subscription<CustomMsg>::SharedPtr, 2> cloud_subs_;
  std::array<rclcpp::Subscription<Imu>::SharedPtr, 2> imu_subs_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<Imu>::SharedPtr imu_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<FusionPcl>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("fusion_pcl"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
