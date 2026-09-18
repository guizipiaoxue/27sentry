// ROS 2 wrapper around Point-LIO's point-by-point iterated Kalman filter.
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <functional>
#include <deque>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "plio/point_lio_estimator.hpp"

namespace plio {
namespace {

template <typename T>
T parameter(rclcpp::Node &node, const std::string &name, const T &fallback) {
  return node.declare_parameter<T>(name, fallback);
}

Eigen::Vector3d vectorParameter(
    rclcpp::Node &node, const std::string &name,
    const Eigen::Vector3d &fallback) {
  const auto value = node.declare_parameter<std::vector<double>>(
      name, {fallback.x(), fallback.y(), fallback.z()});
  if (value.size() != 3) {
    throw std::runtime_error(name + " must contain 3 values");
  }
  return {value[0], value[1], value[2]};
}

Eigen::Matrix3d matrixParameter(
    rclcpp::Node &node, const std::string &name,
    const Eigen::Matrix3d &fallback) {
  std::vector<double> defaults(fallback.data(), fallback.data() + 9);
  const auto value = node.declare_parameter<std::vector<double>>(name, defaults);
  if (value.size() != 9) {
    throw std::runtime_error(name + " must contain 9 column-major values");
  }
  using RowMajorMatrix = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;
  return Eigen::Map<const RowMajorMatrix>(value.data());
}

double stampSeconds(const builtin_interfaces::msg::Time &stamp) {
  return static_cast<double>(stamp.sec) +
      static_cast<double>(stamp.nanosec) * 1.0e-9;
}

}  // namespace

class PointLioNode final : public rclcpp::Node {
 public:
  PointLioNode() : Node("point_lio") {
    world_frame_ = parameter<std::string>(*this, "frames.world", "odom");
    body_frame_ = parameter<std::string>(*this, "frames.body", "gimbal");
    publish_tf_ = parameter<bool>(*this, "publish.tf", true);
    publish_path_ = parameter<bool>(*this, "publish.path", true);
    path_capacity_ = static_cast<std::size_t>(std::max<std::int64_t>(
        1, parameter<std::int64_t>(*this, "publish.path_capacity", 10000)));

    Parameters parameters;
    parameters.surface_leaf_size = parameter<double>(*this, "filter_size_surf", 0.30);
    parameters.map_resolution = parameter<double>(*this, "mapping.ivox_grid_resolution", 0.30);
    parameters.blind = parameter<double>(*this, "preprocess.blind", 0.25);
    parameters.detection_range = parameter<double>(*this, "mapping.det_range", 100.0);
    parameters.plane_threshold = parameter<double>(*this, "mapping.plane_thr", 0.10);
    parameters.match_scale = parameter<double>(*this, "mapping.match_s", 81.0);
    parameters.lidar_measurement_covariance = parameter<double>(*this, "mapping.lidar_meas_cov", 0.10);
    parameters.gyro_covariance = parameter<double>(*this, "mapping.gyr_cov_input", 0.10);
    parameters.accel_covariance = parameter<double>(*this, "mapping.acc_cov_input", 0.10);
    parameters.gyro_bias_covariance = parameter<double>(*this, "mapping.b_gyr_cov", 1.0e-4);
    parameters.accel_bias_covariance = parameter<double>(*this, "mapping.b_acc_cov", 1.0e-4);
    parameters.initialization_samples = static_cast<std::size_t>(std::max<std::int64_t>(
        10, parameter<std::int64_t>(*this, "imu.initialization_samples", 100)));
    parameters.initialization_points = static_cast<std::size_t>(std::max<std::int64_t>(
        10, parameter<std::int64_t>(*this, "mapping.initialization_points", 100)));
    parameters.point_filter = static_cast<std::size_t>(std::max<std::int64_t>(
        1, parameter<std::int64_t>(*this, "preprocess.point_filter_num", 2)));
    parameters.nearby_type = parameter<int>(*this, "mapping.ivox_nearby_type", 18);
    parameters.estimate_extrinsics = parameter<bool>(*this, "mapping.extrinsic_est_en", false);
    parameters.gravity = vectorParameter(
        *this, "mapping.gravity", Eigen::Vector3d(0.0, 0.0, -9.80665));
    parameters.lidar_to_imu_translation = vectorParameter(
        *this, "mapping.extrinsic_T", Eigen::Vector3d::Zero());
    parameters.lidar_to_imu_rotation = matrixParameter(
        *this, "mapping.extrinsic_R", Eigen::Matrix3d::Identity());
    estimator_ = std::make_unique<PointLioEstimator>(parameters);

    const auto sensor_qos = rclcpp::SensorDataQoS();
    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu", sensor_qos,
        std::bind(&PointLioNode::imuCallback, this, std::placeholders::_1));
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "pointcloud", sensor_qos,
        std::bind(&PointLioNode::cloudCallback, this, std::placeholders::_1));
    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odom", output_qos);
    path_publisher_ = create_publisher<nav_msgs::msg::Path>("path", output_qos);
    registered_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("registered", output_qos);
    body_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("registered_body", output_qos);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    path_.header.frame_id = world_frame_;
    parameter_callback_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &updated) {
          rcl_interfaces::msg::SetParametersResult response;
          response.successful = true;
          if (std::any_of(updated.begin(), updated.end(),
                          [](const rclcpp::Parameter &candidate) {
                            return candidate.get_name() == "reset" &&
                                   candidate.as_bool();
                          })) {
            std::lock_guard<std::mutex> lock(mutex_);
            estimator_->reset();
            pending_clouds_.clear();
            path_.poses.clear();
            RCLCPP_INFO(get_logger(), "Point-LIO state and map reset");
          }
          return response;
        });
    declare_parameter<bool>("reset", false);
    RCLCPP_INFO(get_logger(), "Point-LIO: fused pointcloud + IMU, frames %s -> %s",
                world_frame_.c_str(), body_frame_.c_str());
  }

 private:
  struct PendingCloud {
    Cloud::Ptr cloud;
    builtin_interfaces::msg::Time stamp;
  };

  struct PublishedResult {
    Result result;
    builtin_interfaces::msg::Time stamp;
  };

  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr message) {
    ImuSample sample;
    sample.stamp = stampSeconds(message->header.stamp);
    sample.acceleration = {message->linear_acceleration.x,
                           message->linear_acceleration.y,
                           message->linear_acceleration.z};
    sample.angular_velocity = {message->angular_velocity.x,
                               message->angular_velocity.y,
                               message->angular_velocity.z};
    std::vector<PublishedResult> ready;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      estimator_->addImu(sample);
      ready = drainPendingLocked();
    }
    for (const auto &item : ready) publish(item.result, item.stamp);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
    pcl::PointCloud<pcl::PointXYZI> input;
    pcl::fromROSMsg(*message, input);

    Cloud::Ptr cloud(new Cloud);
    cloud->resize(input.size());
    cloud->width = input.width;
    cloud->height = input.height;
    cloud->is_dense = input.is_dense;
    for (std::size_t i = 0; i < input.size(); ++i) {
      const auto &source = input.points[i];
      auto &target = cloud->points[i];
      target.x = source.x;
      target.y = source.y;
      target.z = source.z;
      target.intensity = source.intensity;
      target.normal_x = 0.0F;
      target.normal_y = 0.0F;
      target.normal_z = 0.0F;
      target.curvature = 0.0F;
    }

    // fusion_ws publishes seconds in `time`; Point-LIO stores milliseconds in
    // PointXYZINormal::curvature. Missing timing falls back to zero offset.
    const auto field = std::find_if(
        message->fields.begin(), message->fields.end(),
        [](const sensor_msgs::msg::PointField &candidate) { return candidate.name == "time"; });
    const bool valid_time_field =
        field != message->fields.end() &&
        field->datatype == sensor_msgs::msg::PointField::FLOAT32 &&
        field->count == 1 &&
        field->offset + sizeof(float) <= message->point_step;
    const std::size_t expected_points =
        static_cast<std::size_t>(message->width) * message->height;
    if (valid_time_field && input.size() == expected_points) {
      const std::uint8_t *data = message->data.data();
      for (std::size_t i = 0; i < cloud->size(); ++i) {
        const std::size_t row = i / message->width;
        const std::size_t column = i % message->width;
        const std::size_t offset = row * message->row_step +
            column * message->point_step + field->offset;
        float seconds = 0.0F;
        std::memcpy(&seconds, data + offset, sizeof(seconds));
        cloud->points[i].curvature = seconds * 1000.0F;
      }
    } else {
      RCLCPP_WARN_ONCE(get_logger(),
                       "input cloud has no float32 time field; point timing is disabled");
    }

    std::vector<PublishedResult> ready;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pending_clouds_.push_back({cloud, message->header.stamp});
      if (pending_clouds_.size() > maximum_pending_clouds_) {
        pending_clouds_.pop_front();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "dropping oldest cloud while waiting for fused IMU");
      }
      ready = drainPendingLocked();
    }
    if (ready.empty()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                           "waiting for fused IMU initialization and first map");
    }
    for (const auto &item : ready) publish(item.result, item.stamp);
  }

  std::vector<PublishedResult> drainPendingLocked() {
    std::vector<PublishedResult> ready;
    while (!pending_clouds_.empty()) {
      const auto &pending = pending_clouds_.front();
      Result result = estimator_->process(
          pending.cloud, stampSeconds(pending.stamp));
      if (result.waiting_for_imu) break;
      if (result.initialized) ready.push_back({std::move(result), pending.stamp});
      pending_clouds_.pop_front();
    }
    return ready;
  }

  void publish(const Result &result, const builtin_interfaces::msg::Time &stamp) {
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = world_frame_;
    odometry.child_frame_id = body_frame_;
    odometry.pose.pose.position.x = result.position.x();
    odometry.pose.pose.position.y = result.position.y();
    odometry.pose.pose.position.z = result.position.z();
    odometry.pose.pose.orientation.x = result.orientation.x();
    odometry.pose.pose.orientation.y = result.orientation.y();
    odometry.pose.pose.orientation.z = result.orientation.z();
    odometry.pose.pose.orientation.w = result.orientation.w();
    odometry.twist.twist.linear.x = result.velocity.x();
    odometry.twist.twist.linear.y = result.velocity.y();
    odometry.twist.twist.linear.z = result.velocity.z();
    odometry_publisher_->publish(odometry);

    sensor_msgs::msg::PointCloud2 registered;
    pcl::toROSMsg(*result.registered, registered);
    registered.header.stamp = stamp;
    registered.header.frame_id = world_frame_;
    registered_publisher_->publish(registered);
    sensor_msgs::msg::PointCloud2 body;
    pcl::toROSMsg(*result.body, body);
    body.header.stamp = stamp;
    body.header.frame_id = body_frame_;
    body_publisher_->publish(body);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = odometry.header;
    pose.pose = odometry.pose.pose;
    if (publish_path_) {
      path_.header.stamp = stamp;
      path_.poses.push_back(pose);
      if (path_.poses.size() > path_capacity_) path_.poses.erase(path_.poses.begin());
      path_publisher_->publish(path_);
    }
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odometry.header;
      transform.child_frame_id = body_frame_;
      transform.transform.translation.x = result.position.x();
      transform.transform.translation.y = result.position.y();
      transform.transform.translation.z = result.position.z();
      transform.transform.rotation = odometry.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }
  }

  std::mutex mutex_;
  std::unique_ptr<PointLioEstimator> estimator_;
  std::string world_frame_;
  std::string body_frame_;
  bool publish_tf_ = true;
  bool publish_path_ = true;
  std::size_t path_capacity_ = 10000;
  nav_msgs::msg::Path path_;
  static constexpr std::size_t maximum_pending_clouds_ = 20;
  std::deque<PendingCloud> pending_clouds_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr registered_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr body_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_callback_;
};

}  // namespace plio

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<plio::PointLioNode>();
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("point_lio"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
