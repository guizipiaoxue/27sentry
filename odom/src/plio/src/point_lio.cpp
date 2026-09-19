// ROS 2 wrapper around Point-LIO's point-by-point iterated Kalman filter.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <deque>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <numeric>
#include <sstream>
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
#include <sys/times.h>
#include <unistd.h>

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
    state_publish_rate_hz_ = parameter<double>(
        *this, "publish.state_rate_hz", 100.0);
    maximum_extrapolation_seconds_ = parameter<double>(
        *this, "publish.max_extrapolation_seconds", 0.05);
    if (!(state_publish_rate_hz_ > 0.0) ||
        maximum_extrapolation_seconds_ < 0.0) {
      throw std::runtime_error(
          "publish.state_rate_hz must be positive and "
          "publish.max_extrapolation_seconds must be non-negative");
    }
    terminal_enabled_ = parameter<bool>(*this, "terminal.enabled", true);
    terminal_clear_screen_ =
        parameter<bool>(*this, "terminal.clear_screen", true);
    path_capacity_ = static_cast<std::size_t>(std::max<std::int64_t>(
        1, parameter<std::int64_t>(*this, "publish.path_capacity", 10000)));

    Parameters parameters;
    parameters.surface_leaf_size = parameter<double>(*this, "filter_size_surf", 0.30);
    parameters.map_resolution = parameter<double>(*this, "mapping.ivox_grid_resolution", 0.30);
    parameters.blind = parameter<double>(*this, "preprocess.blind", 0.50);
    parameters.detection_range = parameter<double>(*this, "mapping.det_range", 100.0);
    parameters.plane_threshold = parameter<double>(*this, "mapping.plane_thr", 0.10);
    parameters.match_scale = parameter<double>(*this, "mapping.match_s", 81.0);
    parameters.lidar_measurement_covariance = parameter<double>(*this, "mapping.lidar_meas_cov", 0.01);
    parameters.imu_gyro_measurement_covariance = parameter<double>(*this, "mapping.imu_meas_omg_cov", 0.01);
    parameters.imu_accel_measurement_covariance = parameter<double>(*this, "mapping.imu_meas_acc_cov", 0.01);
    parameters.velocity_covariance = parameter<double>(*this, "mapping.velocity_cov", 20.0);
    parameters.gyro_covariance = parameter<double>(*this, "mapping.gyr_cov_input", 1000.0);
    parameters.accel_covariance = parameter<double>(*this, "mapping.acc_cov_input", 500.0);
    parameters.gyro_bias_covariance = parameter<double>(*this, "mapping.b_gyr_cov", 1.0e-4);
    parameters.accel_bias_covariance = parameter<double>(*this, "mapping.b_acc_cov", 1.0e-4);
    parameters.initialization_samples = static_cast<std::size_t>(std::max<std::int64_t>(
        10, parameter<std::int64_t>(*this, "imu.initialization_samples", 100)));
    parameters.initialization_max_gyro_mean = parameter<double>(
        *this, "imu.initialization_max_gyro_mean", 0.05);
    parameters.initialization_max_gyro_stddev = parameter<double>(
        *this, "imu.initialization_max_gyro_stddev", 0.02);
    parameters.initialization_max_accel_stddev = parameter<double>(
        *this, "imu.initialization_max_accel_stddev", 0.30);
    parameters.initialization_max_gravity_error = parameter<double>(
        *this, "imu.initialization_max_gravity_error", 0.75);
    parameters.initialization_points = static_cast<std::size_t>(std::max<std::int64_t>(
        10, parameter<std::int64_t>(*this, "mapping.initialization_points", 100)));
    parameters.point_filter = static_cast<std::size_t>(std::max<std::int64_t>(
        1, parameter<std::int64_t>(*this, "preprocess.point_filter_num", 2)));
    parameters.maximum_tracking_points = static_cast<std::size_t>(
        std::max<std::int64_t>(
            2, parameter<std::int64_t>(
                   *this, "mapping.max_tracking_points", 1200)));
    parameters.point_time_bin_seconds = 1.0e-3 * parameter<double>(
        *this, "mapping.point_time_bin_ms", 1.0);
    if (parameters.point_time_bin_seconds < 0.0) {
      throw std::runtime_error("mapping.point_time_bin_ms must be non-negative");
    }
    parameters.nearby_type = parameter<int>(*this, "mapping.ivox_nearby_type", 18);
    parameters.estimate_extrinsics = parameter<bool>(*this, "mapping.extrinsic_est_en", false);
    parameters.gravity = vectorParameter(
        *this, "mapping.gravity", Eigen::Vector3d(0.0, 0.0, -9.80665));
    gravity_reference_ = parameters.gravity.norm();
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
    state_callback_group_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    state_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / state_publish_rate_hz_),
        std::bind(&PointLioNode::publishState, this), state_callback_group_);
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
            std::lock_guard<std::mutex> publish_lock(publish_mutex_);
            std::lock_guard<std::mutex> terminal_lock(terminal_mutex_);
            estimator_->reset();
            imu_initialization_logged_ = false;
            pending_clouds_.clear();
            path_.poses.clear();
            imu_rates_.clear();
            lidar_rates_.clear();
            computation_times_.clear();
            cpu_loads_.clear();
            last_imu_stamp_ = 0.0;
            last_lidar_stamp_ = 0.0;
            latest_imu_stamp_ = 0.0;
            first_result_stamp_ = 0.0;
            distance_traveled_ = 0.0;
            published_scans_ = 0;
            last_cpu_clock_ = 0;
            last_cpu_system_ = 0;
            last_cpu_user_ = 0;
            origin_position_.setZero();
            last_position_for_distance_.setZero();
            latest_state_.valid = false;
            RCLCPP_INFO(get_logger(), "Point-LIO state and map reset");
          }
          return response;
        });
    declare_parameter<bool>("reset", false);
    RCLCPP_INFO(get_logger(), "Point-LIO: fused pointcloud + IMU, frames %s -> %s",
                world_frame_.c_str(), body_frame_.c_str());
    RCLCPP_INFO(
        get_logger(),
        "Point-LIO output: odometry/TF %.1f Hz, tracking point cap %zu, "
        "point-time bin %.3f ms",
        state_publish_rate_hz_, parameters.maximum_tracking_points,
        parameters.point_time_bin_seconds * 1.0e3);
    RCLCPP_INFO(
        get_logger(),
        "Point-LIO covariance: lidar %.4g, IMU gyro %.4g, IMU accel %.4g; "
        "Q velocity %.4g, omega %.4g, accel %.4g, bg %.4g, ba %.4g",
        parameters.lidar_measurement_covariance,
        parameters.imu_gyro_measurement_covariance,
        parameters.imu_accel_measurement_covariance,
        parameters.velocity_covariance, parameters.gyro_covariance,
        parameters.accel_covariance, parameters.gyro_bias_covariance,
        parameters.accel_bias_covariance);
    if (terminal_enabled_) printTerminalHeader();
  }

 private:
  struct PendingCloud {
    Cloud::Ptr cloud;
    builtin_interfaces::msg::Time stamp;
  };

  struct PublishedResult {
    Result result;
    builtin_interfaces::msg::Time stamp;
    double computation_seconds = 0.0;
    double average_computation_seconds = 0.0;
    double maximum_computation_seconds = 0.0;
    double imu_rate = 0.0;
    double lidar_rate = 0.0;
    double imu_lead = 0.0;
  };

  struct PublishedState {
    bool valid = false;
    double stamp = 0.0;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  };

  static void recordRate(
      std::deque<double> &rates, double &last_stamp, double stamp) {
    if (last_stamp > 0.0) {
      const double interval = stamp - last_stamp;
      if (interval > 1.0e-6 && interval < 1.0) {
        rates.push_back(1.0 / interval);
        if (rates.size() > 100) rates.pop_front();
      }
    }
    if (stamp > last_stamp) last_stamp = stamp;
  }

  static double average(const std::deque<double> &values) {
    if (values.empty()) return 0.0;
    return std::accumulate(values.begin(), values.end(), 0.0) /
           static_cast<double>(values.size());
  }

  static std::string vectorText(const Eigen::Vector3d &value, int precision) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value.x() << ' '
           << value.y() << ' ' << value.z();
    return stream.str();
  }

  static std::string quaternionText(
      const Eigen::Quaterniond &value, int precision) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value.w() << ' '
           << value.x() << ' ' << value.y() << ' ' << value.z();
    return stream.str();
  }

  static std::string numberText(double value, int precision) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value;
    return stream.str();
  }

  static void printLine(const std::string &content) {
    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
              << content.substr(0, 66) << "|\n";
  }

  void printTerminalHeader() const {
    if (terminal_clear_screen_) std::cout << "\033[2J\033[1;1H";
    std::cout << "\n+-------------------------------------------------------------------+\n"
              << "|                    Point-LIO Odometry                            |\n"
              << "+-------------------------------------------------------------------+\n"
              << std::flush;
  }

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
      recordRate(imu_rates_, last_imu_stamp_, sample.stamp);
      latest_imu_stamp_ = std::max(latest_imu_stamp_, sample.stamp);
      estimator_->addImu(sample);
      if (!estimator_->initialized()) {
        const ImuInitializationReport report =
            estimator_->initializationReport();
        if (report.samples > 0) {
          RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "waiting for stationary Point-LIO IMU initialization: accel "
              "norm %.4f, accel std %.4f, gyro mean %.4f, gyro std %.4f, "
              "gravity error %.4f",
              report.accel_norm, report.accel_stddev,
              report.gyro_mean_norm, report.gyro_stddev,
              report.gravity_error);
        }
      }
      if (!imu_initialization_logged_ && estimator_->initialized()) {
        const ImuInitializationReport report =
            estimator_->initializationReport();
        RCLCPP_INFO(
            get_logger(),
            "Point-LIO IMU initialized with %zu samples; mean accel norm "
            "%.6f m/s^2 [%.6f %.6f %.6f]; gravity [%.6f %.6f %.6f]; "
            "gyro bias [%.6f %.6f %.6f]; accel bias [%.6f %.6f %.6f]",
            report.samples, report.mean_acceleration.norm(),
            report.mean_acceleration.x(), report.mean_acceleration.y(),
            report.mean_acceleration.z(), report.gravity.x(),
            report.gravity.y(), report.gravity.z(), report.gyro_bias.x(),
            report.gyro_bias.y(), report.gyro_bias.z(), report.accel_bias.x(),
            report.accel_bias.y(), report.accel_bias.z());
        imu_initialization_logged_ = true;
      }
      ready = drainPendingLocked();
    }
    for (const auto &item : ready) publishScan(item);
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
      bool invalid_point_time = false;
      for (std::size_t i = 0; i < cloud->size(); ++i) {
        const std::size_t row = i / message->width;
        const std::size_t column = i % message->width;
        const std::size_t offset = row * message->row_step +
            column * message->point_step + field->offset;
        float seconds = 0.0F;
        std::memcpy(&seconds, data + offset, sizeof(seconds));
        if (std::isfinite(seconds)) {
          cloud->points[i].curvature = seconds * 1000.0F;
        } else {
          invalid_point_time = true;
        }
      }
      if (invalid_point_time) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "input cloud contains non-finite point times; replacing them with zero");
      }
    } else {
      RCLCPP_WARN_ONCE(get_logger(),
                       "input cloud has no float32 time field; point timing is disabled");
    }

    std::vector<PublishedResult> ready;
    bool imu_initialized = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      recordRate(
          lidar_rates_, last_lidar_stamp_, stampSeconds(message->header.stamp));
      pending_clouds_.push_back({cloud, message->header.stamp});
      if (pending_clouds_.size() > maximum_pending_clouds_) {
        pending_clouds_.pop_front();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "dropping oldest cloud while waiting for fused IMU");
      }
      ready = drainPendingLocked();
      imu_initialized = estimator_->initialized();
    }
    if (ready.empty()) {
      if (!imu_initialized) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "waiting for fused IMU initialization");
      } else {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "waiting for enough IMU data and first map");
      }
    }
    for (const auto &item : ready) publishScan(item);
  }

  std::vector<PublishedResult> drainPendingLocked() {
    std::vector<PublishedResult> ready;
    while (!pending_clouds_.empty()) {
      const auto &pending = pending_clouds_.front();
      const auto begin = std::chrono::steady_clock::now();
      Result result = estimator_->process(
          pending.cloud, stampSeconds(pending.stamp));
      const double computation_seconds = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - begin).count();
      if (result.waiting_for_imu) break;
      if (result.initialized) {
        computation_times_.push_back(computation_seconds);
        if (computation_times_.size() > 100) computation_times_.pop_front();
        const double average_computation = average(computation_times_);
        const double maximum_computation = *std::max_element(
            computation_times_.begin(), computation_times_.end());
        const double imu_lead = latest_imu_stamp_ - result.scan_end;
        ready.push_back({
            std::move(result), pending.stamp, computation_seconds,
            average_computation, maximum_computation, average(imu_rates_),
            average(lidar_rates_), imu_lead});
      }
      pending_clouds_.pop_front();
    }
    return ready;
  }

  void publishState() {
    std::lock_guard<std::mutex> publish_lock(publish_mutex_);
    if (!latest_state_.valid) return;

    const rclcpp::Time output_time = now();
    const double requested_dt = output_time.seconds() - latest_state_.stamp;
    const double dt = std::clamp(
        requested_dt, 0.0, maximum_extrapolation_seconds_);
    const Eigen::Vector3d position =
        latest_state_.position + latest_state_.velocity * dt;
    Eigen::Quaterniond orientation = latest_state_.orientation;
    const double angular_speed = latest_state_.angular_velocity.norm();
    if (angular_speed > 1.0e-9 && dt > 0.0) {
      orientation = orientation * Eigen::Quaterniond(Eigen::AngleAxisd(
          angular_speed * dt,
          latest_state_.angular_velocity / angular_speed));
      orientation.normalize();
    }

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = output_time;
    odometry.header.frame_id = world_frame_;
    odometry.child_frame_id = body_frame_;
    odometry.pose.pose.position.x = position.x();
    odometry.pose.pose.position.y = position.y();
    odometry.pose.pose.position.z = position.z();
    odometry.pose.pose.orientation.x = orientation.x();
    odometry.pose.pose.orientation.y = orientation.y();
    odometry.pose.pose.orientation.z = orientation.z();
    odometry.pose.pose.orientation.w = orientation.w();
    odometry.twist.twist.linear.x = latest_state_.velocity.x();
    odometry.twist.twist.linear.y = latest_state_.velocity.y();
    odometry.twist.twist.linear.z = latest_state_.velocity.z();
    odometry.twist.twist.angular.x = latest_state_.angular_velocity.x();
    odometry.twist.twist.angular.y = latest_state_.angular_velocity.y();
    odometry.twist.twist.angular.z = latest_state_.angular_velocity.z();
    odometry_publisher_->publish(odometry);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odometry.header;
      transform.child_frame_id = body_frame_;
      transform.transform.translation.x = position.x();
      transform.transform.translation.y = position.y();
      transform.transform.translation.z = position.z();
      transform.transform.rotation = odometry.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }
  }

  void publishScan(const PublishedResult &published) {
    std::lock_guard<std::mutex> publish_lock(publish_mutex_);
    const Result &result = published.result;
    const builtin_interfaces::msg::Time &stamp = published.stamp;
    latest_state_.valid = true;
    latest_state_.stamp = result.scan_end;
    latest_state_.position = result.position;
    latest_state_.orientation = result.orientation;
    latest_state_.velocity = result.velocity;
    latest_state_.angular_velocity = result.angular_velocity;

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
    pose.header.stamp = stamp;
    pose.header.frame_id = world_frame_;
    pose.pose.position.x = result.position.x();
    pose.pose.position.y = result.position.y();
    pose.pose.position.z = result.position.z();
    pose.pose.orientation.x = result.orientation.x();
    pose.pose.orientation.y = result.orientation.y();
    pose.pose.orientation.z = result.orientation.z();
    pose.pose.orientation.w = result.orientation.w();
    if (publish_path_) {
      path_.header.stamp = stamp;
      path_.poses.push_back(pose);
      if (path_.poses.size() > path_capacity_) path_.poses.erase(path_.poses.begin());
      path_publisher_->publish(path_);
    }
    if (terminal_enabled_) printDashboard(published);
  }

  static double residentMemoryMb() {
    std::ifstream stream("/proc/self/statm");
    long total_pages = 0;
    long resident_pages = 0;
    stream >> total_pages >> resident_pages;
    (void)total_pages;
    if (!stream || resident_pages < 0) return 0.0;
    return static_cast<double>(resident_pages) *
           static_cast<double>(sysconf(_SC_PAGESIZE)) / 1.0e6;
  }

  double cpuLoadPercent() {
    struct tms sample {};
    const clock_t current = times(&sample);
    double load = 0.0;
    if (last_cpu_clock_ > 0 && current > last_cpu_clock_ &&
        sample.tms_stime >= last_cpu_system_ &&
        sample.tms_utime >= last_cpu_user_) {
      const double process_ticks =
          static_cast<double>(sample.tms_stime - last_cpu_system_) +
          static_cast<double>(sample.tms_utime - last_cpu_user_);
      load = process_ticks / static_cast<double>(current - last_cpu_clock_) *
             100.0 /
             static_cast<double>(std::max(1L, sysconf(_SC_NPROCESSORS_ONLN)));
    }
    last_cpu_clock_ = current;
    last_cpu_system_ = sample.tms_stime;
    last_cpu_user_ = sample.tms_utime;
    cpu_loads_.push_back(load);
    if (cpu_loads_.size() > 100) cpu_loads_.pop_front();
    return load;
  }

  void printDashboard(const PublishedResult &published) {
    std::lock_guard<std::mutex> terminal_lock(terminal_mutex_);
    const Result &result = published.result;
    ++published_scans_;
    if (first_result_stamp_ == 0.0) {
      first_result_stamp_ = result.stamp;
      origin_position_ = result.position;
      last_position_for_distance_ = result.position;
    }
    const double step = (result.position - last_position_for_distance_).norm();
    if (step >= 0.1 && std::isfinite(step)) {
      distance_traveled_ += step;
      last_position_for_distance_ = result.position;
    }

    const double scan_duration = result.scan_end - result.scan_start;
    const double match_ratio = result.filtered_points == 0
        ? 0.0
        : static_cast<double>(result.matched_points) /
              static_cast<double>(result.filtered_points);
    std::string health = "GOOD";
    std::string reason;
    const auto check = [&health, &reason](bool condition, const char *message) {
      if (!condition) return;
      health = "CHECK";
      if (!reason.empty()) reason += ", ";
      reason += message;
    };
    check(!result.position.allFinite() || !result.velocity.allFinite(),
          "non-finite state");
    check(std::abs(result.gravity.norm() - gravity_reference_) > 0.2,
          "gravity magnitude");
    check(result.gyro_bias.norm() > 0.2, "gyro bias");
    check(result.accel_bias.norm() > 1.0, "accel bias");
    check(scan_duration < 1.0e-4 || scan_duration > 0.2,
          "point timestamps");
    check(published.imu_lead < -1.0e-3 || published.imu_lead > 0.1,
          "IMU/cloud timing");
    if (published_scans_ > 5) {
      check(published.imu_rate < 100.0 || published.imu_rate > 400.0,
            "IMU rate");
      check(published.lidar_rate < 5.0 || published.lidar_rate > 20.0,
            "LiDAR rate");
    }
    if (published_scans_ > 5) {
      check(match_ratio < 0.01, "low plane matches");
    } else if (health == "GOOD") {
      health = "WARMUP";
    }
    if (published.lidar_rate > 0.0) {
      check(published.computation_seconds > 0.9 / published.lidar_rate,
            "processing slower than LiDAR");
    }

    const std::time_t wall_time = static_cast<std::time_t>(result.stamp);
    std::ostringstream date;
    date << std::put_time(std::localtime(&wall_time), "%a %b %d %H:%M:%S %Y");
    const Eigen::Vector3d body_velocity =
        result.orientation.conjugate() * result.velocity;
    const double start_offset_ms = (result.scan_start - result.stamp) * 1.0e3;
    const double end_offset_ms = (result.scan_end - result.stamp) * 1.0e3;
    const double cpu_load = cpuLoadPercent();
    const double average_cpu_load = average(cpu_loads_);
    const long processor_count = std::max(1L, sysconf(_SC_NPROCESSORS_ONLN));

    printTerminalHeader();
    printLine(date.str() + "   Elapsed: " +
              numberText(result.stamp - first_result_stamp_, 2) + " seconds");
    printLine("Sensor Rates: Livox @ " + numberText(published.lidar_rate, 2) +
              " Hz, IMU @ " + numberText(published.imu_rate, 2) + " Hz");
    std::cout << "|===================================================================|\n";
    printLine("Health             :: " + health +
              (reason.empty() ? "" : " - " + reason));
    printLine("Position     {W}   :: " + vectorText(result.position, 4));
    printLine("Orientation {W} wxyz :: " +
              quaternionText(result.orientation, 4));
    printLine("Lin Velocity {W}   :: " + vectorText(result.velocity, 4));
    printLine("Lin Velocity {B}   :: " + vectorText(body_velocity, 4));
    printLine("Ang Velocity {B}   :: " +
              vectorText(result.angular_velocity, 4));
    printLine("Gravity      {W}   :: " + vectorText(result.gravity, 5));
    printLine("Accel Bias         :: " + vectorText(result.accel_bias, 8));
    printLine("Gyro Bias          :: " + vectorText(result.gyro_bias, 8));
    std::cout << "|                                                                   |\n";
    printLine("Distance Traveled  :: " + numberText(distance_traveled_, 4) +
              " meters");
    printLine("Distance to Origin :: " +
              numberText((result.position - origin_position_).norm(), 4) +
              " meters");
    printLine("Registration       :: matches " +
              std::to_string(result.matched_points) + "/" +
              std::to_string(result.filtered_points) + " (" +
              numberText(match_ratio * 100.0, 1) + "%), map voxels " +
              std::to_string(result.map_voxels));
    printLine("Point Time [ms]    :: " + numberText(start_offset_ms, 3) +
              " .. " + numberText(end_offset_ms, 3) +
              ", IMU lead " + numberText(published.imu_lead * 1.0e3, 3));
    std::cout << "|                                                                   |\n";
    printLine("Computation Time   :: " +
              numberText(published.computation_seconds * 1.0e3, 2) +
              " ms // Avg: " +
              numberText(published.average_computation_seconds * 1.0e3, 2) +
              " / Max: " +
              numberText(published.maximum_computation_seconds * 1.0e3, 2));
    printLine("Cores Utilized     :: " +
              numberText(cpu_load / 100.0 * processor_count, 2) + " / " +
              std::to_string(processor_count) + " cores");
    printLine("CPU Load           :: " + numberText(cpu_load, 2) +
              "% // Avg: " + numberText(average_cpu_load, 2) + "%");
    printLine("RAM Allocation     :: " + numberText(residentMemoryMb(), 2) +
              " MB");
    std::cout << "+-------------------------------------------------------------------+\n"
              << std::flush;
  }

  std::mutex mutex_;
  std::mutex publish_mutex_;
  std::unique_ptr<PointLioEstimator> estimator_;
  std::string world_frame_;
  std::string body_frame_;
  bool publish_tf_ = true;
  bool publish_path_ = true;
  double state_publish_rate_hz_ = 100.0;
  double maximum_extrapolation_seconds_ = 0.05;
  bool terminal_enabled_ = true;
  bool terminal_clear_screen_ = true;
  bool imu_initialization_logged_ = false;
  std::size_t path_capacity_ = 10000;
  nav_msgs::msg::Path path_;
  static constexpr std::size_t maximum_pending_clouds_ = 20;
  std::deque<PendingCloud> pending_clouds_;
  std::deque<double> imu_rates_;
  std::deque<double> lidar_rates_;
  std::deque<double> computation_times_;
  std::deque<double> cpu_loads_;
  double last_imu_stamp_ = 0.0;
  double last_lidar_stamp_ = 0.0;
  double latest_imu_stamp_ = 0.0;
  double first_result_stamp_ = 0.0;
  double distance_traveled_ = 0.0;
  double gravity_reference_ = 9.80665;
  std::size_t published_scans_ = 0;
  clock_t last_cpu_clock_ = 0;
  clock_t last_cpu_system_ = 0;
  clock_t last_cpu_user_ = 0;
  Eigen::Vector3d origin_position_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_position_for_distance_ = Eigen::Vector3d::Zero();
  PublishedState latest_state_;
  std::mutex terminal_mutex_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr registered_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr body_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::CallbackGroup::SharedPtr state_callback_group_;
  rclcpp::TimerBase::SharedPtr state_timer_;
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
