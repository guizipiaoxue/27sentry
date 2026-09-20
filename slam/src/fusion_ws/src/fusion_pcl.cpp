// Use the calibrated lidar-to-gimbal transforms to fuse lidar 5 and lidar 3.
// Cloud pairs are published at the lidar rate (about 10 Hz), while synchronized
// IMU pairs are rotated into the gimbal frame, averaged, and published at the
// IMU rate (about 200 Hz). The lidar5 timestamps define the fused IMU timeline;
// lidar3 measurements are interpolated onto it. ROS header stamps always use
// sec + nanosec; the different device packet units must not be applied to
// header.stamp.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <yaml-cpp/yaml.h>

struct EIGEN_ALIGN16 TimedPoint {
  PCL_ADD_POINT4D;
  float intensity;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
    TimedPoint,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
        (float, time, time))

class FusionPcl final : public rclcpp::Node {
 public:
  using CustomMsg = livox_ros_driver2::msg::CustomMsg;
  using Imu = sensor_msgs::msg::Imu;
  using Cloud = pcl::PointCloud<TimedPoint>;

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
    imu_interpolation_max_gap_ = declare_parameter<double>(
        "imu_interpolation_max_gap", 0.020);
    imu_calibration_seconds_ = declare_parameter<double>(
        "imu_calibration_seconds", 3.0);
    imu_calibration_min_samples_ = declare_parameter<std::int64_t>(
        "imu_calibration_min_samples", 400);
    gravity_ = declare_parameter<double>("gravity", 9.80665);
    imu_accel_unit_ = declare_parameter<std::string>(
        "imu_accel_unit", "auto");
    max_gyro_stddev_ = declare_parameter<double>(
        "max_calibration_gyro_stddev", 0.02);
    max_gyro_mean_ = declare_parameter<double>(
        "max_calibration_gyro_mean", 0.1);
    max_accel_stddev_ = declare_parameter<double>(
        "max_calibration_accel_stddev", 0.30);
    max_gravity_error_ = declare_parameter<double>(
        "max_calibration_gravity_error", 0.75);
    max_queue_size_ = static_cast<std::size_t>(std::max<std::int64_t>(
        2, declare_parameter<std::int64_t>("max_queue_size", 100)));
    if (!(cloud_sync_tolerance_ > 0.0) ||
        !(imu_interpolation_max_gap_ > 0.0)) {
      throw std::runtime_error(
          "cloud_sync_tolerance and imu_interpolation_max_gap must be in "
          "positive seconds");
    }
    if (imu_accel_unit_ != "m/s^2" && imu_accel_unit_ != "mps2" &&
        imu_accel_unit_ != "g" && imu_accel_unit_ != "auto") {
      throw std::runtime_error(
          "imu_accel_unit must be m/s^2, mps2, g, or auto");
    }

    const std::string lidar5_calibration = declare_parameter<std::string>(
        "lidar5_calibration",
        package_share + "/config/gimbal_lidar_5.yaml");
    const std::string lidar3_calibration = declare_parameter<std::string>(
        "lidar3_calibration",
        package_share + "/config/gimbal_lidar_3.yaml");
    transforms_[0] = loadTransform(lidar5_calibration);
    transforms_[1] = loadTransform(lidar3_calibration);

    const auto qos = rclcpp::SensorDataQoS();
    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        cloud_output_topic_, output_qos);
    imu_pub_ = create_publisher<Imu>(imu_output_topic_, output_qos);
    calibration_pub_ = create_publisher<std_msgs::msg::Bool>(
        "/gimbal/imu_calibrated",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    std_msgs::msg::Bool initial_status;
    initial_status.data = false;
    calibration_pub_->publish(initial_status);

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
    RCLCPP_INFO(
        get_logger(),
        "keep both lidars stationary for %.1f s while their IMUs calibrate",
        imu_calibration_seconds_);
    RCLCPP_INFO(
        get_logger(),
        "MID360 IMU fusion uses lidar5 timestamps and interpolates lidar3 "
        "(maximum data gap %.1f ms)",
        imu_interpolation_max_gap_ * 1000.0);
    RCLCPP_INFO(
        get_logger(), "configured raw IMU acceleration unit: %s",
        imu_accel_unit_.c_str());
  }

 private:
  template <typename MessageT>
  struct TimedMessage {
    rclcpp::Time stamp;
    std::shared_ptr<const MessageT> message;
  };

  struct ImuCalibration {
    std::chrono::steady_clock::time_point start;
    Eigen::Vector3d gyro_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_square_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_square_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_bias_gimbal = Eigen::Vector3d::Zero();
    Eigen::Matrix3d imu_to_gimbal = Eigen::Matrix3d::Identity();
    double accel_scale = 1.0;
    std::size_t samples = 0;
    bool started = false;
    bool complete = false;
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
      const CustomMsg &message, const Eigen::Matrix4d &transform,
      double header_offset) {
    Cloud::Ptr input(new Cloud);
    input->reserve(message.points.size());
    // Point-LIO consumes the per-point offset in seconds.  Keep it through the
    // two-lidar transform instead of collapsing the fused scan to PointXYZI.
    // Both packets use their own header as time zero; the synchronization
    // tolerance keeps those two origins close enough for a single scan.
    for (const auto &point : message.points) {
      const double range_squared =
          point.x * point.x + point.y * point.y + point.z * point.z;
      if (std::isfinite(point.x) && std::isfinite(point.y) &&
          std::isfinite(point.z) && range_squared > 1e-8) {
        TimedPoint output;
        output.x = point.x;
        output.y = point.y;
        output.z = point.z;
        output.intensity = static_cast<float>(point.reflectivity);
        output.time = static_cast<float>(
            static_cast<double>(point.offset_time) * 1.0e-9 + header_offset);
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
    // Keep the later packet stamp used by the 1c37bd6 fusion pipeline. Point
    // offsets include each packet's signed header delta, so their absolute
    // acquisition times remain correct even when the earlier packet is first.
    const rclcpp::Time output_stamp =
        lidar5.stamp >= lidar3.stamp ? lidar5.stamp : lidar3.stamp;
    Cloud::Ptr fused = transformCloud(
        *lidar5.message, transforms_[0],
        (lidar5.stamp - output_stamp).seconds());
    const Cloud::Ptr cloud3 = transformCloud(
        *lidar3.message, transforms_[1],
        (lidar3.stamp - output_stamp).seconds());
    *fused += *cloud3;

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*fused, output);
    output.header.frame_id = frame_id_;
    output.header.stamp = output_stamp;
    cloud_pub_->publish(output);
  }

  void imuCallback(std::size_t index, Imu::ConstSharedPtr message) {
    if (!calibrations_[index].complete) {
      calibrateImu(index, *message);
      return;
    }
    if (!imu_calibration_complete_) {
      return;
    }
    imu_queues_[index].push_back(
        {rclcpp::Time(message->header.stamp), std::move(message)});
    limitQueue(imu_queues_[index]);
    synchronizeImus();
  }

  void resetCalibration(std::size_t index) {
    calibrations_[index] = ImuCalibration{};
  }

  void calibrateImu(std::size_t index, const Imu &imu) {
    const Eigen::Vector3d gyro = angularVelocity(imu);
    const Eigen::Vector3d accel = linearAcceleration(imu);
    if (!gyro.allFinite() || !accel.allFinite()) {
      return;
    }

    auto &calibration = calibrations_[index];
    const auto now = std::chrono::steady_clock::now();
    if (!calibration.started) {
      calibration.start = now;
      calibration.started = true;
    }
    calibration.gyro_sum += gyro;
    calibration.gyro_square_sum += gyro.cwiseProduct(gyro);
    calibration.accel_sum += accel;
    calibration.accel_square_sum += accel.cwiseProduct(accel);
    ++calibration.samples;

    const double elapsed =
        std::chrono::duration<double>(now - calibration.start).count();
    if (elapsed < imu_calibration_seconds_) {
      return;
    }
    if (calibration.samples <
        static_cast<std::size_t>(imu_calibration_min_samples_)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "lidar%zu IMU calibration has only %zu samples; keep waiting",
          index == 0 ? 5UL : 3UL, calibration.samples);
      return;
    }

    const double count = static_cast<double>(calibration.samples);
    const Eigen::Vector3d gyro_mean = calibration.gyro_sum / count;
    const Eigen::Vector3d accel_mean_raw = calibration.accel_sum / count;
    const double accel_norm_raw = accel_mean_raw.norm();
    if (imu_accel_unit_ == "g") {
      calibration.accel_scale = gravity_;
    } else if (imu_accel_unit_ == "auto") {
      calibration.accel_scale =
          std::abs(accel_norm_raw - 1.0) <
                  std::abs(accel_norm_raw - gravity_)
              ? gravity_
              : 1.0;
    } else {
      calibration.accel_scale = 1.0;
    }
    const Eigen::Vector3d accel_mean =
        calibration.accel_scale * accel_mean_raw;
    const Eigen::Vector3d gyro_variance =
        (calibration.gyro_square_sum / count -
         gyro_mean.cwiseProduct(gyro_mean))
            .cwiseMax(0.0);
    const Eigen::Vector3d accel_variance_raw =
        (calibration.accel_square_sum / count -
         accel_mean_raw.cwiseProduct(accel_mean_raw))
            .cwiseMax(0.0);
    const double gyro_stddev = gyro_variance.cwiseSqrt().maxCoeff();
    const double accel_stddev = calibration.accel_scale *
        accel_variance_raw.cwiseSqrt().maxCoeff();
    const double gravity_error = std::abs(accel_mean.norm() - gravity_);

    if (gyro_mean.norm() > max_gyro_mean_ ||
        gyro_stddev > max_gyro_stddev_ ||
        accel_stddev > max_accel_stddev_ ||
        gravity_error > max_gravity_error_) {
      RCLCPP_WARN(
          get_logger(),
          "lidar%zu moved during IMU calibration (gyro mean %.4f, gyro std "
          "%.4f, raw accel norm %.4f %s, accel std %.4f m/s^2, gravity "
          "error %.3f); restarting",
          index == 0 ? 5UL : 3UL, gyro_mean.norm(), gyro_stddev,
          accel_norm_raw, imu_accel_unit_.c_str(), accel_stddev, gravity_error);
      resetCalibration(index);
      return;
    }

    calibration.imu_to_gimbal = transforms_[index].block<3, 3>(0, 0);
    calibration.gyro_bias = gyro_mean;
    const Eigen::Vector3d measured_gravity =
        calibration.imu_to_gimbal * accel_mean;
    // As in DLIO initialization, gravity direction is attitude, not an IMU
    // extrinsic or accelerometer bias. Preserve it and only remove the static
    // magnitude residual; Point-LIO initializes world gravity from the fused
    // direction.
    calibration.accel_bias_gimbal = measured_gravity -
        measured_gravity.normalized() * gravity_;
    calibration.complete = true;

    RCLCPP_INFO(
        get_logger(),
        "lidar%zu IMU calibrated with %zu samples; accel input unit: %s; "
        "accel norm %.5f m/s^2, std %.5f; gyro bias [%.6f %.6f %.6f]",
        index == 0 ? 5UL : 3UL, calibration.samples,
        calibration.accel_scale == 1.0 ? "m/s^2" : "g", accel_mean.norm(),
        accel_stddev, gyro_mean.x(), gyro_mean.y(), gyro_mean.z());

    if (calibrations_[0].complete && calibrations_[1].complete) {
      imu_calibration_complete_ = true;
      imu_queues_[0].clear();
      imu_queues_[1].clear();
      std_msgs::msg::Bool status;
      status.data = true;
      calibration_pub_->publish(status);
      RCLCPP_INFO(
          get_logger(),
          "both IMUs calibrated; fused IMU output is now enabled");
    }
  }

  void synchronizeImus() {
    // Two independent 200 Hz MID360 IMUs normally have a sampling phase
    // difference of several milliseconds. Keep lidar5 as the output clock and
    // interpolate lidar3 instead of requiring two samples to have nearly equal
    // timestamps.
    while (!imu_queues_[0].empty() && imu_queues_[1].size() >= 2) {
      const rclcpp::Time target = imu_queues_[0].front().stamp;
      while (imu_queues_[1].size() >= 2 &&
             imu_queues_[1][1].stamp < target) {
        imu_queues_[1].pop_front();
      }
      if (imu_queues_[1].size() < 2) {
        return;
      }

      const auto &before = imu_queues_[1][0];
      const auto &after = imu_queues_[1][1];
      if (target < before.stamp) {
        // No earlier lidar3 measurement is available for this initial lidar5
        // sample, so interpolation is impossible.
        imu_queues_[0].pop_front();
        continue;
      }

      const double interval = (after.stamp - before.stamp).seconds();
      if (!(interval > 0.0)) {
        imu_queues_[1].pop_front();
        continue;
      }
      if (interval > imu_interpolation_max_gap_) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "lidar3 IMU data gap is %.4f s; dropping lidar5 sample instead "
            "of interpolating across the gap",
            interval);
        imu_queues_[0].pop_front();
        continue;
      }

      const double alpha = (target - before.stamp).seconds() / interval;
      publishInterpolatedImu(
          imu_queues_[0].front(), before, after,
          std::clamp(alpha, 0.0, 1.0));
      imu_queues_[0].pop_front();
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

  void publishInterpolatedImu(
      const TimedMessage<Imu> &imu5,
      const TimedMessage<Imu> &imu3_before,
      const TimedMessage<Imu> &imu3_after,
      double alpha) {
    const Eigen::Vector3d gyro5 = calibrations_[0].imu_to_gimbal *
        (angularVelocity(*imu5.message) - calibrations_[0].gyro_bias);
    const Eigen::Vector3d gyro3_raw =
        (1.0 - alpha) * angularVelocity(*imu3_before.message) +
        alpha * angularVelocity(*imu3_after.message);
    const Eigen::Vector3d gyro3 = calibrations_[1].imu_to_gimbal *
        (gyro3_raw - calibrations_[1].gyro_bias);
    const Eigen::Vector3d accel5 =
        calibrations_[0].imu_to_gimbal *
            (calibrations_[0].accel_scale *
             linearAcceleration(*imu5.message)) -
        calibrations_[0].accel_bias_gimbal;
    const Eigen::Vector3d accel3_raw =
        (1.0 - alpha) * linearAcceleration(*imu3_before.message) +
        alpha * linearAcceleration(*imu3_after.message);
    const Eigen::Vector3d accel3 =
        calibrations_[1].imu_to_gimbal *
            (calibrations_[1].accel_scale * accel3_raw) -
        calibrations_[1].accel_bias_gimbal;

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
    output.header.stamp = imu5.stamp;
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
  std::string imu_accel_unit_ = "auto";
  double cloud_sync_tolerance_ = 0.03;
  double imu_interpolation_max_gap_ = 0.020;
  double imu_calibration_seconds_ = 3.0;
  std::int64_t imu_calibration_min_samples_ = 400;
  double gravity_ = 9.80665;
  double max_gyro_stddev_ = 0.02;
  double max_gyro_mean_ = 0.1;
  double max_accel_stddev_ = 0.30;
  double max_gravity_error_ = 0.75;
  bool imu_calibration_complete_ = false;
  std::size_t max_queue_size_ = 100;
  std::array<Eigen::Matrix4d, 2> transforms_ = {
      Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity()};

  std::array<std::deque<TimedMessage<CustomMsg>>, 2> cloud_queues_;
  std::array<std::deque<TimedMessage<Imu>>, 2> imu_queues_;
  std::array<ImuCalibration, 2> calibrations_;
  std::array<rclcpp::Subscription<CustomMsg>::SharedPtr, 2> cloud_subs_;
  std::array<rclcpp::Subscription<Imu>::SharedPtr, 2> imu_subs_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr calibration_pub_;
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
