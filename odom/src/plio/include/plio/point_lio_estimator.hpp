#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace plio {

using Point = pcl::PointXYZINormal;
using Cloud = pcl::PointCloud<Point>;

struct ImuSample {
  double stamp = 0.0;
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
};

struct Parameters {
  double surface_leaf_size = 0.30;
  double map_resolution = 0.30;
  double blind = 0.25;
  double detection_range = 100.0;
  double plane_threshold = 0.10;
  double match_scale = 81.0;
  double lidar_measurement_covariance = 0.01;
  double imu_gyro_measurement_covariance = 0.01;
  double imu_accel_measurement_covariance = 0.01;
  double velocity_covariance = 20.0;
  double gyro_covariance = 1000.0;
  double accel_covariance = 500.0;
  double gyro_bias_covariance = 1.0e-4;
  double accel_bias_covariance = 1.0e-4;
  std::size_t initialization_samples = 100;
  std::size_t initialization_points = 100;
  std::size_t point_filter = 2;
  int nearby_type = 18;
  bool estimate_extrinsics = false;
  Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, -9.80665);
  Eigen::Vector3d lidar_to_imu_translation = Eigen::Vector3d::Zero();
  Eigen::Matrix3d lidar_to_imu_rotation = Eigen::Matrix3d::Identity();
};

struct Result {
  bool waiting_for_imu = false;
  bool initialized = false;
  double stamp = 0.0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Cloud::Ptr registered{new Cloud};
  Cloud::Ptr body{new Cloud};
};

struct ImuInitializationReport {
  bool valid = false;
  std::size_t samples = 0;
  Eigen::Vector3d mean_acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
};

class PointLioEstimator {
 public:
  explicit PointLioEstimator(Parameters parameters);
  ~PointLioEstimator();
  PointLioEstimator(PointLioEstimator &&) noexcept;
  PointLioEstimator &operator=(PointLioEstimator &&) noexcept;
  PointLioEstimator(const PointLioEstimator &) = delete;
  PointLioEstimator &operator=(const PointLioEstimator &) = delete;

  void addImu(const ImuSample &sample);
  Result process(const Cloud::ConstPtr &cloud, double stamp);
  void reset();
  bool initialized() const;
  ImuInitializationReport initializationReport() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace plio
