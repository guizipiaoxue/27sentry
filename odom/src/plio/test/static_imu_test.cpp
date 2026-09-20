#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>

#include "plio/point_lio_estimator.hpp"

namespace {

constexpr double kGravity = 9.80665;

plio::Cloud::Ptr makePlane(bool duplicate_returns = false) {
  plio::Cloud::Ptr cloud(new plio::Cloud);
  for (int x = -10; x <= 10; ++x) {
    for (int y = -10; y <= 10; ++y) {
      plio::Point point;
      point.x = static_cast<float>(x) * 0.2F;
      point.y = static_cast<float>(y) * 0.2F;
      point.z = 0.0F;
      point.intensity = 1.0F;
      point.curvature = static_cast<float>(x + 10) / 20.0F * 90.0F;
      cloud->push_back(point);
      // A second return in the same voxel, at a different acquisition time.
      // A centroid would create a coordinate/time pair absent from the input.
      if (duplicate_returns) {
        point.x += 0.003F;
        point.curvature += 1.0F;
        cloud->push_back(point);
      }
    }
  }
  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

bool near(const Eigen::Vector3d &actual, const Eigen::Vector3d &expected,
          double tolerance) {
  return (actual - expected).norm() <= tolerance;
}

plio::Result runStaticScenario(const Eigen::Vector3d &acceleration,
                               plio::ImuInitializationReport &report) {
  plio::Parameters parameters;
  parameters.initialization_samples = 100;
  parameters.initialization_points = 100;
  parameters.point_filter = 1;
  parameters.maximum_tracking_points = 128;
  parameters.surface_leaf_size = 0.05;
  parameters.map_resolution = 0.10;
  plio::PointLioEstimator estimator(parameters);

  constexpr double kDt = 0.005;
  for (int i = 0; i <= 500; ++i) {
    plio::ImuSample sample;
    sample.stamp = static_cast<double>(i) * kDt;
    sample.acceleration = acceleration;
    sample.angular_velocity.setZero();
    estimator.addImu(sample);
  }
  report = estimator.initializationReport();

  const auto plane = makePlane();
  plio::Result result;
  for (int i = 100; i <= 400; i += 20) {
    result = estimator.process(plane, static_cast<double>(i) * kDt);
  }
  return result;
}

}  // namespace

int main() {
  {
    plio::Parameters parameters;
    parameters.initialization_samples = 100;
    plio::PointLioEstimator moving_estimator(parameters);
    for (int i = 0; i < 100; ++i) {
      plio::ImuSample sample;
      sample.stamp = static_cast<double>(i) * 0.005;
      sample.acceleration = Eigen::Vector3d(
          i % 2 == 0 ? 1.0 : -1.0, 0.0, kGravity);
      moving_estimator.addImu(sample);
    }
    const plio::ImuInitializationReport moving_report =
        moving_estimator.initializationReport();
    if (moving_estimator.initialized() || moving_report.valid ||
        moving_report.accel_stddev < 0.9) {
      std::cerr << "moving IMU window was accepted during initialization\n";
      return EXIT_FAILURE;
    }
  }

  plio::ImuInitializationReport report;
  plio::Result result = runStaticScenario(
      Eigen::Vector3d(0.0, 0.0, kGravity), report);
  if (!report.valid || report.samples != 100 ||
      !near(report.gravity, Eigen::Vector3d(0.0, 0.0, -kGravity), 1.0e-9) ||
      report.gyro_bias.norm() > 1.0e-12 ||
      report.accel_bias.norm() > 1.0e-12) {
    std::cerr << "invalid IMU initialization: gravity="
              << report.gravity.transpose() << " gyro_bias="
              << report.gyro_bias.transpose() << " accel_bias="
              << report.accel_bias.transpose() << '\n';
    return EXIT_FAILURE;
  }

  if (!result.initialized || result.position.norm() > 1.0e-4 ||
      result.velocity.norm() > 1.0e-4 ||
      result.orientation.angularDistance(Eigen::Quaterniond::Identity()) >
          1.0e-4 ||
      !near(result.gravity, Eigen::Vector3d(0.0, 0.0, -kGravity), 1.0e-9) ||
      result.gyro_bias.norm() > 1.0e-12 ||
      result.accel_bias.norm() > 1.0e-12 || result.filtered_points != 128 ||
      result.matched_points == 0 || result.map_voxels == 0 ||
      std::abs((result.scan_end - result.scan_start) - 0.09) > 1.0e-6) {
    std::cerr << "static estimate drifted: position="
              << result.position.transpose() << " velocity="
              << result.velocity.transpose() << " rotation_error="
              << result.orientation.angularDistance(
                     Eigen::Quaterniond::Identity())
              << '\n';
    return EXIT_FAILURE;
  }

  const Eigen::Vector3d tilted_acceleration(
      2.0, -1.0, std::sqrt(kGravity * kGravity - 5.0));
  plio::ImuInitializationReport tilted_report;
  const plio::Result tilted_result =
      runStaticScenario(tilted_acceleration, tilted_report);
  if (!near(tilted_report.gravity, -tilted_acceleration, 1.0e-9) ||
      tilted_result.position.norm() > 1.0e-4 ||
      tilted_result.velocity.norm() > 1.0e-4) {
    std::cerr << "tilted static estimate drifted: gravity="
              << tilted_report.gravity.transpose() << " position="
              << tilted_result.position.transpose() << " velocity="
              << tilted_result.velocity.transpose() << '\n';
    return EXIT_FAILURE;
  }

  const auto source = makePlane(true);
  plio::Parameters voxel_parameters;
  voxel_parameters.point_filter = 1;
  voxel_parameters.surface_leaf_size = 0.05;
  plio::PointLioEstimator voxel_estimator(voxel_parameters);
  for (int i = 0; i <= 150; ++i) {
    plio::ImuSample sample;
    sample.stamp = i * 0.005;
    sample.acceleration = Eigen::Vector3d(0, 0, kGravity);
    voxel_estimator.addImu(sample);
  }
  const auto voxel_result = voxel_estimator.process(source, 0.5);
  if (!voxel_result.initialized || voxel_result.body->empty() ||
      std::abs(voxel_result.scan_end - 0.591) > 1e-6) return EXIT_FAILURE;
  for (const auto &point : voxel_result.body->points) {
    bool found = false;
    for (const auto &raw : source->points) {
      if (std::abs(point.x - raw.x) < 1e-5 &&
          std::abs(point.y - raw.y) < 1e-5 &&
          std::abs(point.z - raw.z) < 1e-5 &&
          point.curvature == raw.curvature) { found = true; break; }
    }
    if (!found) {
      std::cerr << "voxel filtering changed measurement coordinates/time\n";
      return EXIT_FAILURE;
    }
  }
  std::cout << "static IMU test passed: position="
            << result.position.transpose() << " velocity="
            << result.velocity.transpose() << '\n';
  return EXIT_SUCCESS;
}
