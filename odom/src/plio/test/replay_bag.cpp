// Replays extracted fused IMU and point clouds through the production estimator.
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include "plio/imu_conditioning.hpp"
#include "plio/point_lio_estimator.hpp"

int main(int argc, char **argv) {
  if (argc != 5) {
    std::cerr << "usage: replay_bag imu.bin clouds.bin output.csv imu_conditioning.txt\n";
    return 2;
  }

  plio::ImuConditioning conditioning;
  std::ifstream config(argv[4]);
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      config >> conditioning.rotation(row, column);
    }
  }
  for (int axis = 0; axis < 3; ++axis) config >> conditioning.lever_arm[axis];
  if (!config) return 3;

  std::ifstream imu_file(argv[1], std::ios::binary);
  std::ifstream cloud_file(argv[2], std::ios::binary);
  std::ofstream output(argv[3]);
  if (!imu_file || !cloud_file || !output) return 4;

  std::vector<plio::ImuSample> imu;
  double values[7];
  while (imu_file.read(reinterpret_cast<char *>(values), sizeof(values))) {
    plio::ImuSample sample;
    sample.stamp = values[0];
    sample.angular_velocity = {values[1], values[2], values[3]};
    sample.acceleration = {values[4], values[5], values[6]};
    imu.push_back(conditioning.apply(sample));
  }

  plio::PointLioEstimator estimator(plio::Parameters{});
  std::size_t next_imu = 0;
  output << "stamp,x,y,z,points,matches,ms\n" << std::setprecision(15);
  double stamp;
  std::uint32_t count;
  while (cloud_file.read(reinterpret_cast<char *>(&stamp), sizeof(stamp)) &&
         cloud_file.read(reinterpret_cast<char *>(&count), sizeof(count))) {
    std::vector<float> packed(static_cast<std::size_t>(count) * 5);
    if (!cloud_file.read(reinterpret_cast<char *>(packed.data()),
                         packed.size() * sizeof(float))) return 5;
    plio::Cloud::Ptr cloud(new plio::Cloud);
    cloud->reserve(count);
    double scan_end = stamp;
    for (std::size_t i = 0; i < count; ++i) {
      plio::Point point{};
      point.x = packed[5 * i];
      point.y = packed[5 * i + 1];
      point.z = packed[5 * i + 2];
      point.intensity = packed[5 * i + 3];
      point.curvature = packed[5 * i + 4] * 1000.0F;
      cloud->push_back(point);
      scan_end = std::max(scan_end, stamp + packed[5 * i + 4]);
    }
    while (next_imu < imu.size() && imu[next_imu].stamp <= scan_end + 0.015) {
      estimator.addImu(imu[next_imu++]);
    }
    const auto begin = std::chrono::steady_clock::now();
    const auto result = estimator.process(cloud, stamp);
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - begin).count();
    if (result.initialized) {
      output << stamp << ',' << result.position.x() << ',' << result.position.y()
             << ',' << result.position.z() << ',' << result.filtered_points
             << ',' << result.matched_points << ',' << elapsed_ms << '\n';
    }
  }
  return 0;
}
