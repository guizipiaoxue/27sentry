#include <cstdlib>
#include <iostream>
#include "plio/imu_prediction.hpp"
#include "plio/imu_conditioning.hpp"

int main() {
  {
    plio::ImuConditioning conditioning;
    conditioning.lever_arm = {.1, -.2, 0};
    conditioning.rotation = Eigen::AngleAxisd(.2, Eigen::Vector3d::UnitX()).toRotationMatrix();
    plio::ImuSample input;
    input.stamp = 1.;
    const Eigen::Vector3d omega(0, 0, 2);
    input.angular_velocity = conditioning.rotation.transpose() * omega;
    input.acceleration = conditioning.rotation.transpose() *
        (Eigen::Vector3d(0, 0, 9.80665) + omega.cross(omega.cross(conditioning.lever_arm)));
    const auto corrected = conditioning.apply(input);
    if ((corrected.acceleration - Eigen::Vector3d(0, 0, 9.80665)).norm() > 1e-10 ||
        (corrected.angular_velocity - omega).norm() > 1e-10) return EXIT_FAILURE;
  }
  // Delayed corrections, nonzero gyro bias, sensor time 37 s ahead of host.
  plio::ImuPrediction prediction;
  plio::Result correction;
  correction.initialized = true;
  correction.scan_end = 1037.0;
  correction.gravity = {0, 0, -9.80665};
  correction.acceleration = {0, 0, 9.80665};
  correction.gyro_bias = {.01, -.02, .03};
  correction.angular_velocity = {0, 0, 1.0};
  for (int i = 0; i <= 40; ++i) {
    plio::ImuSample sample;
    sample.stamp = 1037.0 + i * .005;
    sample.acceleration = {0, 0, 9.80665};
    sample.angular_velocity = correction.angular_velocity + correction.gyro_bias;
    prediction.add(sample);
  }
  prediction.correct(correction);
  const auto state = prediction.state();
  const Eigen::Quaterniond expected(Eigen::AngleAxisd(.2, Eigen::Vector3d::UnitZ()));
  if (!state.valid || state.position.norm() > 1e-10 ||
      state.velocity.norm() > 1e-10 || state.orientation.angularDistance(expected) > 1e-10 ||
      std::abs(state.stamp - 1037.2) > 1e-10 || prediction.state(.1).valid) {
    std::cerr << "sensor-clock yaw prediction/correction failed\n"; return EXIT_FAILURE;
  }
  plio::ImuSample gap;
  gap.stamp = 1037.4; gap.acceleration = {0, 0, 9.80665};
  prediction.add(gap);
  if (prediction.state().valid || prediction.add(gap)) return EXIT_FAILURE;
  prediction.reset();
  if (prediction.state().valid) return EXIT_FAILURE;
  // Translational acceleration must be integrated, not only constant velocity.
  correction.scan_end = 10.; correction.angular_velocity.setZero();
  correction.gyro_bias.setZero(); correction.acceleration = {1, 0, 9.80665};
  prediction.correct(correction);
  for (int i = 1; i <= 20; ++i) {
    plio::ImuSample sample;
    sample.stamp = 10. + i * .005; sample.acceleration = correction.acceleration;
    prediction.add(sample);
  }
  const auto moving = prediction.state();
  if (!moving.valid || std::abs(moving.position.x() - .005) > 1e-10 ||
      std::abs(moving.velocity.x() - .1) > 1e-10) return EXIT_FAILURE;
  std::cout << "IMU prediction: delayed correction, yaw, acceleration, gap, reset passed\n";
  return EXIT_SUCCESS;
}
