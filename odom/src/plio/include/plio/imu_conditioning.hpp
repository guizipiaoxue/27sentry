#pragma once

#include "plio/point_lio_estimator.hpp"

namespace plio {
// Optional fused virtual IMU correction. r is its effective position relative
// to the body origin in metres. Acceleration stays specific force in m/s^2.
class ImuConditioning {
 public:
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d lever_arm = Eigen::Vector3d::Zero();
  double smoothing_seconds = 0.02;
  void reset() { last_stamp_ = -1.0; omega_.setZero(); }
  ImuSample apply(const ImuSample &input) {
    ImuSample output = input;
    output.angular_velocity = rotation * input.angular_velocity;
    output.acceleration = rotation * input.acceleration;
    const double dt = input.stamp - last_stamp_;
    Eigen::Vector3d alpha = Eigen::Vector3d::Zero();
    if (last_stamp_ >= 0.0 && dt > 0.0 && dt <= .05) {
      const Eigen::Vector3d next = omega_ + dt / (smoothing_seconds + dt) *
          (output.angular_velocity - omega_);
      alpha = (next - omega_) / dt;
      omega_ = next;
    } else {
      omega_ = output.angular_velocity;
    }
    output.acceleration -= alpha.cross(lever_arm) +
        omega_.cross(omega_.cross(lever_arm));
    last_stamp_ = input.stamp;
    return output;
  }
 private:
  double last_stamp_ = -1.0;
  Eigen::Vector3d omega_ = Eigen::Vector3d::Zero();
};
}  // namespace plio
