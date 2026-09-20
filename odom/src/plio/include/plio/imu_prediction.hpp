#pragma once

#include <deque>
#include "plio/point_lio_estimator.hpp"

namespace plio {

// Sensor-clock propagation independent of the scan matcher. Caller serializes
// access. A delayed scan correction replays the retained IMU history.
class ImuPrediction {
 public:
  struct State {
    bool valid = false;
    double stamp = 0.0;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  };

  explicit ImuPrediction(double maximum_gap = 0.05) : maximum_gap_(maximum_gap) {}
  void reset() { history_.clear(); state_ = State{}; correction_stamp_ = 0.0; }
  bool add(const ImuSample &sample) {
    if (!std::isfinite(sample.stamp) || !sample.acceleration.allFinite() ||
        !sample.angular_velocity.allFinite() ||
        (!history_.empty() && sample.stamp <= history_.back().stamp)) return false;
    history_.push_back(sample);
    if (state_.valid) advance(sample);
    while (history_.size() > 2000) history_.pop_front();
    return true;
  }
  void correct(const Result &result) {
    state_.valid = result.initialized;
    state_.stamp = result.scan_end;
    correction_stamp_ = result.scan_end;
    state_.position = result.position;
    state_.orientation = result.orientation;
    state_.velocity = result.velocity;
    state_.angular_velocity = result.angular_velocity;
    gravity_ = result.gravity;
    gyro_bias_ = result.gyro_bias;
    accel_bias_ = result.accel_bias;
    previous_.stamp = state_.stamp;
    previous_.angular_velocity = result.angular_velocity + gyro_bias_;
    previous_.acceleration = result.acceleration + accel_bias_;
    // Interpolate the measurement at the correction time when bracketed.
    for (std::size_t i = 1; i < history_.size(); ++i) {
      const auto &a = history_[i - 1]; const auto &b = history_[i];
      if (a.stamp <= state_.stamp && b.stamp >= state_.stamp) {
        const double f = (state_.stamp - a.stamp) / (b.stamp - a.stamp);
        previous_.angular_velocity = (1.0 - f) * a.angular_velocity + f * b.angular_velocity;
        previous_.acceleration = (1.0 - f) * a.acceleration + f * b.acceleration;
        break;
      }
    }
    for (const auto &sample : history_) if (sample.stamp > state_.stamp) advance(sample);
  }
  State state(double maximum_correction_age = 0.5) const {
    State result = state_;
    result.valid = result.valid && result.stamp - correction_stamp_ <= maximum_correction_age;
    return result;
  }

 private:
  void advance(const ImuSample &sample) {
    if (!state_.valid) return;
    const double dt = sample.stamp - state_.stamp;
    if (dt <= 0.0) return;
    if (dt > maximum_gap_) { state_.valid = false; return; }
    const Eigen::Vector3d omega =
        0.5 * (previous_.angular_velocity + sample.angular_velocity) - gyro_bias_;
    Eigen::Quaterniond next = state_.orientation;
    if (omega.norm() > 1.0e-12) next = next * Eigen::Quaterniond(
        Eigen::AngleAxisd(omega.norm() * dt, omega.normalized()));
    next.normalize();
    const Eigen::Vector3d acceleration = 0.5 * (
        state_.orientation * (previous_.acceleration - accel_bias_) +
        next * (sample.acceleration - accel_bias_)) + gravity_;
    state_.position += state_.velocity * dt + 0.5 * acceleration * dt * dt;
    state_.velocity += acceleration * dt;
    state_.orientation = next;
    state_.angular_velocity = sample.angular_velocity - gyro_bias_;
    state_.stamp = sample.stamp;
    previous_ = sample;
  }
  double maximum_gap_;
  double correction_stamp_ = 0.0;
  State state_;
  ImuSample previous_;
  Eigen::Vector3d gravity_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_bias_ = Eigen::Vector3d::Zero();
  std::deque<ImuSample> history_;
};
}  // namespace plio
