#pragma once

#include <algorithm>
#include <cmath>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace dlio {
struct MotionImu {
  double stamp = 0.;
  double dt = 0.;
  Eigen::Vector3f ang_vel = Eigen::Vector3f::Zero();
  Eigen::Vector3f lin_accel = Eigen::Vector3f::Zero();
};
using MotionFrames = std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>;

class ImuConditioner {
 public:
  Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
  Eigen::Vector3f lever = Eigen::Vector3f::Zero();
  double smoothing = .02;
  bool apply(const MotionImu &raw, MotionImu &result) {
    if (!std::isfinite(raw.stamp) || raw.stamp <= previous_stamp_ ||
        !raw.ang_vel.allFinite() || !raw.lin_accel.allFinite()) return false;
    result = raw;
    result.ang_vel = rotation * raw.ang_vel;
    const double dt = raw.stamp - previous_stamp_;
    Eigen::Vector3f alpha = Eigen::Vector3f::Zero();
    if (previous_stamp_ > 0. && dt <= .05) {
      const Eigen::Vector3f next = filtered_omega_ + dt/(smoothing+dt)*(result.ang_vel-filtered_omega_);
      alpha = (next-filtered_omega_)/dt; filtered_omega_ = next;
    } else filtered_omega_ = result.ang_vel;
    previous_stamp_ = raw.stamp;
    result.lin_accel = rotation * raw.lin_accel - alpha.cross(lever) -
        filtered_omega_.cross(filtered_omega_.cross(lever));
    return true;
  }
 private:
  double previous_stamp_ = 0.;
  Eigen::Vector3f filtered_omega_ = Eigen::Vector3f::Zero();
};

inline Eigen::Quaternionf rotate(const Eigen::Quaternionf &q, const Eigen::Vector3f &omega, double dt) {
  const float angle = omega.norm() * dt;
  return angle > 1e-12f ? (q * Eigen::Quaternionf(Eigen::AngleAxisf(angle, omega.normalized()))).normalized() : q;
}

// Integrate from an exact sensor timestamp. Point poses are interpolated from
// the START of each IMU interval, never from the already advanced orientation.
// All integration is local to IMU intervals, so dense point sampling does not
// accumulate tens of thousands of float quaternion updates per scan.
inline MotionFrames integrateMotion(double start, Eigen::Quaternionf q,
    Eigen::Vector3f p, Eigen::Vector3f v, const std::vector<double> &targets,
    const std::vector<MotionImu> &imu, double gravity) {
  MotionFrames frames;
  if (targets.empty() || imu.size() < 2 || start > targets.front() ||
      imu.front().stamp > start || imu.back().stamp < targets.back() ||
      !std::is_sorted(targets.begin(), targets.end())) return frames;
  for (double t : targets) if (!std::isfinite(t)) return {};
  std::size_t k = 0, target = 0;
  while (k + 1 < imu.size() && imu[k+1].stamp <= start) ++k;
  double current = start;
  frames.reserve(targets.size());
  const auto pose = [](const Eigen::Quaternionf &orientation, const Eigen::Vector3f &position) {
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3,3>(0,0) = orientation.toRotationMatrix(); T.block<3,1>(0,3) = position;
    return T;
  };
  while (target < targets.size() && targets[target] == current) {
    frames.push_back(pose(q,p)); ++target;
  }
  for (; k + 1 < imu.size() && target < targets.size(); ++k) {
    const auto &a = imu[k]; const auto &b = imu[k+1];
    const double sample_dt = b.stamp - a.stamp;
    if (!(sample_dt > 0.) || sample_dt > .05 ||
        !a.ang_vel.allFinite() || !b.ang_vel.allFinite() ||
        !a.lin_accel.allFinite() || !b.lin_accel.allFinite()) return {};
    const float f = (current - a.stamp) / sample_dt;
    const Eigen::Vector3f omega0 = (1.f-f)*a.ang_vel + f*b.ang_vel;
    const Eigen::Vector3f accel0 = (1.f-f)*a.lin_accel + f*b.lin_accel;
    const double dt = b.stamp - current;
    if (!(dt > 0.)) continue;
    const Eigen::Quaternionf next = rotate(q, .5f*(omega0+b.ang_vel), dt);
    Eigen::Vector3f world0 = q * accel0, world1 = next * b.lin_accel;
    world0.z() -= gravity; world1.z() -= gravity;
    const Eigen::Vector3f jerk = (world1-world0)/dt;
    while (target < targets.size() && targets[target] <= b.stamp) {
      const double partial = targets[target] - current;
      if (partial < 0.) return {};
      const Eigen::Vector3f omega = omega0 + .5f * (b.ang_vel-omega0) * (partial/dt);
      frames.push_back(pose(rotate(q,omega,partial),
          p + v*partial + .5*world0*partial*partial + jerk*(partial*partial*partial/6.)));
      ++target;
    }
    p += v*dt + .5*world0*dt*dt + jerk*(dt*dt*dt/6.);
    v += .5*(world0+world1)*dt;
    q = next; current = b.stamp;
  }
  return target == targets.size() ? frames : MotionFrames{};
}
}  // namespace dlio
