#include <cstdlib>
#include <iostream>
#include "dlio/motion.h"

int main() {
  constexpr double gravity = 9.80665;
  std::vector<dlio::MotionImu> imu;
  for (int i=0;i<=40;++i) {
    dlio::MotionImu s; s.stamp = 1037. + i*.005;
    s.ang_vel = {0,0,2}; s.lin_accel = {0,0,gravity}; imu.push_back(s);
  }
  // Non-grid start and queries expose the original extra full-IMU-step rotation.
  const double start = 1037.002;
  std::vector<double> times{start, 1037.003, 1037.01, 1037.075, 1037.1};
  auto frames = dlio::integrateMotion(start, Eigen::Quaternionf::Identity(),
      Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), times, imu, gravity);
  if (frames.size() != times.size()) return EXIT_FAILURE;
  for (std::size_t i=0;i<frames.size();++i) {
    Eigen::Quaternionf expected(Eigen::AngleAxisf(2*(times[i]-start), Eigen::Vector3f::UnitZ()));
    if (Eigen::Quaternionf(frames[i].block<3,3>(0,0)).angularDistance(expected)>2e-6f ||
        frames[i].block<3,1>(0,3).norm()>1e-6f) return EXIT_FAILURE;
  }
  const Eigen::Quaternionf tilt(Eigen::AngleAxisf(.3f, Eigen::Vector3f::UnitX()));
  auto tilted = imu;
  for (auto &s : tilted) {
    s.ang_vel = tilt.conjugate()*Eigen::Vector3f(0,0,2);
    s.lin_accel = tilt.conjugate()*Eigen::Vector3f(0,0,gravity);
  }
  frames = dlio::integrateMotion(start, tilt, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), times, tilted, gravity);
  const Eigen::Quaternionf tilted_expected = Eigen::Quaternionf(
      Eigen::AngleAxisf(2*(times.back()-start), Eigen::Vector3f::UnitZ()))*tilt;
  if (frames.size()!=times.size() || frames.back().block<3,1>(0,3).norm()>1e-6f ||
      Eigen::Quaternionf(frames.back().block<3,3>(0,0)).angularDistance(tilted_expected)>2e-6f) return EXIT_FAILURE;
  for (auto &s : imu) {s.ang_vel.setZero();s.lin_accel.x()=1.;}
  frames = dlio::integrateMotion(start, Eigen::Quaternionf::Identity(),
      Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), times, imu, gravity);
  if (std::abs(frames.back()(0,3)-.5*.098*.098)>1e-6) return EXIT_FAILURE;
  auto missing = imu; missing.erase(missing.begin()+3, missing.begin()+20);
  if (!dlio::integrateMotion(start, Eigen::Quaternionf::Identity(), Eigen::Vector3f::Zero(),
      Eigen::Vector3f::Zero(), times, missing, gravity).empty()) return EXIT_FAILURE;
  std::reverse(times.begin(),times.end());
  if (!dlio::integrateMotion(start, Eigen::Quaternionf::Identity(), Eigen::Vector3f::Zero(),
      Eigen::Vector3f::Zero(), times, imu, gravity).empty()) return EXIT_FAILURE;
  dlio::ImuConditioner conditioner;
  conditioner.rotation = Eigen::AngleAxisf(.3, Eigen::Vector3f::UnitX()).toRotationMatrix();
  conditioner.lever = {.1,-.2,0};
  dlio::MotionImu raw, corrected;
  raw.stamp = 1037.; const Eigen::Vector3f w(0,0,2);
  raw.ang_vel = conditioner.rotation.transpose()*w;
  raw.lin_accel = conditioner.rotation.transpose()*(Eigen::Vector3f(0,0,gravity)+w.cross(w.cross(conditioner.lever)));
  if (!conditioner.apply(raw,corrected) || (corrected.lin_accel-Eigen::Vector3f(0,0,gravity)).norm()>2e-6 ||
      conditioner.apply(raw,corrected)) return EXIT_FAILURE;
  std::cout<<"DLIO motion: yaw interpolation, gravity, translation, missing IMU, timestamps and lever compensation passed\n";
}
