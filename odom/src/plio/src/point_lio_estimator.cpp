#include "plio/point_lio_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <utility>

#include <pcl/filters/voxel_grid.h>

#include "plio/point_lio_core.hpp"

namespace plio::core {

Eigen::Matrix<double, 24, 1> processModel(State &state, const Input &input) {
  Eigen::Matrix<double, 24, 1> result = Eigen::Matrix<double, 24, 1>::Zero();
  Vect3 omega;
  input.gyro.boxminus(omega, state.bg);
  const Vect3 acceleration = state.rot * (input.acc - state.ba);
  for (int i = 0; i < 3; ++i) {
    result(i) = state.vel[i];
    result(i + 3) = omega[i];
    result(i + 12) = acceleration[i] + state.gravity[i];
  }
  return result;
}

Eigen::Matrix<double, 24, 24> processJacobian(
    State &state, const Input &input) {
  Eigen::Matrix<double, 24, 24> jacobian =
      Eigen::Matrix<double, 24, 24>::Zero();
  jacobian.block<3, 3>(0, 12).setIdentity();
  Vect3 acceleration;
  input.acc.boxminus(acceleration, state.ba);
  jacobian.block<3, 3>(12, 3) = -state.rot * MTK::hat(acceleration);
  jacobian.block<3, 3>(12, 18) = -state.rot;
  jacobian.block<3, 3>(12, 21).setIdentity();
  jacobian.block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity();
  return jacobian;
}

Eigen::Matrix<double, 24, 24> processNoise(
    double gyro, double accel, double gyro_bias, double accel_bias) {
  Eigen::Matrix<double, 24, 24> noise =
      Eigen::Matrix<double, 24, 24>::Zero();
  noise.block<3, 3>(3, 3).diagonal().setConstant(gyro);
  noise.block<3, 3>(12, 12).diagonal().setConstant(accel);
  noise.block<3, 3>(15, 15).diagonal().setConstant(gyro_bias);
  noise.block<3, 3>(18, 18).diagonal().setConstant(accel_bias);
  return noise;
}

}  // namespace plio::core

namespace plio {
namespace {

constexpr int kNeighbors = 5;

struct MeasurementContext {
  const Cloud *body = nullptr;
  Cloud *world = nullptr;
  std::vector<core::PointVector> *nearest = nullptr;
  core::IVox *map = nullptr;
  Eigen::Matrix3d lidar_rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d lidar_translation = Eigen::Vector3d::Zero();
  double plane_threshold = 0.10;
  double match_scale = 81.0;
  double covariance = 0.10;
  bool estimate_extrinsics = false;
  std::size_t begin = 0;
  std::size_t end = 0;
};

thread_local MeasurementContext *measurement_context = nullptr;

Eigen::Vector3d pointInImu(const core::State &state, const Point &point) {
  const Eigen::Vector3d lidar(point.x, point.y, point.z);
  return state.offset_R_L_I * lidar + state.offset_T_L_I;
}

void transformPoint(const core::State &state, const Point &input, Point &output) {
  const Eigen::Vector3d global = state.rot * pointInImu(state, input) + state.pos;
  output = input;
  output.x = static_cast<float>(global.x());
  output.y = static_cast<float>(global.y());
  output.z = static_cast<float>(global.z());
}

bool fitPlane(
    Eigen::Vector4d &plane, const core::PointVector &points,
    double threshold) {
  if (points.size() < kNeighbors) {
    return false;
  }
  Eigen::Matrix<double, kNeighbors, 3> coordinates;
  coordinates.setZero();
  for (int i = 0; i < kNeighbors; ++i) {
    coordinates.row(i) << points[i].x, points[i].y, points[i].z;
  }
  const Eigen::Matrix<double, kNeighbors, 1> rhs =
      -Eigen::Matrix<double, kNeighbors, 1>::Ones();
  Eigen::Vector3d normal = coordinates.colPivHouseholderQr().solve(rhs);
  if (!normal.allFinite() || normal.norm() < 1.0e-8) {
    return false;
  }
  const double norm = normal.norm();
  plane.head<3>() = normal / norm;
  plane.w() = 1.0 / norm;
  for (int i = 0; i < kNeighbors; ++i) {
    const double distance = std::abs(
        plane.x() * points[i].x + plane.y() * points[i].y +
        plane.z() * points[i].z + plane.w());
    if (distance > threshold) {
      return false;
    }
  }
  return true;
}

void lidarMeasurement(
    core::State &state, Eigen::Matrix3d, Eigen::Matrix3d,
    esekfom::dyn_share_modified<double> &data) {
  if (measurement_context == nullptr || measurement_context->body == nullptr ||
      measurement_context->map == nullptr) {
    data.valid = false;
    return;
  }

  struct Match {
    Eigen::Vector3d normal;
    Eigen::Vector3d body;
    double residual;
  };
  std::vector<Match> matches;
  matches.reserve(measurement_context->end - measurement_context->begin);

  for (std::size_t i = measurement_context->begin;
       i < measurement_context->end; ++i) {
    const Point &body_point = measurement_context->body->points[i];
    Point &world_point = measurement_context->world->points[i];
    transformPoint(state, body_point, world_point);
    auto &nearby = (*measurement_context->nearest)[i];
    measurement_context->map->GetClosestPoint(
        world_point, nearby, kNeighbors);
    Eigen::Vector4d plane;
    if (!fitPlane(plane, nearby, measurement_context->plane_threshold)) {
      continue;
    }
    const double residual = plane.head<3>().dot(
        Eigen::Vector3d(world_point.x, world_point.y, world_point.z)) +
        plane.w();
    const Eigen::Vector3d body(body_point.x, body_point.y, body_point.z);
    if (body.norm() > measurement_context->match_scale * residual * residual) {
      matches.push_back({plane.head<3>(), body, residual});
    }
  }

  if (matches.empty()) {
    data.valid = false;
    return;
  }
  data.M_Noise = measurement_context->covariance;
  data.h_x = Eigen::MatrixXd::Zero(matches.size(), 12);
  data.z.resize(matches.size());
  for (std::size_t row = 0; row < matches.size(); ++row) {
    const Match &match = matches[row];
    const Eigen::Vector3d point_imu =
        state.offset_R_L_I * match.body + state.offset_T_L_I;
    const Eigen::Vector3d normal_body = state.rot.transpose() * match.normal;
    const Eigen::Vector3d attitude = point_imu.cross(normal_body);
    data.h_x.block<1, 6>(row, 0) <<
        match.normal.transpose(), attitude.transpose();
    if (measurement_context->estimate_extrinsics) {
      const Eigen::Vector3d extrinsic_rotation =
          match.body.cross(state.offset_R_L_I.transpose() * normal_body);
      data.h_x.block<1, 6>(row, 6) <<
          extrinsic_rotation.transpose(), normal_body.transpose();
    }
    data.z(row) = -match.residual;
  }
}

double pointOffset(const Point &point) {
  return static_cast<double>(point.curvature) * 1.0e-3;
}

}  // namespace

class PointLioEstimator::Impl {
 public:
  explicit Impl(Parameters parameters) : parameters_(std::move(parameters)) {
    reset();
  }

  void reset() {
    imu_.clear();
    imu_for_initialization_.clear();
    initialized_ = false;
    map_initialized_ = false;
    last_prediction_time_ = -1.0;
    filter_ = core::Filter();
    filter_.init_dyn_share_modified_2h(
        core::processModel, core::processJacobian, lidarMeasurement);
    Eigen::Matrix<double, 24, 24> covariance =
        Eigen::Matrix<double, 24, 24>::Identity() * 0.1;
    covariance.block<3, 3>(21, 21) *= 1.0e-3;
    covariance.block<6, 6>(15, 15) *= 1.0e-2;
    filter_.change_P(covariance);
    filter_.x_.offset_R_L_I = parameters_.lidar_to_imu_rotation;
    filter_.x_.offset_T_L_I = parameters_.lidar_to_imu_translation;
    filter_.x_.gravity = parameters_.gravity;

    core::IVox::Options options;
    options.resolution_ = static_cast<float>(parameters_.map_resolution);
    if (parameters_.nearby_type == 0) {
      options.nearby_type_ = core::IVox::NearbyType::CENTER;
    } else if (parameters_.nearby_type == 6) {
      options.nearby_type_ = core::IVox::NearbyType::NEARBY6;
    } else if (parameters_.nearby_type == 26) {
      options.nearby_type_ = core::IVox::NearbyType::NEARBY26;
    } else {
      options.nearby_type_ = core::IVox::NearbyType::NEARBY18;
    }
    map_ = std::make_unique<core::IVox>(options);
    voxel_.setLeafSize(
        parameters_.surface_leaf_size, parameters_.surface_leaf_size,
        parameters_.surface_leaf_size);
    noise_ = core::processNoise(
        parameters_.gyro_covariance, parameters_.accel_covariance,
        parameters_.gyro_bias_covariance, parameters_.accel_bias_covariance);
  }

  void addImu(const ImuSample &sample) {
    if (!std::isfinite(sample.stamp) || !sample.acceleration.allFinite() ||
        !sample.angular_velocity.allFinite()) {
      return;
    }
    if (!imu_.empty() && sample.stamp <= imu_.back().stamp) {
      imu_.clear();
      last_prediction_time_ = -1.0;
    }
    imu_.push_back(sample);
    if (!initialized_) {
      imu_for_initialization_.push_back(sample);
      while (imu_for_initialization_.size() >
             parameters_.initialization_samples) {
        imu_for_initialization_.pop_front();
      }
      initializeImu();
    }
  }

  Result process(const Cloud::ConstPtr &input, double stamp) {
    Result result;
    result.stamp = stamp;
    if (!initialized_) {
      result.waiting_for_imu = true;
      return result;
    }
    if (input == nullptr || input->empty()) {
      return result;
    }

    Cloud::Ptr valid(new Cloud);
    valid->reserve(input->size() / parameters_.point_filter + 1);
    const double minimum_range_squared = parameters_.blind * parameters_.blind;
    const double maximum_range_squared =
        parameters_.detection_range * parameters_.detection_range;
    const std::size_t stride = std::max<std::size_t>(1, parameters_.point_filter);
    for (std::size_t i = 0; i < input->size(); i += stride) {
      const Point &point = input->points[i];
      const double range_squared =
          point.x * point.x + point.y * point.y + point.z * point.z;
      if (std::isfinite(point.x) && std::isfinite(point.y) &&
          std::isfinite(point.z) && range_squared >= minimum_range_squared &&
          range_squared <= maximum_range_squared) {
        valid->push_back(point);
      }
    }
    Cloud::Ptr filtered(new Cloud);
    voxel_.setInputCloud(valid);
    voxel_.filter(*filtered);
    if (filtered->empty()) {
      return result;
    }
    std::sort(
        filtered->points.begin(), filtered->points.end(),
        [](const Point &left, const Point &right) {
          return left.curvature < right.curvature;
        });

    const double scan_end = stamp + pointOffset(filtered->points.back());
    if (imu_.empty() || imu_.back().stamp < scan_end) {
      result.waiting_for_imu = true;
      return result;
    }

    Cloud world;
    world.resize(filtered->size());
    std::vector<core::PointVector> nearest(filtered->size());
    std::size_t begin = 0;
    while (begin < filtered->size()) {
      std::size_t end = begin + 1;
      const float time = filtered->points[begin].curvature;
      while (end < filtered->size() &&
             filtered->points[end].curvature == time) {
        ++end;
      }
      const double point_time = stamp + static_cast<double>(time) * 1.0e-3;
      propagateTo(point_time);
      if (map_initialized_) {
        MeasurementContext context;
        context.body = filtered.get();
        context.world = &world;
        context.nearest = &nearest;
        context.map = map_.get();
        context.plane_threshold = parameters_.plane_threshold;
        context.match_scale = parameters_.match_scale;
        context.covariance = parameters_.lidar_measurement_covariance;
        context.estimate_extrinsics = parameters_.estimate_extrinsics;
        context.begin = begin;
        context.end = end;
        measurement_context = &context;
        filter_.update_iterated_dyn_share_modified();
        measurement_context = nullptr;
      }
      for (std::size_t i = begin; i < end; ++i) {
        transformPoint(filter_.x_, filtered->points[i], world.points[i]);
      }
      begin = end;
    }

    world.width = static_cast<std::uint32_t>(world.size());
    world.height = 1;
    world.is_dense = true;
    if (!map_initialized_) {
      map_initialized_ = world.size() >= parameters_.initialization_points;
    }
    if (map_initialized_) {
      map_->AddPoints(world.points);
    }
    discardImuBefore(scan_end);

    result.initialized = map_initialized_;
    result.position = filter_.x_.pos;
    result.orientation = Eigen::Quaterniond(filter_.x_.rot);
    result.velocity = filter_.x_.vel;
    *result.registered = world;
    *result.body = *filtered;
    return result;
  }

  bool initialized() const { return initialized_; }

 private:
  void initializeImu() {
    if (imu_for_initialization_.size() < parameters_.initialization_samples) {
      return;
    }
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
    for (const auto &sample : imu_for_initialization_) {
      acceleration += sample.acceleration;
      gyro += sample.angular_velocity;
    }
    acceleration /= static_cast<double>(imu_for_initialization_.size());
    gyro /= static_cast<double>(imu_for_initialization_.size());
    if (acceleration.norm() < 1.0e-3) {
      return;
    }

    // The fused IMU is already calibrated into the gimbal frame.  Use the
    // stationary mean only for initial roll/pitch and residual gyro bias.
    const Eigen::Quaterniond alignment = Eigen::Quaterniond::FromTwoVectors(
        acceleration.normalized(), -parameters_.gravity.normalized());
    filter_.x_.rot = alignment.toRotationMatrix();
    filter_.x_.bg = gyro;
    filter_.x_.ba.setZero();
    initialized_ = true;
    last_prediction_time_ = imu_for_initialization_.back().stamp;
  }

  void propagateTo(double target) {
    if (last_prediction_time_ < 0.0) {
      last_prediction_time_ = target;
      return;
    }
    while (!imu_.empty() && imu_.front().stamp <= target) {
      if (imu_.front().stamp > last_prediction_time_) {
        predict(imu_.front(), imu_.front().stamp - last_prediction_time_);
        last_prediction_time_ = imu_.front().stamp;
      }
      if (imu_.size() == 1) {
        break;
      }
      imu_.pop_front();
    }
    if (!imu_.empty() && target > last_prediction_time_) {
      predict(imu_.front(), target - last_prediction_time_);
      last_prediction_time_ = target;
    }
  }

  void predict(const ImuSample &sample, double interval) {
    if (!(interval > 0.0) || interval > 0.5) {
      return;
    }
    core::Input input;
    input.acc = sample.acceleration;
    input.gyro = sample.angular_velocity;
    filter_.predict(interval, noise_, input, true, true);
  }

  void discardImuBefore(double stamp) {
    while (imu_.size() > 1 && imu_[1].stamp <= stamp) {
      imu_.pop_front();
    }
  }

  Parameters parameters_;
  core::Filter filter_;
  Eigen::Matrix<double, 24, 24> noise_;
  pcl::VoxelGrid<Point> voxel_;
  std::unique_ptr<core::IVox> map_;
  std::deque<ImuSample> imu_;
  std::deque<ImuSample> imu_for_initialization_;
  bool initialized_ = false;
  bool map_initialized_ = false;
  double last_prediction_time_ = -1.0;
};

PointLioEstimator::PointLioEstimator(Parameters parameters)
    : impl_(std::make_unique<Impl>(std::move(parameters))) {}
PointLioEstimator::~PointLioEstimator() = default;
PointLioEstimator::PointLioEstimator(PointLioEstimator &&) noexcept = default;
PointLioEstimator &PointLioEstimator::operator=(PointLioEstimator &&) noexcept =
    default;
void PointLioEstimator::addImu(const ImuSample &sample) { impl_->addImu(sample); }
Result PointLioEstimator::process(const Cloud::ConstPtr &cloud, double stamp) {
  return impl_->process(cloud, stamp);
}
void PointLioEstimator::reset() { impl_->reset(); }
bool PointLioEstimator::initialized() const { return impl_->initialized(); }

}  // namespace plio
