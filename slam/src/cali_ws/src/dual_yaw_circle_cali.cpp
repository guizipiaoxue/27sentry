#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

class DualYawCircleCali final : public rclcpp::Node {
 public:
  using Pose = geometry_msgs::msg::PoseStamped;

  DualYawCircleCali() : Node("dual_yaw_circle_cali") {
    pose_topics_[0] = declare_parameter<std::string>(
        "lidar5_pose_topic", "/cali/lidar5/pose");
    pose_topics_[1] = declare_parameter<std::string>(
        "lidar3_pose_topic", "/cali/lidar3/pose");
    output_files_[0] = declare_parameter<std::string>(
        "lidar5_output_file", "gimbal_lidar_5.yaml");
    output_files_[1] = declare_parameter<std::string>(
        "lidar3_output_file", "gimbal_lidar_3.yaml");
    minimum_samples_ = declare_parameter<int>("minimum_samples", 180);
    maximum_samples_ = declare_parameter<int>("maximum_samples", 6000);
    minimum_direction_degrees_ =
        declare_parameter<double>("minimum_direction_degrees", 270.0);
    minimum_fit_angle_degrees_ =
        declare_parameter<double>("minimum_fit_angle_degrees", 5.0);
    maximum_circle_residual_ =
        declare_parameter<double>("maximum_circle_residual", 0.01);
    maximum_radius_error_ =
        declare_parameter<double>("maximum_radius_error", 0.03);
    minimum_axis_dominance_ =
        declare_parameter<double>("minimum_axis_dominance", 0.95);
    maximum_motion_mismatch_ratio_ =
        declare_parameter<double>("maximum_motion_mismatch_ratio", 0.25);
    maximum_position_jump_ =
        declare_parameter<double>("maximum_position_jump", 0.35);
    maximum_angular_jump_ =
        declare_parameter<double>("maximum_angular_jump", 0.50);
    maximum_linear_speed_ =
        declare_parameter<double>("maximum_linear_speed", 3.0);
    maximum_angular_speed_ =
        declare_parameter<double>("maximum_angular_speed", 3.0);
    convergence_rotation_degrees_ =
        declare_parameter<double>("convergence_rotation_degrees", 0.10);
    convergence_reports_ = declare_parameter<int>("convergence_reports", 5);
    equal_height_ = declare_parameter<bool>("equal_height", true);
    auto_exit_ = declare_parameter<bool>("auto_exit", true);

    if (minimum_samples_ < 20 || maximum_samples_ < minimum_samples_ ||
        minimum_direction_degrees_ <= 0.0 || maximum_circle_residual_ <= 0.0 ||
        maximum_radius_error_ <= 0.0 || minimum_axis_dominance_ <= 0.0 ||
        minimum_axis_dominance_ > 1.0 || convergence_reports_ < 1) {
      throw std::runtime_error("invalid yaw calibration parameters");
    }

    for (std::size_t index = 0; index < 2; ++index) {
      configurations_[index] = YAML::LoadFile(output_files_[index]);
      initial_rotations_[index] = readRotation(configurations_[index]);
      translations_[index] = readTranslation(configurations_[index]);
      if (translations_[index].head<2>().norm() < 0.02) {
        throw std::runtime_error(
            "configured lidar XY translation is too small for yaw-circle calibration");
      }
    }

    const auto qos = rclcpp::SensorDataQoS();
    for (std::size_t index = 0; index < 2; ++index) {
      subscriptions_[index] = create_subscription<Pose>(
          pose_topics_[index], qos,
          [this, index](Pose::ConstSharedPtr message) {
            poseCallback(index, *message);
          });
    }
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { report(); });

    RCLCPP_INFO(get_logger(), "yaw-only dual-LiDAR calibration started");
    RCLCPP_INFO(get_logger(), "poses: %s and %s", pose_topics_[0].c_str(),
                pose_topics_[1].c_str());
    RCLCPP_INFO(get_logger(),
                "keep the chassis fixed; rotate yaw smoothly in both directions "
                "(at least %.0f deg each way)",
                minimum_direction_degrees_);
    RCLCPP_INFO(get_logger(),
                "configured XY translations are fixed and Z is unobservable");
  }

 private:
  struct Sample {
    double stamp;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d position;
  };

  struct FitResult {
    bool valid = false;
    std::string reason = "waiting for poses";
    std::size_t samples = 0;
    std::size_t inliers = 0;
    double positive_degrees = 0.0;
    double negative_degrees = 0.0;
    double axis_dominance = 0.0;
    double residual = std::numeric_limits<double>::infinity();
    double radius = 0.0;
    Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  };

  static constexpr double kPi = 3.14159265358979323846;

  static Eigen::Matrix3d readRotation(const YAML::Node &configuration) {
    const YAML::Node value = configuration["lidar_to_gimbal_rotation"];
    if (!value || !value.IsSequence() || value.size() != 3) {
      throw std::runtime_error("missing lidar_to_gimbal_rotation in YAML");
    }
    Eigen::Matrix3d rotation;
    for (int row = 0; row < 3; ++row) {
      if (!value[row].IsSequence() || value[row].size() != 3) {
        throw std::runtime_error("invalid lidar_to_gimbal_rotation in YAML");
      }
      for (int column = 0; column < 3; ++column) {
        rotation(row, column) = value[row][column].as<double>();
      }
    }
    if (!rotation.allFinite() ||
        std::abs(rotation.determinant() - 1.0) > 1e-3) {
      throw std::runtime_error("non-rigid lidar_to_gimbal_rotation in YAML");
    }
    return rotation;
  }

  static Eigen::Vector3d readTranslation(const YAML::Node &configuration) {
    const YAML::Node value = configuration["lidar_to_gimbal_translation"];
    if (!value || !value.IsSequence() || value.size() != 3) {
      throw std::runtime_error("missing lidar_to_gimbal_translation in YAML");
    }
    const Eigen::Vector3d translation(value[0].as<double>(), value[1].as<double>(),
                                      value[2].as<double>());
    if (!translation.allFinite()) {
      throw std::runtime_error("invalid lidar_to_gimbal_translation in YAML");
    }
    return translation;
  }

  static double rotationDistance(const Eigen::Matrix3d &first,
                                 const Eigen::Matrix3d &second) {
    return Eigen::AngleAxisd(first.transpose() * second).angle();
  }

  void poseCallback(std::size_t index, const Pose &message) {
    Eigen::Quaterniond orientation(
        message.pose.orientation.w, message.pose.orientation.x,
        message.pose.orientation.y, message.pose.orientation.z);
    const Eigen::Vector3d position(message.pose.position.x,
                                   message.pose.position.y,
                                   message.pose.position.z);
    if (!position.allFinite() || !orientation.coeffs().allFinite() ||
        orientation.norm() < 1e-9) {
      return;
    }
    orientation.normalize();
    const Sample sample{
        rclcpp::Time(message.header.stamp).seconds(), orientation, position};

    std::lock_guard<std::mutex> lock(mutex_);
    if (completed_) {
      return;
    }
    auto &poses = poses_[index];
    if (!poses.empty()) {
      const Sample &previous = poses.back();
      const double dt = sample.stamp - previous.stamp;
      if (dt <= 0.0) {
        return;
      }
      const double position_change =
          (sample.position - previous.position).norm();
      Eigen::Quaterniond delta =
          previous.orientation.inverse() * sample.orientation;
      if (delta.w() < 0.0) {
        delta.coeffs() = -delta.coeffs();
      }
      const double angle_change = Eigen::AngleAxisd(delta).angle();
      const bool normal_interval = dt < 0.25;
      if (position_change > maximum_position_jump_ ||
          angle_change > maximum_angular_jump_ ||
          (normal_interval &&
           (position_change / dt > maximum_linear_speed_ ||
            angle_change / dt > maximum_angular_speed_))) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "lidar%zu rejected discontinuous pose: dp=%.3f m da=%.2f deg dt=%.3f s",
            index == 0 ? 5UL : 3UL, position_change,
            angle_change * 180.0 / kPi, dt);
        return;
      }
    }
    poses.push_back(sample);
    if (poses.size() > static_cast<std::size_t>(maximum_samples_)) {
      poses.erase(poses.begin(),
                  poses.begin() + (poses.size() - maximum_samples_));
    }
  }

  FitResult fit(std::size_t index, const std::vector<Sample> &poses) const {
    FitResult result;
    result.samples = poses.size();
    if (poses.size() < static_cast<std::size_t>(minimum_samples_)) {
      result.reason = "need more pose samples";
      return result;
    }

    Eigen::Matrix3d axis_covariance = Eigen::Matrix3d::Zero();
    std::vector<Eigen::Vector3d> increments;
    increments.reserve(poses.size() - 1);
    for (std::size_t i = 1; i < poses.size(); ++i) {
      Eigen::Quaterniond delta =
          poses[i - 1].orientation.inverse() * poses[i].orientation;
      delta.normalize();
      if (delta.w() < 0.0) {
        delta.coeffs() = -delta.coeffs();
      }
      const Eigen::AngleAxisd angle_axis(delta);
      if (angle_axis.angle() < 1e-5) {
        continue;
      }
      const Eigen::Vector3d vector = angle_axis.axis() * angle_axis.angle();
      increments.push_back(vector);
      axis_covariance += vector * vector.transpose();
    }
    if (increments.size() < 20) {
      result.reason = "need more yaw motion";
      return result;
    }

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(axis_covariance);
    if (eigen.info() != Eigen::Success ||
        eigen.eigenvalues().z() <= 1e-12) {
      result.reason = "rotation axis is not observable";
      return result;
    }
    result.axis = eigen.eigenvectors().col(2).normalized();
    const Eigen::Vector3d expected_axis =
        initial_rotations_[index].transpose() * Eigen::Vector3d::UnitZ();
    if (result.axis.dot(expected_axis) < 0.0) {
      result.axis = -result.axis;
    }
    result.axis_dominance =
        eigen.eigenvalues().z() / eigen.eigenvalues().sum();
    for (const Eigen::Vector3d &increment : increments) {
      const double signed_angle = result.axis.dot(increment);
      if (signed_angle >= 0.0) {
        result.positive_degrees += signed_angle * 180.0 / kPi;
      } else {
        result.negative_degrees -= signed_angle * 180.0 / kPi;
      }
    }
    if (result.axis_dominance < minimum_axis_dominance_) {
      result.reason = "motion is not a clean single yaw axis";
      return result;
    }
    if (result.positive_degrees < minimum_direction_degrees_ ||
        result.negative_degrees < minimum_direction_degrees_) {
      result.reason = "rotate farther in both yaw directions";
      return result;
    }

    const Eigen::Quaterniond first_orientation = poses.front().orientation;
    const Eigen::Vector3d first_position = poses.front().position;
    std::vector<Eigen::Matrix3d> relative_rotations;
    std::vector<Eigen::Vector3d> relative_positions;
    relative_rotations.reserve(poses.size());
    relative_positions.reserve(poses.size());
    const double minimum_fit_angle = minimum_fit_angle_degrees_ * kPi / 180.0;
    for (const Sample &pose : poses) {
      const Eigen::Matrix3d rotation =
          (first_orientation.inverse() * pose.orientation).toRotationMatrix();
      const double angle = Eigen::AngleAxisd(rotation).angle();
      if (angle < minimum_fit_angle) {
        continue;
      }
      relative_rotations.push_back(rotation);
      relative_positions.push_back(
          first_orientation.inverse() * (pose.position - first_position));
    }
    if (relative_rotations.size() < static_cast<std::size_t>(minimum_samples_ / 2)) {
      result.reason = "not enough informative circle samples";
      return result;
    }

    std::vector<bool> inlier(relative_rotations.size(), true);
    for (int iteration = 0; iteration < 5; ++iteration) {
      Eigen::Matrix3d hessian = Eigen::Matrix3d::Zero();
      Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
      std::size_t used = 0;
      for (std::size_t i = 0; i < relative_rotations.size(); ++i) {
        if (!inlier[i]) {
          continue;
        }
        const Eigen::Matrix3d a =
            Eigen::Matrix3d::Identity() - relative_rotations[i];
        hessian += a.transpose() * a;
        gradient += a.transpose() * relative_positions[i];
        ++used;
      }
      if (used < static_cast<std::size_t>(minimum_samples_ / 2)) {
        result.reason = "too many circle-fit outliers";
        return result;
      }
      // Translation along the yaw axis is unobservable.  The minimum-norm
      // center is selected by constraining its component along that axis.
      hessian += static_cast<double>(used) *
                 result.axis * result.axis.transpose();
      result.center = hessian.ldlt().solve(gradient);
      if (!result.center.allFinite()) {
        result.reason = "circle fit is singular";
        return result;
      }

      std::vector<double> errors;
      errors.reserve(used);
      for (std::size_t i = 0; i < relative_rotations.size(); ++i) {
        if (inlier[i]) {
          errors.push_back((relative_positions[i] -
                            (result.center -
                             relative_rotations[i] * result.center)).norm());
        }
      }
      std::vector<double> sorted = errors;
      std::sort(sorted.begin(), sorted.end());
      const double median = sorted[sorted.size() / 2];
      for (double &value : sorted) {
        value = std::abs(value - median);
      }
      std::sort(sorted.begin(), sorted.end());
      const double mad = sorted[sorted.size() / 2];
      const double cutoff = std::max(2.0 * maximum_circle_residual_,
                                     median + 4.0 * 1.4826 * mad);
      bool changed = false;
      for (std::size_t i = 0; i < relative_rotations.size(); ++i) {
        if (!inlier[i]) {
          continue;
        }
        const double error =
            (relative_positions[i] -
             (result.center - relative_rotations[i] * result.center)).norm();
        if (error > cutoff) {
          inlier[i] = false;
          changed = true;
        }
      }
      if (!changed) {
        break;
      }
    }

    double squared_error = 0.0;
    for (std::size_t i = 0; i < relative_rotations.size(); ++i) {
      if (!inlier[i]) {
        continue;
      }
      const Eigen::Vector3d error =
          relative_positions[i] -
          (result.center - relative_rotations[i] * result.center);
      squared_error += error.squaredNorm();
      ++result.inliers;
    }
    if (result.inliers < static_cast<std::size_t>(minimum_samples_ / 2)) {
      result.reason = "too few circle-fit inliers";
      return result;
    }
    result.residual = std::sqrt(squared_error / result.inliers);
    result.center -= result.axis * result.axis.dot(result.center);
    result.radius = result.center.norm();
    const double expected_radius = translations_[index].head<2>().norm();
    if (result.residual > maximum_circle_residual_) {
      result.reason = "circle residual is too high";
      return result;
    }
    if (std::abs(result.radius - expected_radius) > maximum_radius_error_) {
      result.reason = "fitted radius disagrees with configured XY offset";
      return result;
    }
    if (result.radius < 1e-6) {
      result.reason = "fitted circle radius is zero";
      return result;
    }

    const Eigen::Vector3d source_x = result.center.normalized();
    const Eigen::Vector3d source_z = result.axis;
    const Eigen::Vector3d source_y = source_z.cross(source_x).normalized();
    const Eigen::Vector3d target_z = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d target_x(-translations_[index].x(),
                             -translations_[index].y(), 0.0);
    target_x.normalize();
    const Eigen::Vector3d target_y = target_z.cross(target_x).normalized();
    Eigen::Matrix3d source_basis;
    source_basis.col(0) = source_x;
    source_basis.col(1) = source_y;
    source_basis.col(2) = source_z;
    Eigen::Matrix3d target_basis;
    target_basis.col(0) = target_x;
    target_basis.col(1) = target_y;
    target_basis.col(2) = target_z;
    result.rotation = target_basis * source_basis.transpose();
    result.valid = true;
    result.reason = "fit passed";
    return result;
  }

  static YAML::Node vectorNode(const Eigen::Vector3d &value) {
    YAML::Node node(YAML::NodeType::Sequence);
    node.push_back(value.x());
    node.push_back(value.y());
    node.push_back(value.z());
    node.SetStyle(YAML::EmitterStyle::Flow);
    return node;
  }

  static YAML::Node matrixNode(const Eigen::Matrix3d &value) {
    YAML::Node matrix(YAML::NodeType::Sequence);
    for (int row = 0; row < 3; ++row) {
      YAML::Node line(YAML::NodeType::Sequence);
      for (int column = 0; column < 3; ++column) {
        line.push_back(value(row, column));
      }
      line.SetStyle(YAML::EmitterStyle::Flow);
      matrix.push_back(line);
    }
    return matrix;
  }

  std::string renderConfiguration(std::size_t index, const FitResult &fit,
                                  double shared_height) const {
    YAML::Node configuration = YAML::Clone(configurations_[index]);
    Eigen::Vector3d translation = translations_[index];
    if (equal_height_) {
      translation.z() = shared_height;
    }
    configuration["calibration_method"] = "dual_dlio_yaw_circle";
    configuration["circle_center"] = vectorNode(fit.center);
    configuration["circle_residual"] = fit.residual;
    configuration["circle_radius"] = fit.radius;
    configuration["samples"] = static_cast<unsigned long long>(fit.inliers);
    configuration["yaw_positive_degrees"] = fit.positive_degrees;
    configuration["yaw_negative_degrees"] = fit.negative_degrees;
    configuration["axis_dominance"] = fit.axis_dominance;
    configuration["yaw_axis_lidar"] = vectorNode(fit.axis);
    configuration["lidar_to_gimbal_rotation"] = matrixNode(fit.rotation);
    configuration["lidar_to_gimbal_translation"] = vectorNode(translation);
    YAML::Emitter emitter;
    emitter.SetDoublePrecision(10);
    emitter << configuration;
    if (!emitter.good()) {
      throw std::runtime_error("failed to serialize calibration YAML");
    }
    return std::string(emitter.c_str()) + "\n";
  }

  static void writeFile(const std::string &path, const std::string &contents) {
    std::ofstream stream(path, std::ios::binary | std::ios::trunc);
    if (!stream) {
      throw std::runtime_error("cannot open " + path);
    }
    stream << contents;
    stream.close();
    if (!stream) {
      throw std::runtime_error("cannot write " + path);
    }
  }

  static std::string readFile(const std::string &path) {
    std::ifstream stream(path, std::ios::binary);
    if (!stream) {
      throw std::runtime_error("cannot read " + path);
    }
    std::ostringstream contents;
    contents << stream.rdbuf();
    if (!stream.good() && !stream.eof()) {
      throw std::runtime_error("cannot read " + path);
    }
    return contents.str();
  }

  void writePair(const std::array<FitResult, 2> &fits) {
    const double shared_height =
        0.5 * (translations_[0].z() + translations_[1].z());
    const std::array<std::string, 2> contents = {
        renderConfiguration(0, fits[0], shared_height),
        renderConfiguration(1, fits[1], shared_height)};
    const std::array<std::string, 2> temporary = {
        output_files_[0] + ".yaw-cali.tmp",
        output_files_[1] + ".yaw-cali.tmp"};
    const std::array<std::string, 2> original = {
        readFile(output_files_[0]), readFile(output_files_[1])};
    bool replaced_first = false;
    try {
      writeFile(temporary[0], contents[0]);
      writeFile(temporary[1], contents[1]);
      if (std::rename(temporary[0].c_str(), output_files_[0].c_str()) != 0) {
        throw std::runtime_error("cannot replace " + output_files_[0]);
      }
      replaced_first = true;
      if (std::rename(temporary[1].c_str(), output_files_[1].c_str()) != 0) {
        throw std::runtime_error("cannot replace " + output_files_[1]);
      }
    } catch (...) {
      std::remove(temporary[0].c_str());
      std::remove(temporary[1].c_str());
      if (replaced_first) {
        try {
          writeFile(output_files_[0], original[0]);
        } catch (...) {
          RCLCPP_FATAL(get_logger(),
                       "failed to restore %s after pair-write failure",
                       output_files_[0].c_str());
        }
      }
      throw;
    }
  }

  void report() {
    std::array<std::vector<Sample>, 2> poses;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (completed_) {
        return;
      }
      poses = poses_;
    }
    const std::array<FitResult, 2> fits = {fit(0, poses[0]), fit(1, poses[1])};
    for (std::size_t index = 0; index < 2; ++index) {
      const FitResult &value = fits[index];
      RCLCPP_INFO(
          get_logger(),
          "lidar%zu: poses=%zu yaw +%.0f/-%.0f deg axis=%.3f "
          "radius=%.4f m residual=%.4f m [%s]",
          index == 0 ? 5UL : 3UL, value.samples, value.positive_degrees,
          value.negative_degrees, value.axis_dominance, value.radius,
          value.residual, value.reason.c_str());
    }
    if (!fits[0].valid || !fits[1].valid) {
      stable_reports_ = 0;
      return;
    }
    const double motion0 = fits[0].positive_degrees + fits[0].negative_degrees;
    const double motion1 = fits[1].positive_degrees + fits[1].negative_degrees;
    const double motion_mismatch =
        std::abs(motion0 - motion1) / std::max(motion0, motion1);
    if (motion_mismatch > maximum_motion_mismatch_ratio_) {
      stable_reports_ = 0;
      RCLCPP_WARN(get_logger(),
                  "two DLIO trajectories disagree on yaw travel (%.1f vs %.1f "
                  "deg); keep the chassis fixed and check both odometries",
                  motion0, motion1);
      return;
    }

    if (have_previous_fit_) {
      const double change0 =
          rotationDistance(previous_rotations_[0], fits[0].rotation);
      const double change1 =
          rotationDistance(previous_rotations_[1], fits[1].rotation);
      const double limit = convergence_rotation_degrees_ * kPi / 180.0;
      stable_reports_ = (change0 <= limit && change1 <= limit)
                            ? stable_reports_ + 1
                            : 1;
    } else {
      stable_reports_ = 1;
      have_previous_fit_ = true;
    }
    previous_rotations_[0] = fits[0].rotation;
    previous_rotations_[1] = fits[1].rotation;
    RCLCPP_INFO(get_logger(), "valid solution stability: %d/%d reports",
                stable_reports_, convergence_reports_);
    if (stable_reports_ < convergence_reports_) {
      return;
    }

    try {
      writePair(fits);
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "failed to save calibration: %s", error.what());
      stable_reports_ = 0;
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      completed_ = true;
    }
    RCLCPP_INFO(get_logger(), "saved lidar5 calibration: %s",
                output_files_[0].c_str());
    RCLCPP_INFO(get_logger(), "saved lidar3 calibration: %s",
                output_files_[1].c_str());
    if (auto_exit_) {
      rclcpp::shutdown();
    }
  }

  std::array<std::string, 2> pose_topics_;
  std::array<std::string, 2> output_files_;
  std::array<YAML::Node, 2> configurations_;
  std::array<Eigen::Matrix3d, 2> initial_rotations_;
  std::array<Eigen::Vector3d, 2> translations_;
  std::array<std::vector<Sample>, 2> poses_;
  std::array<rclcpp::Subscription<Pose>::SharedPtr, 2> subscriptions_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mutex_;
  int minimum_samples_;
  int maximum_samples_;
  double minimum_direction_degrees_;
  double minimum_fit_angle_degrees_;
  double maximum_circle_residual_;
  double maximum_radius_error_;
  double minimum_axis_dominance_;
  double maximum_motion_mismatch_ratio_;
  double maximum_position_jump_;
  double maximum_angular_jump_;
  double maximum_linear_speed_;
  double maximum_angular_speed_;
  double convergence_rotation_degrees_;
  int convergence_reports_;
  bool equal_height_;
  bool auto_exit_;
  bool completed_ = false;
  bool have_previous_fit_ = false;
  int stable_reports_ = 0;
  std::array<Eigen::Matrix3d, 2> previous_rotations_ = {
      Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity()};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<DualYawCircleCali>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("dual_yaw_circle_cali"), "%s",
                 error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
