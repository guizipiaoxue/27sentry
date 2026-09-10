#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <nav_msgs/msg/path.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "loop_closure/msg/keyframe.hpp"
#include "loop_closure/msg/loop_constraint.hpp"

namespace
{

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

gtsam::Key poseKey(int id)
{
  return gtsam::Symbol('x', static_cast<std::uint64_t>(id));
}

gtsam::Pose3 poseFromMessage(const geometry_msgs::msg::Pose & pose)
{
  return gtsam::Pose3(
    gtsam::Rot3::Quaternion(
      pose.orientation.w, pose.orientation.x, pose.orientation.y,
      pose.orientation.z),
    gtsam::Point3(pose.position.x, pose.position.y, pose.position.z));
}

geometry_msgs::msg::Pose poseToMessage(const gtsam::Pose3 & pose)
{
  geometry_msgs::msg::Pose message;
  const auto translation = pose.translation();
  const auto rotation = pose.rotation().toQuaternion();
  message.position.x = translation.x();
  message.position.y = translation.y();
  message.position.z = translation.z();
  message.orientation.w = rotation.w();
  message.orientation.x = rotation.x();
  message.orientation.y = rotation.y();
  message.orientation.z = rotation.z();
  return message;
}

Eigen::Isometry3f poseToEigen(const gtsam::Pose3 & pose)
{
  Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
  transform.matrix() = pose.matrix().cast<float>();
  return transform;
}

gtsam::Vector6 noiseSigmas(
  double rotation_sigma_deg,
  double translation_sigma)
{
  const double rotation_sigma = rotation_sigma_deg * M_PI / 180.0;
  gtsam::Vector6 sigmas;
  sigmas << rotation_sigma, rotation_sigma, rotation_sigma,
    translation_sigma, translation_sigma, translation_sigma;
  return sigmas;
}

}  // namespace

class PoseGraphBackend final : public rclcpp::Node
{
public:
  PoseGraphBackend()
  : Node("pose_graph_backend"),
    map_to_odom_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(
        *this))
  {
    map_frame_ = declare_parameter<std::string>("backend.map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("backend.odom_frame", "odom");
    prior_translation_sigma_ =
      declare_parameter<double>("backend.prior_translation_sigma", 0.01);
    prior_rotation_sigma_deg_ =
      declare_parameter<double>("backend.prior_rotation_sigma_deg", 0.1);
    odom_translation_sigma_ =
      declare_parameter<double>("backend.odom_translation_sigma", 0.10);
    odom_rotation_sigma_deg_ =
      declare_parameter<double>("backend.odom_rotation_sigma_deg", 1.0);
    loop_translation_sigma_ =
      declare_parameter<double>("backend.loop_translation_sigma", 0.15);
    loop_rotation_sigma_deg_ =
      declare_parameter<double>("backend.loop_rotation_sigma_deg", 2.0);
    loop_huber_scale_ =
      declare_parameter<double>("backend.loop_huber_scale", 1.345);
    relinearize_threshold_ =
      declare_parameter<double>("backend.relinearize_threshold", 0.01);
    relinearize_skip_ =
      declare_parameter<int>("backend.relinearize_skip", 1);
    optimization_iterations_ =
      declare_parameter<int>("backend.optimization_iterations", 2);
    save_path_ = declare_parameter<std::string>(
      "backend.save_path", "optimized_map.pcd");

    validateParameters();

    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = relinearize_threshold_;
    parameters.relinearizeSkip = relinearize_skip_;
    isam2_ = std::make_unique<gtsam::ISAM2>(parameters);

    prior_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
      noiseSigmas(
        prior_rotation_sigma_deg_, prior_translation_sigma_));
    odometry_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
      noiseSigmas(
        odom_rotation_sigma_deg_, odom_translation_sigma_));
    const auto loop_gaussian = gtsam::noiseModel::Diagonal::Sigmas(
      noiseSigmas(
        loop_rotation_sigma_deg_, loop_translation_sigma_));
    loop_noise_ = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(loop_huber_scale_),
      loop_gaussian);

    const auto reliable_qos = rclcpp::QoS(50).reliable();
    keyframe_sub_ = create_subscription<loop_closure::msg::Keyframe>(
      "keyframe", reliable_qos,
      std::bind(
        &PoseGraphBackend::keyframeCallback, this,
        std::placeholders::_1));
    loop_sub_ = create_subscription<loop_closure::msg::LoopConstraint>(
      "loop_constraint", reliable_qos,
      std::bind(
        &PoseGraphBackend::loopCallback, this,
        std::placeholders::_1));

    optimized_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "optimized_path", rclcpp::QoS(1).reliable().transient_local());
    save_map_service_ = create_service<std_srvs::srv::Trigger>(
      "save_map", std::bind(
        &PoseGraphBackend::saveMap, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "GTSAM iSAM2 backend ready: keyframe + loop constraints -> %s map",
      map_frame_.c_str());
  }

private:
  struct KeyframeData
  {
    builtin_interfaces::msg::Time stamp;
    gtsam::Pose3 odometry_pose;
    Cloud::Ptr local_cloud;
  };

  void validateParameters() const
  {
    const bool invalid_sigma =
      prior_translation_sigma_ <= 0.0 || prior_rotation_sigma_deg_ <= 0.0 ||
      odom_translation_sigma_ <= 0.0 || odom_rotation_sigma_deg_ <= 0.0 ||
      loop_translation_sigma_ <= 0.0 || loop_rotation_sigma_deg_ <= 0.0;
    if (map_frame_.empty() || odom_frame_.empty() || invalid_sigma ||
      loop_huber_scale_ <= 0.0 || relinearize_threshold_ <= 0.0 ||
      relinearize_skip_ <= 0 || optimization_iterations_ <= 0 ||
      save_path_.empty())
    {
      throw std::invalid_argument("invalid pose graph backend parameters");
    }
  }

  Cloud::Ptr extractLocalCloud(
    const loop_closure::msg::Keyframe & message,
    const gtsam::Pose3 & odometry_pose) const
  {
    Cloud::Ptr odometry_cloud(new Cloud);
    pcl::fromROSMsg(message.cloud, *odometry_cloud);
    Cloud::Ptr local_cloud(new Cloud);
    pcl::transformPointCloud(
      *odometry_cloud, *local_cloud, poseToEigen(odometry_pose).inverse());
    return local_cloud;
  }

  void keyframeCallback(
    const loop_closure::msg::Keyframe::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (keyframes_.count(message->id) != 0U) {
      return;
    }
    if (!keyframes_.empty() && message->id <= keyframes_.rbegin()->first) {
      RCLCPP_WARN(
        get_logger(), "ignoring out-of-order keyframe id %d",
        message->id);
      return;
    }

    const gtsam::Pose3 odometry_pose = poseFromMessage(message->pose);
    Cloud::Ptr local_cloud = extractLocalCloud(*message, odometry_pose);
    if (local_cloud->empty()) {
      RCLCPP_WARN(get_logger(), "ignoring empty keyframe %d", message->id);
      return;
    }

    gtsam::NonlinearFactorGraph factors;
    gtsam::Values initial_values;
    const gtsam::Key current_key = poseKey(message->id);
    if (keyframes_.empty()) {
      initial_values.insert(current_key, odometry_pose);
      factors.add(
        gtsam::PriorFactor<gtsam::Pose3>(
          current_key, odometry_pose, prior_noise_));
    } else {
      const auto & previous = *keyframes_.rbegin();
      const gtsam::Pose3 odometry_increment =
        previous.second.odometry_pose.between(odometry_pose);
      factors.add(
        gtsam::BetweenFactor<gtsam::Pose3>(
          poseKey(previous.first), current_key, odometry_increment,
          odometry_noise_));
      gtsam::Pose3 initial_pose = odometry_pose;
      if (optimized_values_.exists(poseKey(previous.first))) {
        initial_pose =
          optimized_values_.at<gtsam::Pose3>(poseKey(previous.first))
          .compose(odometry_increment);
      }
      initial_values.insert(current_key, initial_pose);
    }

    keyframes_.emplace(
      message->id,
      KeyframeData{message->cloud.header.stamp, odometry_pose, local_cloud});
    isam2_->update(factors, initial_values);
    optimizeAdditionalIterations();
    optimized_values_ = isam2_->calculateEstimate();
    addPendingLoops();
    publishOptimizedPathAndTransform();

  }

  void loopCallback(
    const loop_closure::msg::LoopConstraint::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!hasKeyframe(message->matched_index) ||
      !hasKeyframe(message->current_index))
    {
      pending_loops_.push_back(*message);
      return;
    }
    addLoopFactor(*message);
    publishOptimizedPathAndTransform();
  }

  bool hasKeyframe(int id) const {return keyframes_.count(id) != 0U;}

  void addLoopFactor(const loop_closure::msg::LoopConstraint & constraint)
  {
    gtsam::NonlinearFactorGraph factor;
    factor.add(
      gtsam::BetweenFactor<gtsam::Pose3>(
        poseKey(constraint.matched_index), poseKey(constraint.current_index),
        poseFromMessage(constraint.relative_pose), loop_noise_));
    isam2_->update(factor, gtsam::Values());
    optimizeAdditionalIterations();
    optimized_values_ = isam2_->calculateEstimate();
    ++accepted_loop_count_;
    RCLCPP_INFO(
      get_logger(),
      "optimized loop %d <-> %d (fitness %.3f, total %zu)",
      constraint.current_index, constraint.matched_index,
      constraint.fitness, accepted_loop_count_);
  }

  void addPendingLoops()
  {
    auto iterator = pending_loops_.begin();
    while (iterator != pending_loops_.end()) {
      if (hasKeyframe(iterator->matched_index) &&
        hasKeyframe(iterator->current_index))
      {
        addLoopFactor(*iterator);
        iterator = pending_loops_.erase(iterator);
      } else {
        ++iterator;
      }
    }
  }

  void optimizeAdditionalIterations()
  {
    for (int iteration = 1; iteration < optimization_iterations_; ++iteration) {
      isam2_->update();
    }
  }

  void publishOptimizedPathAndTransform()
  {
    if (keyframes_.empty() || optimized_values_.empty()) {
      return;
    }

    nav_msgs::msg::Path path;
    path.header.frame_id = map_frame_;
    path.header.stamp = now();
    for (const auto &[id, keyframe] : keyframes_) {
      if (!optimized_values_.exists(poseKey(id))) {
        continue;
      }
      geometry_msgs::msg::PoseStamped stamped_pose;
      stamped_pose.header.frame_id = map_frame_;
      stamped_pose.header.stamp = keyframe.stamp;
      stamped_pose.pose =
        poseToMessage(optimized_values_.at<gtsam::Pose3>(poseKey(id)));
      path.poses.push_back(stamped_pose);
    }
    optimized_path_pub_->publish(path);

    const auto & latest = *keyframes_.rbegin();
    const gtsam::Pose3 map_to_base =
      optimized_values_.at<gtsam::Pose3>(poseKey(latest.first));
    const gtsam::Pose3 map_to_odom =
      map_to_base.compose(latest.second.odometry_pose.inverse());
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    transform.header.stamp = latest.second.stamp;
    const auto translation = map_to_odom.translation();
    const auto rotation = map_to_odom.rotation().toQuaternion();
    transform.transform.translation.x = translation.x();
    transform.transform.translation.y = translation.y();
    transform.transform.translation.z = translation.z();
    transform.transform.rotation.w = rotation.w();
    transform.transform.rotation.x = rotation.x();
    transform.transform.rotation.y = rotation.y();
    transform.transform.rotation.z = rotation.z();
    map_to_odom_broadcaster_->sendTransform(transform);
  }

  Cloud::Ptr buildMap() const
  {
    Cloud::Ptr dense_map(new Cloud);
    for (const auto &[id, keyframe] : keyframes_) {
      if (!optimized_values_.exists(poseKey(id))) {
        continue;
      }
      Cloud transformed;
      pcl::transformPointCloud(
        *keyframe.local_cloud, transformed,
        poseToEigen(optimized_values_.at<gtsam::Pose3>(poseKey(id))));
      *dense_map += transformed;
    }

    return dense_map;
  }

  void saveMap(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (keyframes_.empty() || optimized_values_.empty()) {
      response->success = false;
      response->message = "no optimized keyframes available";
      return;
    }

    const std::filesystem::path output_path(save_path_);
    std::error_code error;
    if (output_path.has_parent_path()) {
      std::filesystem::create_directories(output_path.parent_path(), error);
    }
    if (error) {
      response->success = false;
      response->message = "cannot create output directory: " + error.message();
      return;
    }

    const Cloud::Ptr map = buildMap();
    const int result = pcl::io::savePCDFileBinary(save_path_, *map);
    response->success = result == 0;
    response->message = response->success ?
      "saved " + std::to_string(map->size()) +
      " points to " + save_path_ :
      "failed to save " + save_path_;
    if (response->success) {
      RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    }
  }

  std::string map_frame_;
  std::string odom_frame_;
  std::string save_path_;
  double prior_translation_sigma_;
  double prior_rotation_sigma_deg_;
  double odom_translation_sigma_;
  double odom_rotation_sigma_deg_;
  double loop_translation_sigma_;
  double loop_rotation_sigma_deg_;
  double loop_huber_scale_;
  double relinearize_threshold_;
  int relinearize_skip_;
  int optimization_iterations_;
  std::size_t accepted_loop_count_ = 0;

  std::mutex mutex_;
  std::map<int, KeyframeData> keyframes_;
  std::vector<loop_closure::msg::LoopConstraint> pending_loops_;
  std::unique_ptr<gtsam::ISAM2> isam2_;
  gtsam::Values optimized_values_;
  gtsam::SharedNoiseModel prior_noise_;
  gtsam::SharedNoiseModel odometry_noise_;
  gtsam::SharedNoiseModel loop_noise_;

  rclcpp::Subscription<loop_closure::msg::Keyframe>::SharedPtr keyframe_sub_;
  rclcpp::Subscription<loop_closure::msg::LoopConstraint>::SharedPtr loop_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr optimized_path_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> map_to_odom_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseGraphBackend>());
  rclcpp::shutdown();
  return 0;
}
