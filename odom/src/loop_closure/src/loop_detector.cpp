#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>

#include "loop_closure/msg/keyframe.hpp"
#include "loop_closure/msg/loop_constraint.hpp"

namespace {

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

Eigen::Isometry3f poseToIsometry(const geometry_msgs::msg::Pose &pose) {
  Eigen::Quaternionf rotation(
      static_cast<float>(pose.orientation.w),
      static_cast<float>(pose.orientation.x),
      static_cast<float>(pose.orientation.y),
      static_cast<float>(pose.orientation.z));
  Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
  transform.linear() = rotation.normalized().toRotationMatrix();
  transform.translation() = Eigen::Vector3f(
      static_cast<float>(pose.position.x), static_cast<float>(pose.position.y),
      static_cast<float>(pose.position.z));
  return transform;
}

geometry_msgs::msg::Pose isometryToPose(const Eigen::Isometry3f &transform) {
  geometry_msgs::msg::Pose pose;
  const Eigen::Quaternionf rotation(transform.linear());
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  pose.orientation.w = rotation.w();
  pose.orientation.x = rotation.x();
  pose.orientation.y = rotation.y();
  pose.orientation.z = rotation.z();
  return pose;
}

}  // namespace

class LoopDetector final : public rclcpp::Node {
 public:
  LoopDetector() : Node("loop_detector") {
    rings_ = declare_parameter<int>("detector.rings", 20);
    sectors_ = declare_parameter<int>("detector.sectors", 60);
    max_radius_ = declare_parameter<double>("detector.max_radius", 80.0);
    voxel_size_ = declare_parameter<double>("detector.voxel_size", 0.25);
    descriptor_threshold_ =
        declare_parameter<double>("detector.descriptor_threshold", 0.20);
    num_candidates_ =
        declare_parameter<int>("detector.num_candidates", 5);
    exclusion_recent_ =
        declare_parameter<int>("detector.exclusion_recent", 30);
    min_travel_distance_ =
        declare_parameter<double>("detector.min_travel_distance", 10.0);
    icp_max_correspondence_ =
        declare_parameter<double>("detector.icp_max_correspondence", 2.0);
    icp_iterations_ =
        declare_parameter<int>("detector.icp_iterations", 40);
    max_fitness_ = declare_parameter<double>("detector.max_fitness", 0.35);

    if (rings_ <= 0 || sectors_ <= 0 || voxel_size_ <= 0.0 ||
        max_radius_ <= 0.0 || num_candidates_ <= 0) {
      throw std::invalid_argument("invalid loop detector parameters");
    }

    keyframe_sub_ = create_subscription<loop_closure::msg::Keyframe>(
        "keyframe", rclcpp::QoS(20).reliable(),
        std::bind(&LoopDetector::keyframeCallback, this,
                  std::placeholders::_1));
    loop_pub_ = create_publisher<loop_closure::msg::LoopConstraint>(
        "loop_constraint", rclcpp::QoS(10).reliable());
  }

 private:
  struct Frame {
    int id;
    geometry_msgs::msg::Pose pose;
    Cloud::Ptr local_cloud;
    Eigen::MatrixXf descriptor;
    Eigen::VectorXf ring_key;
  };

  Eigen::MatrixXf makeDescriptor(const Cloud &cloud) const {
    Eigen::MatrixXf descriptor = Eigen::MatrixXf::Constant(
        rings_, sectors_, -std::numeric_limits<float>::infinity());
    for (const auto &point : cloud.points) {
      const float radius = std::hypot(point.x, point.y);
      if (radius >= max_radius_ || radius < 0.1F) {
        continue;
      }
      const int ring = std::min(
          rings_ - 1, static_cast<int>(radius / max_radius_ * rings_));
      float angle = std::atan2(point.y, point.x);
      if (angle < 0.0F) {
        angle += 2.0F * static_cast<float>(M_PI);
      }
      const int sector = std::min(
          sectors_ - 1,
          static_cast<int>(angle / (2.0F * static_cast<float>(M_PI)) *
                           sectors_));
      descriptor(ring, sector) =
          std::max(descriptor(ring, sector), point.z);
    }
    for (int ring = 0; ring < rings_; ++ring) {
      for (int sector = 0; sector < sectors_; ++sector) {
        if (!std::isfinite(descriptor(ring, sector))) {
          descriptor(ring, sector) = 0.0F;
        }
      }
    }
    return descriptor;
  }

  double descriptorDistance(
      const Eigen::MatrixXf &first, const Eigen::MatrixXf &second,
      int *best_shift) const {
    double best = std::numeric_limits<double>::max();
    int selected_shift = 0;
    for (int shift = 0; shift < sectors_; ++shift) {
      double sum = 0.0;
      int count = 0;
      for (int ring = 0; ring < rings_; ++ring) {
        for (int sector = 0; sector < sectors_; ++sector) {
          const double lhs = first(ring, sector);
          const double rhs = second(ring, (sector + shift) % sectors_);
          const double denominator = std::abs(lhs) + std::abs(rhs);
          if (denominator > 1e-3) {
            sum += std::abs(lhs - rhs) / denominator;
            ++count;
          }
        }
      }
      const double distance = count > 0 ? sum / count : 1e9;
      if (distance < best) {
        best = distance;
        selected_shift = shift;
      }
    }
    *best_shift = selected_shift;
    return best;
  }

  void keyframeCallback(
      const loop_closure::msg::Keyframe::SharedPtr message) {
    if (!frames_.empty() && message->id <= frames_.back().id) {
      RCLCPP_WARN(get_logger(), "ignoring non-increasing keyframe id %d",
                  message->id);
      return;
    }

    Cloud::Ptr odom_cloud(new Cloud);
    pcl::fromROSMsg(message->cloud, *odom_cloud);
    if (odom_cloud->empty()) {
      return;
    }

    Cloud::Ptr local_cloud(new Cloud);
    pcl::transformPointCloud(
        *odom_cloud, *local_cloud, poseToIsometry(message->pose).inverse());
    Cloud::Ptr filtered(new Cloud);
    pcl::VoxelGrid<Point> voxel_filter;
    voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    voxel_filter.setInputCloud(local_cloud);
    voxel_filter.filter(*filtered);
    if (filtered->empty()) {
      return;
    }

    Frame frame;
    frame.id = message->id;
    frame.pose = message->pose;
    frame.local_cloud = filtered;
    frame.descriptor = makeDescriptor(*filtered);
    frame.ring_key = frame.descriptor.rowwise().mean();

    const int current = static_cast<int>(frames_.size());
    frames_.push_back(std::move(frame));
    if (current <= exclusion_recent_) {
      return;
    }

    const auto &current_frame = frames_.back();
    const auto &current_position = current_frame.pose.position;
    double maximum_travel = 0.0;
    for (int index = 0; index < current; ++index) {
      const auto &position = frames_[index].pose.position;
      maximum_travel = std::max(
          maximum_travel,
          std::hypot(current_position.x - position.x,
                     current_position.y - position.y));
    }
    if (maximum_travel < min_travel_distance_) {
      return;
    }

    std::vector<std::pair<double, int>> ranked;
    for (int index = 0; index < current - exclusion_recent_; ++index) {
      ranked.emplace_back(
          (current_frame.ring_key - frames_[index].ring_key).norm(), index);
    }
    std::sort(ranked.begin(), ranked.end());

    int tested = 0;
    for (const auto &[ring_distance, candidate] : ranked) {
      if (tested++ >= num_candidates_ ||
          ring_distance > descriptor_threshold_ * 10.0) {
        break;
      }
      int shift = 0;
      const double scan_context_distance = descriptorDistance(
          current_frame.descriptor, frames_[candidate].descriptor, &shift);
      if (scan_context_distance > descriptor_threshold_) {
        continue;
      }

      const Eigen::Isometry3f odometry_guess =
          poseToIsometry(frames_[candidate].pose).inverse() *
          poseToIsometry(current_frame.pose);
      pcl::GeneralizedIterativeClosestPoint<Point, Point> registration;
      registration.setMaxCorrespondenceDistance(icp_max_correspondence_);
      registration.setMaximumIterations(icp_iterations_);
      registration.setInputSource(current_frame.local_cloud);
      registration.setInputTarget(frames_[candidate].local_cloud);
      Cloud aligned;
      registration.align(aligned, odometry_guess.matrix());
      if (!registration.hasConverged() ||
          registration.getFitnessScore() > max_fitness_) {
        continue;
      }

      loop_closure::msg::LoopConstraint output;
      output.header = message->cloud.header;
      output.current_index = current_frame.id;
      output.matched_index = frames_[candidate].id;
      output.relative_pose = isometryToPose(Eigen::Isometry3f(
          registration.getFinalTransformation()));
      output.fitness = registration.getFitnessScore();
      output.yaw_difference = static_cast<double>(shift) * 2.0 * M_PI /
                              static_cast<double>(sectors_);
      loop_pub_->publish(output);
      RCLCPP_INFO(get_logger(), "loop closure %d <-> %d, fitness %.3f",
                  output.current_index, output.matched_index, output.fitness);
      break;
    }
  }

  int rings_;
  int sectors_;
  int num_candidates_;
  int exclusion_recent_;
  int icp_iterations_;
  double max_radius_;
  double voxel_size_;
  double descriptor_threshold_;
  double min_travel_distance_;
  double icp_max_correspondence_;
  double max_fitness_;
  std::vector<Frame> frames_;
  rclcpp::Subscription<loop_closure::msg::Keyframe>::SharedPtr keyframe_sub_;
  rclcpp::Publisher<loop_closure::msg::LoopConstraint>::SharedPtr loop_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LoopDetector>());
  rclcpp::shutdown();
  return 0;
}
