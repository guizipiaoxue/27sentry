// DLIO entry point for the cloud and IMU already fused in the gimbal frame.
// The fused PointXYZI cloud has no per-point timestamps, so deskewing is
// intentionally disabled. No startup IMU calibration is performed.

#include "dlio/odom.h"

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  // Put defaults before user arguments. ROS 2 uses the last override, allowing
  // every default below to be changed from the command line.
  std::vector<std::string> args;
  args.reserve(static_cast<std::size_t>(argc) + 32U);
  args.emplace_back(argv[0]);
  args.insert(args.end(), {
      "--ros-args",
      "-r", "pointcloud:=/gimbal/cloud_fused",
      "-r", "imu:=/gimbal/imu_fused",
      "-r", "path:=/path",
      "-r", "deskewed:=/fusion_pcl",
      "-p", "imu/calibration:=false",
      "-p", "pointcloud/deskew:=false",
      "-p", "odom/computeTimeOffset:=false",
      "-p", "publish/pose_odom:=true",
      "-p", "publish/keyframes:=true",
      "-p", "frames/odom:=odom",
      "-p", "frames/baselink:=gimbal",
      "-p", "frames/lidar:=fusion_lidar",
      "-p", "frames/imu:=fusion_imu",
      "--"});
  for (int i = 1; i < argc; ++i) {
    args.emplace_back(argv[i]);
  }

  std::vector<char *> ros_argv;
  ros_argv.reserve(args.size());
  for (auto &arg : args) {
    ros_argv.push_back(arg.data());
  }

  rclcpp::init(static_cast<int>(ros_argv.size()), ros_argv.data());
  auto node = std::make_shared<dlio::OdomNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
