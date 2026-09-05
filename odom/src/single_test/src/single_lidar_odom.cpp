// 单雷达里程计测试入口。
// DLIO 的 OdomNode 使用相对话题名 `pointcloud` 和 `imu`。MID360 3 号雷达
// 驱动默认发布 sensor_msgs/msg/PointCloud2 和 sensor_msgs/msg/Imu，话题名为
// livox/lidar_192_168_1_3 与 livox/imu_192_168_1_3，因此在这里自动完成重映射。

#include "dlio/odom.h"

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
  // 直接运行时连接到 3 号 MID360；若调用者已经提供 --ros-args，则保留其配置。
  std::vector<std::string> args;
  args.reserve(static_cast<std::size_t>(argc) + 8U);
  for (int i = 0; i < argc; ++i) {
    args.emplace_back(argv[i]);
  }

  bool has_ros_args = false;
  for (int i = 1; i < argc; ++i) {
    if (args[static_cast<std::size_t>(i)] == "--ros-args") {
      has_ros_args = true;
      break;
    }
  }
  if (!has_ros_args) {
    args.insert(args.end(), {
        "--ros-args", "-r", "pointcloud:=livox/lidar_192_168_1_3",
        "-r", "imu:=livox/imu_192_168_1_3"});
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
