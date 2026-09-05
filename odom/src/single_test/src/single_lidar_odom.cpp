/***********************************************************
 *                                                         *
 * Official DLIO ROS 2 entry point.                       *
 *                                                         *
 * The odometry implementation is intentionally kept in   *
 * direct_lidar_inertial_odometry/src/dlio/odom.cc. This   *
 * executable only creates the official OdomNode; topic     *
 * names and all algorithm parameters are configured using  *
 * ROS 2 parameters/remappings, exactly as in dlio.launch.py.*
 *                                                         *
 ***********************************************************/

#include "dlio/odom.h"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<dlio::OdomNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
