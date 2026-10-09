#include <iostream>
#include <dlfcn.h>

#include <rclcpp/rclcpp.hpp>
#include <livox_lidar_api.h>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("sdk_logging_test"), "ROS logging before SDK logging");
  // Load ROS first, matching the component container's symbol resolution order.
  void* library = dlopen(LIVOX_TEST_SDK_LIBRARY, RTLD_NOW | RTLD_LOCAL);
  if (!library) {
    std::cerr << dlerror() << '\n';
    return 1;
  }
  const auto initialize = reinterpret_cast<decltype(&LivoxLidarSdkInit)>(
      dlsym(library, "LivoxLidarSdkInit"));
  if (!initialize) return 1;
  // Invalid JSON triggers SDK logging without opening sensor sockets.
  const bool initialized = initialize("/dev/null", "", nullptr);
  if (initialized) {
    std::cerr << "SDK unexpectedly accepted invalid JSON\n";
    return 1;
  }
  RCLCPP_INFO(rclcpp::get_logger("sdk_logging_test"), "ROS logging after SDK logging");
  dlclose(library);
  rclcpp::shutdown();
  std::cout << "SDK and ROS logging coexist without a crash\n";
  return 0;
}
