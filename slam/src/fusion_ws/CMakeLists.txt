cmake_minimum_required(VERSION 3.8)
project(fusion_ws)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(fusion_pcl src/fusion_pcl.cpp)
ament_target_dependencies(
  fusion_pcl
  ament_index_cpp
  livox_ros_driver2
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs)
target_include_directories(fusion_pcl PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(fusion_pcl Eigen3::Eigen ${PCL_LIBRARIES} yaml-cpp)

install(TARGETS fusion_pcl DESTINATION lib/${PROJECT_NAME})
install(
  FILES
    ../../config/gimbal_lidar_3.yaml
    ../../config/gimbal_lidar_5.yaml
  DESTINATION share/${PROJECT_NAME}/config)

ament_package()
