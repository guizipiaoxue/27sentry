cmake_minimum_required(VERSION 3.8)
project(single_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters registration)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED)

set(DLIO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../direct_lidar_inertial_odometry)
include_directories(${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${DLIO_ROOT}/include)

add_executable(single_lidar_odom
  src/single_lidar_odom.cpp
  ${DLIO_ROOT}/src/dlio/odom.cc
  ${DLIO_ROOT}/src/nano_gicp/lsq_registration.cc
  ${DLIO_ROOT}/src/nano_gicp/nano_gicp.cc
  ${DLIO_ROOT}/src/nano_gicp/nanoflann.cc)
target_compile_features(single_lidar_odom PRIVATE cxx_std_17)
target_compile_options(single_lidar_odom PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(single_lidar_odom
  ${PCL_LIBRARIES}
  Threads::Threads
  OpenMP::OpenMP_CXX)
ament_target_dependencies(single_lidar_odom
  rclcpp sensor_msgs nav_msgs geometry_msgs pcl_conversions tf2_ros pcl_ros)

install(TARGETS single_lidar_odom DESTINATION lib/${PROJECT_NAME})

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

ament_package()
