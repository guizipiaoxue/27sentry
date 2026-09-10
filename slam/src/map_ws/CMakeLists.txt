cmake_minimum_required(VERSION 3.8)
project(map_ws)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io kdtree)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(kdtree_map src/map.cpp)
target_compile_features(kdtree_map PRIVATE cxx_std_17)
target_include_directories(kdtree_map PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(kdtree_map ${PCL_LIBRARIES})
ament_target_dependencies(
  kdtree_map
  pcl_conversions
  rclcpp
  sensor_msgs
  std_srvs)

install(TARGETS kdtree_map DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES ../../config/map.yaml DESTINATION share/${PROJECT_NAME}/config)

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
