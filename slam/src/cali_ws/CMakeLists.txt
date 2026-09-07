cmake_minimum_required(VERSION 3.8)
project(cali_ws)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters registration segmentation)
find_package(livox_ros_driver2 REQUIRED)

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

add_executable(rotate_cali src/rotate_cali.cpp)
ament_target_dependencies(rotate_cali rclcpp sensor_msgs pcl_conversions pcl_ros livox_ros_driver2)
target_include_directories(rotate_cali PRIVATE ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(rotate_cali Eigen3::Eigen ${PCL_LIBRARIES})
install(TARGETS rotate_cali DESTINATION lib/${PROJECT_NAME})

add_executable(circle_cali_dlio src/circle_cali_dlio.cpp)
ament_target_dependencies(circle_cali_dlio rclcpp geometry_msgs)
target_link_libraries(circle_cali_dlio Eigen3::Eigen)
target_include_directories(circle_cali_dlio PRIVATE ${EIGEN3_INCLUDE_DIRS})
install(TARGETS circle_cali_dlio DESTINATION lib/${PROJECT_NAME})

add_executable(pcl_publish src/pcl_publish.cpp)
ament_target_dependencies(pcl_publish rclcpp sensor_msgs pcl_conversions pcl_ros livox_ros_driver2)
target_include_directories(pcl_publish PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(pcl_publish Eigen3::Eigen ${PCL_LIBRARIES})
install(TARGETS pcl_publish DESTINATION lib/${PROJECT_NAME})

add_executable(use_ground_cali src/use_ground_cali.cpp)
ament_target_dependencies(use_ground_cali rclcpp sensor_msgs pcl_conversions pcl_ros livox_ros_driver2)
target_include_directories(use_ground_cali PRIVATE ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(use_ground_cali Eigen3::Eigen ${PCL_LIBRARIES})
install(TARGETS use_ground_cali DESTINATION lib/${PROJECT_NAME})

ament_package()
