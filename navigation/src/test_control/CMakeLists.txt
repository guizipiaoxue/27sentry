cmake_minimum_required(VERSION 3.8)
project(test_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(receiver REQUIRED)

add_executable(keyboard_control src/keyboard_control.cpp)
target_compile_features(keyboard_control PRIVATE cxx_std_17)
add_executable(mpc_control src/mpc_control.cpp)
target_compile_features(mpc_control PRIVATE cxx_std_17)
target_link_libraries(mpc_control Eigen3::Eigen)
ament_target_dependencies(mpc_control rclcpp nav_msgs geometry_msgs std_msgs tf2_ros receiver)

install(TARGETS keyboard_control mpc_control DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
