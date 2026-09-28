cmake_minimum_required(VERSION 3.8)
project(sender)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(receiver REQUIRED)

add_executable(sender_node src/sender.cpp)
target_compile_features(sender_node PRIVATE cxx_std_17)
ament_target_dependencies(sender_node rclcpp std_msgs geometry_msgs receiver)

install(TARGETS sender_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_package()
