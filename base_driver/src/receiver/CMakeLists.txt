cmake_minimum_required(VERSION 3.8)
project(receiver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UplinkFrame.msg"
  "msg/GimbalState.msg"
  "msg/ChassisState.msg"
  "msg/GameState.msg"
  "msg/PositionPair.msg"
  "msg/GimbalDynamics.msg"
  DEPENDENCIES std_msgs
)

add_executable(receiver_node src/receiver.cpp)
target_compile_features(receiver_node PRIVATE cxx_std_17)
ament_target_dependencies(receiver_node rclcpp std_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(receiver_node "${cpp_typesupport_target}")

install(TARGETS receiver_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
