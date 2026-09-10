cmake_minimum_required(VERSION 3.8)
project(loop_closure)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters registration)
find_package(Eigen3 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LoopConstraint.msg"
  "msg/Keyframe.msg"
  DEPENDENCIES geometry_msgs sensor_msgs builtin_interfaces
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(loop_detector src/loop_detector.cpp)
target_compile_features(loop_detector PRIVATE cxx_std_17)
target_include_directories(loop_detector PRIVATE ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(loop_detector ${PCL_LIBRARIES})
ament_target_dependencies(loop_detector rclcpp geometry_msgs sensor_msgs std_msgs pcl_conversions)
add_dependencies(loop_detector ${cpp_typesupport_target})
rosidl_target_interfaces(loop_detector ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS loop_detector DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
