cmake_minimum_required(VERSION 3.16)
project(loop_closure)

option(LOOP_CLOSURE_USE_SYSTEM_GTSAM
  "Use an installed GTSAM instead of the source vendored at repository root" OFF)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io registration)
find_package(Eigen3 REQUIRED)

if(LOOP_CLOSURE_USE_SYSTEM_GTSAM)
  find_package(GTSAM REQUIRED)
else()
  get_filename_component(REPOSITORY_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
  set(GTSAM_SOURCE_DIR "${REPOSITORY_ROOT}/gtsam")
  if(NOT EXISTS "${GTSAM_SOURCE_DIR}/CMakeLists.txt")
    message(FATAL_ERROR
      "Vendored GTSAM is missing at ${GTSAM_SOURCE_DIR}. "
      "Restore the gtsam directory or configure "
      "-DLOOP_CLOSURE_USE_SYSTEM_GTSAM=ON.")
  endif()
  set(GTSAM_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GTSAM_BUILD_EXAMPLES_ALWAYS OFF CACHE BOOL "" FORCE)
  set(GTSAM_BUILD_TIMING_ALWAYS OFF CACHE BOOL "" FORCE)
  set(GTSAM_BUILD_UNSTABLE OFF CACHE BOOL "" FORCE)
  set(GTSAM_BUILD_PYTHON OFF CACHE BOOL "" FORCE)
  set(GTSAM_INSTALL_MATLAB_TOOLBOX OFF CACHE BOOL "" FORCE)
  set(GTSAM_BUILD_WITH_MARCH_NATIVE OFF CACHE BOOL "" FORCE)
  set(GTSAM_FORCE_STATIC_LIB ON CACHE BOOL "" FORCE)
  set(GTSAM_USE_SYSTEM_EIGEN ON CACHE BOOL "" FORCE)
  set(GTSAM_USE_BOOST_FEATURES OFF CACHE BOOL "" FORCE)
  set(GTSAM_ENABLE_BOOST_SERIALIZATION OFF CACHE BOOL "" FORCE)
  set(GTSAM_WITH_TBB OFF CACHE BOOL "" FORCE)
  set(GTSAM_SUPPORT_NESTED_DISSECTION OFF CACHE BOOL "" FORCE)
  set(GTSAM_BUILD_TYPE_POSTFIXES OFF CACHE BOOL "" FORCE)
  add_subdirectory("${GTSAM_SOURCE_DIR}"
    "${CMAKE_CURRENT_BINARY_DIR}/gtsam" EXCLUDE_FROM_ALL)
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LoopConstraint.msg"
  "msg/Keyframe.msg"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs builtin_interfaces
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(loop_detector src/loop_detector.cpp)
target_compile_features(loop_detector PRIVATE cxx_std_17)
target_include_directories(loop_detector PRIVATE ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(loop_detector ${PCL_LIBRARIES} ${cpp_typesupport_target})
ament_target_dependencies(loop_detector rclcpp geometry_msgs sensor_msgs std_msgs pcl_conversions)

add_executable(pose_graph_backend src/pose_graph_backend.cpp)
target_compile_features(pose_graph_backend PRIVATE cxx_std_17)
target_include_directories(
  pose_graph_backend PRIVATE ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(
  pose_graph_backend ${PCL_LIBRARIES} gtsam ${cpp_typesupport_target})
ament_target_dependencies(
  pose_graph_backend
  geometry_msgs
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_srvs
  tf2_ros)

install(TARGETS loop_detector pose_graph_backend DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../config/loop.yaml"
  DESTINATION share/${PROJECT_NAME}/config)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
