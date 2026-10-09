cmake_minimum_required(VERSION 3.16)
project(sentry_livox_sdk LANGUAGES C CXX)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_subdirectory("${LIVOX_SDK_SOURCE}" sdk)
# A unique SONAME prevents a runtime library path from substituting the unsafe
# system SDK. Only the public C API may cross the SDK/ROS boundary.
set_target_properties(livox_lidar_sdk_shared PROPERTIES
  OUTPUT_NAME livox_lidar_sdk_sentry)
target_link_options(livox_lidar_sdk_shared PRIVATE
  "-Wl,-Bsymbolic" "-Wl,--version-script=${CMAKE_CURRENT_LIST_DIR}/../livox_sdk_exports.map")
