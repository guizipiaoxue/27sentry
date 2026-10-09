#!/usr/bin/env bash
set -Eeuo pipefail
LIVOX_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SDK_SOURCE="${1:-${LIVOX_WS}/../../Livox-SDK2}"
SDK_LOCAL="${LIVOX_WS}/src/livox_ros_driver2/.livox_sdk"
[[ -f "${SDK_SOURCE}/sdk_core/CMakeLists.txt" ]] || {
  echo "Usage: $0 /path/to/Livox-SDK2" >&2
  exit 2
}
# Keep the SDK's bundled spdlog/fmt/rapidjson private to its own shared object.
# No SDK source edits, system installation, sensor access or PTP changes.
cmake -S "${LIVOX_WS}/src/livox_ros_driver2/cmake/isolated_sdk" \
  -B "${SDK_LOCAL}/isolated-build" -DCMAKE_BUILD_TYPE=Release \
  "-DLIVOX_SDK_SOURCE=$(realpath "${SDK_SOURCE}")"
cmake --build "${SDK_LOCAL}/isolated-build" --target livox_lidar_sdk_shared --parallel 4
mkdir -p "${SDK_LOCAL}/lib" "${SDK_LOCAL}/include"
cp "${SDK_LOCAL}/isolated-build/sdk/sdk_core/liblivox_lidar_sdk_sentry.so" "${SDK_LOCAL}/lib/"
cp "${SDK_SOURCE}"/include/livox_lidar_*.h "${SDK_LOCAL}/include/"
echo "Isolated SDK: ${SDK_LOCAL}/lib/liblivox_lidar_sdk_sentry.so"
