#include <cmath>
#include <cstdlib>
#include <iostream>

#include "plio/point_lio_core.hpp"

int main() {
  plio::core::IVox::Options options;
  options.resolution_ = 0.30F;
  options.max_points_per_voxel_ = 4;
  plio::core::IVox map(options);

  for (int i = 0; i < 20; ++i) {
    plio::core::Point point{};
    point.x = static_cast<float>(i) * 0.01F;
    map.AddPoints(plio::core::PointVector{point});
  }

  if (map.NumValidGrids() != 1 ||
      map.grids_map_.begin()->second->second.Size() != 4) {
    std::cerr << "iVox point count exceeded its per-voxel limit\n";
    return EXIT_FAILURE;
  }

  plio::core::Point query{};
  query.x = 0.19F;
  plio::core::PointVector nearest;
  if (!map.GetClosestPoint(query, nearest, 1, 0.03) || nearest.empty() ||
      std::abs(nearest.front().x - query.x) > 1.0e-5F) {
    std::cerr << "iVox did not retain a recent point\n";
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
