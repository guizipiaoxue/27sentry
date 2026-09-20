#include <cstdlib>
#include <iostream>
#include "plio/point_lio_core.hpp"

namespace {
int count = 1;
void measurement(plio::core::State &, Eigen::Matrix3d, Eigen::Matrix3d,
                 esekfom::dyn_share_modified<double> &data) {
  data.valid = true;
  data.M_Noise = 0.01;
  data.h_x = Eigen::MatrixXd::Zero(count, 12);
  data.h_x.col(0).setOnes();
  data.z = Eigen::VectorXd::Constant(count, 0.1);
}
void imu(plio::core::State &, esekfom::dyn_share_modified<double> &) {}
}
int main() {
  for (int n : {1, 29, 30, 48}) {
    count = n;
    plio::core::Filter filter;
    filter.init_dyn_share_modified_3h(plio::core::processModel,
        plio::core::processJacobian, measurement, imu);
    Eigen::Matrix<double, 30, 30> covariance = Eigen::Matrix<double, 30, 30>::Identity() * .01;
    filter.change_P(covariance);
    filter.update_iterated_dyn_share_modified();
    const double expected = .1 * n / (1. + n);
    if (std::abs(filter.x_.pos.x() - expected) > 1e-10) {
      std::cerr << "LiDAR covariance branch incorrect for " << n << " rows\n";
      return EXIT_FAILURE;
    }
  }
  std::cout << "Both LiDAR update branches match analytic scalar posterior\n";
  return EXIT_SUCCESS;
}
