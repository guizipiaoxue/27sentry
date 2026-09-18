#pragma once

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp"
#include "ivox/ivox3d.h"
#include "so3_math.h"

namespace plio::core {

using Vect3 = MTK::vect<3, double>;
using SO3 = MTK::SO3<double>;

MTK_BUILD_MANIFOLD(State,
((Vect3, pos))
((SO3, rot))
((SO3, offset_R_L_I))
((Vect3, offset_T_L_I))
((Vect3, vel))
((Vect3, bg))
((Vect3, ba))
((Vect3, gravity))
);

MTK_BUILD_MANIFOLD(Input,
((Vect3, acc))
((Vect3, gyro))
);

using Filter = esekfom::esekf<State, 24, Input>;
using Point = pcl::PointXYZINormal;
using Cloud = pcl::PointCloud<Point>;
using PointVector = std::vector<Point, Eigen::aligned_allocator<Point>>;
using IVox = faster_lio::IVox<3, faster_lio::IVoxNodeType::DEFAULT, Point>;

Eigen::Matrix<double, 24, 1> processModel(State &state, const Input &input);
Eigen::Matrix<double, 24, 24> processJacobian(State &state, const Input &input);
Eigen::Matrix<double, 24, 24> processNoise(
    double gyro, double accel, double gyro_bias, double accel_bias);

}  // namespace plio::core
