#ifndef MATH_UTILS_H_
#define MATH_UTILS_H_

#include <Eigen/Dense>

namespace Utils{
    Eigen::Matrix3d parallel_axis_transform(Eigen::Matrix3d inertia, Eigen::Vector3d translation, double volume);
    //TODO: add vectorized version of this
    double beta(double mach);
}

#endif