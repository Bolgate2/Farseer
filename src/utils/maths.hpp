#ifndef MATH_UTILS_H_
#define MATH_UTILS_H_

#include <Eigen/Dense>

namespace Utils{
    Eigen::Matrix3d parallel_axis_transform(Eigen::Matrix3d inertia, Eigen::Vector3d translation, double volume, bool inverse=false);
    //TODO: add vectorized version of this
    double beta(double mach);

    /**
     * @brief transforms a set of euler angles into a rotation matrix, extrinsic rotation as coordinate system is fixed
     * 
     * @param yaw rotation about the x axis in radians
     * @param pitch rotation about the y axis in radians
     * @param roll rotation about the z axis in radians
     * @return Eigen::Matrix3d 
     */
    Eigen::Matrix3d eulerToRotmat(double yaw, double pitch, double roll);
}

#endif