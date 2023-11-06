#include "maths.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>

namespace Utils{
    Eigen::Matrix3d parallel_axis_transform(Eigen::Matrix3d inertia, Eigen::Vector3d translation, double volume, bool inverse){
        // -(d^2)
        Eigen::Matrix3d translationInertia = translation.dot(translation)*Eigen::Matrix3d::Identity() - translation*translation.transpose();
        // -(d^2)*M
        translationInertia *= volume;
        
        auto translatedInertia = inertia;
        if(inverse){
            translatedInertia -= translationInertia;
        } else {
            translatedInertia += translationInertia;
        }
        return translatedInertia;
    }

    double beta(double mach){
        return std::sqrt(std::abs(1-std::pow(mach,2)));
    }


}