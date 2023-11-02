#include "parallelAxis.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>

namespace Utils{
    Eigen::Matrix3d parallel_axis_transform(Eigen::Matrix3d inertia, Eigen::Vector3d translation, double volume){
        auto x = translation.x();
        auto y = translation.y();
        auto z = translation.z();
        auto x2 = std::pow(x,2);
        if(x < 0) x2 *= -1;
        auto y2 = std::pow(y,2);
        if(y < 0) y2 *= -1;
        auto z2 = std::pow(z,2);
        if(z < 0) z2 *= -1;
        Eigen::Matrix3d translationInertia {
            {y2+z2, x*y,    x*z},
            {x*y,   x2+z2,  y*z},
            {x*z,   y*z,    x2+y2}
        };
        translationInertia *= volume;
        auto translatedInertia = inertia + translationInertia;
        return translatedInertia;
    }
}