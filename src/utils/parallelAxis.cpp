#include "parallelAxis.hpp"
#include <Eigen/Dense>
#include <cmath>

namespace Utils{
    Eigen::Matrix3d parallel_axis_transform(Eigen::Matrix3d inertia, Eigen::Vector3d translation, double volume){
        auto x = translation[0];
        auto y = translation[1];
        auto z = translation[2];
        auto x2 = std::pow(x,2);
        auto y2 = std::pow(y,2);
        auto z2 = std::pow(z,2);
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