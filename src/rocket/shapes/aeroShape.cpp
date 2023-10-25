#include "aeroShape.hpp"
#include <Eigen/Dense>
#include <iostream>

namespace Shapes{

    double AeroShape::volume(){
        return shape()->volume();
    }

    Eigen::Matrix3d AeroShape::inertia(){
        return shape()->inertia();
    }
}