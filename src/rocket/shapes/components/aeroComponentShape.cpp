#include "aeroComponentShape.hpp"
#include <Eigen/Dense>
#include <iostream>

namespace Shapes{

    double AeroComponentShape::volume(){
        return shape()->volume();
    }

    Eigen::Matrix3d AeroComponentShape::inertia(){
        return shape()->inertia();
    }
}