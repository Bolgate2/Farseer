#include "componentShape.hpp"
#include <Eigen/Dense>

namespace Shapes{
    //ComponentShape
    double ComponentShape::volume(){
        return shape()->volume();
    }
    Eigen::Matrix3d ComponentShape::inertia(){
        return shape()->inertia();
    }
    Eigen::Vector3d ComponentShape::cm(){
        return shape()->cm();
    }
}