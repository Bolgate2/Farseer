#include "shape.hpp"
#include <Eigen/Dense>

namespace Shapes{
    // unless otherwise specified, the shapes "center of mass" and origin are identical
    Eigen::Vector3d Shape::cm(){
        return Eigen::Vector3d::Zero();
    }
}