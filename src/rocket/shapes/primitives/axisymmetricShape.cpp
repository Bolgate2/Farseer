#include "axisymmetricShape.hpp"
#include <cmath>

namespace Shapes{

    double AxisymmetricShape::area(double x){
        auto rad = radius(x);
        return M_PI*std::pow(rad,2);
    }
}