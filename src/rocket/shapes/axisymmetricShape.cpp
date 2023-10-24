#include "axisymmetricShape.hpp"
#include <cmath>

namespace Shapes{
    void AxisymmetricShape::setLength(double length){
        if(length >= 0){
            _length = length;
        } else {
            // TODO: raise warning
            _length = 0;
        }
    }

    double AxisymmetricShape::length(){
        return _length;
    }

    double AxisymmetricShape::area(double x){
        auto rad = radius(x);
        return M_PI*std::pow(rad,2);
    }
}