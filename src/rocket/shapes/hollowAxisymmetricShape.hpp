# ifndef HOLLOW_AXISYMMETRIC_SHAPE_H_
# define HOLLOW_AXISYMMETRIC_SHAPE_H_

#include "hollowShape.hpp"
#include "axisymmetricShape.hpp"

namespace Shapes{
    class HollowAxisymmetricShape: public HollowShape, public AxisymmetricShape{
    };
}

# endif