# ifndef AERO_SHAPE_H_
# define AERO_SHAPE_H_

#include "shape.hpp"
#include "aeroShapeInterface.hpp"

namespace Shapes{
    class AeroShape : virtual public Shape, virtual public AeroShapeInterface {

    };
    
}

#endif