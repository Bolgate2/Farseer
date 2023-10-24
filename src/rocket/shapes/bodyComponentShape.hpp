# ifndef BODY_COMPONENT_SHAPE_H_
# define BODY_COMPONENT_SHAPE_H_

#include "axisymmetricShape.hpp"
#include "hollowShape.hpp"
#include "aeroShapeInterface.hpp"
#include "aeroShape.hpp"

namespace Shapes{
    // this exists for the shape in BodyComponent
    class BodyComponentShape: virtual public HollowShape, virtual public AxisymmetricShape, public AeroShape{
        public:
            virtual double referenceLength() = 0;
    };
}

#endif