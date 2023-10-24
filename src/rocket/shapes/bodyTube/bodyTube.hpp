# ifndef BODY_TUBE_SHAPE_H_
# define BODY_TUBE_SHAPE_H_

#include "../cylinder.hpp"
#include "../aeroShapeInterface.hpp"
#include "../bodyComponentShape.hpp"

namespace Shapes{
    class BodyTubeShape: public Cylinder, public BodyComponentShape{
        public:
            //constructor for a filled tube
            BodyTubeShape(double radius, double length);
            //constructor for a tube
            BodyTubeShape(double radius, double length, double thickness);

            virtual double wettedArea(); // surface area of the shape exposed to the air
            virtual double referenceArea(); // shapes reference area
            virtual double referenceLength();
            virtual double planformArea();
            virtual double planformCenter(); // geometric center of the shapes planform
    };
}

# endif