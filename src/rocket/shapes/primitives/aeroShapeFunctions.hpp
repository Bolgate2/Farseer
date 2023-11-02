# ifndef AERO_SHAPE_INTERFACE_H_
# define AERO_SHAPE_INTERFACE_H_

namespace Shapes {
    // this class defines the methods required by a shape for it to be used for aerodynamics
    class AeroShapeFunctions{
        public:
            virtual double wettedArea() = 0; // surface area of the shape exposed to the air
            virtual double referenceArea() = 0; // shapes reference area
            virtual double planformArea() = 0;
            virtual double planformCenter() = 0; // geometric center of the shapes planform
    };
}


# endif

