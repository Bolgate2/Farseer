# ifndef AXISYMMETRIC_SHAPE_H_
# define AXISYMMETRIC_SHAPE_H_

#include <Eigen/Dense>
#include "shape.hpp"

namespace Shapes{
    class AxisymmetricShape{
        public:
            virtual double length() = 0; // returns the length of the shape
            virtual void setLength(double length) = 0; //sets the length of the shape

            virtual double radius(double x) = 0; // gets the radius of the shape at distance x from the origin along the axis of symmetry
            virtual double radius() = 0; //gets the base radius of this shape, the base radius is used to construct this shape
            virtual void setRadius(double radius) = 0; // sets the base radius for the shape

            virtual double area(double x); // gets the area of the shape at distance x from the origin along the axis of symmetry, by refault pi*radius(x)^2
    };
}

#endif