# ifndef CYLINDER_H_
# define CYLINDER_H_

#include "axisymmetricShape.hpp"
#include "hollowShape.hpp"
#include <Eigen/Dense>
#include <cmath>

namespace Shapes{
    class Cylinder: virtual public HollowShape, virtual public AxisymmetricShape{
        private:
            double _radius = 0;
            double _length = 0;
            double unfilledVolume();
            double filledVolume();

            // inertia functions with caches
            Eigen::Matrix3d _unfilledInertia;
            Eigen::Matrix3d unfilledInertia();
            Eigen::Matrix3d _filledInertia;
            Eigen::Matrix3d filledInertia();
        public:
            //constructor for a filled cylinder
            Cylinder(double radius, double length);
            //constructor for a hollow cylinder
            Cylinder(double radius, double length, double thickness);

            // inheritance from axisymmetric shape
            double radius(double x); // gets the radius of the shape at distance x from the origin along the axis of symmetry
            double radius(); //gets the base radius of this shape, the base radius is used to construct this shape
            void setRadius(double radius); // sets the base radius for the shape

            // inheritance from hollow shape
            // redeclaring for caching
            void setThickness(double thickness);
            // double thickness();
            void setLength(double length);
            // double length();

    };
}

#endif