# ifndef CYLINDER_H_
# define CYLINDER_H_

#include "hollowAxisymmetricShape.hpp"
#include <Eigen/Dense>
#include <cmath>

namespace Shapes{
    class Cylinder: public HollowAxisymmetricShape{
        private:
            double _radius;
            double _length;
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
            double radius(double x) override; // gets the radius of the shape at distance x from the origin along the axis of symmetry
            double radius() override; //gets the base radius of this shape, the base radius is used to construct this shape
            void setRadius(double radius) override; // sets the base radius for the shape
            void setLength(double length) override;
            double length() override;

            // inheritance from hollow shape
            // redeclaring setters for caching
            void setThickness(double thickness) override;
            // getter is inherited

    };
}

#endif