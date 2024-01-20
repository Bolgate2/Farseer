#ifndef SHAPE_H_
#define SHAPE_H_
#include <Eigen/Dense>

namespace Shapes{
    // because this class uses pure virtual functions only, it doesn't need an implementation (shape.cpp)
    class Shape{
        public:
            virtual double volume() = 0; // returns volume in m^3
            
            virtual Eigen::Matrix3d inertia() = 0; // returns inertia tensor over density (assumes uniform density) about cm

            // the reference coordinates for a shape are its "center of mass" and its origin
            // by default these will be the same unless otherwise mentioned
            virtual Eigen::Vector3d cm();
    };
}

#endif