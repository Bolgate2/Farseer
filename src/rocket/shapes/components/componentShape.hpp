#ifndef COMPONENT_SHAPE_H_
#define COMPONENT_SHAPE_H_

#include "../primitives/shape.hpp"
#include <memory>
#include <Eigen/Dense>

namespace Shapes{
    // this class stores and references a shape primitive
    class ComponentShape{
        virtual Shape* shape() = 0;
        virtual void setShape( std::unique_ptr<Shape> shape ) = 0;

        // functions from the shape
        virtual double volume();
        virtual Eigen::Matrix3d inertia(); // returns inertia tensor over density (assumes uniform density) about cm
        // cm must be defined for a shape used on the rocket as the components origin and CM are rarely the same
        virtual Eigen::Vector3d cm();
    };
}

#endif