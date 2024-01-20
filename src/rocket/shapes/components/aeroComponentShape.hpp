# ifndef AERO_SHAPE_H_
# define AERO_SHAPE_H_

#include "../primitives/shape.hpp"
#include "../primitives/aeroShapeFunctions.hpp"
#include <memory>
#include <Eigen/Dense>

namespace Shapes{
    class AeroComponentShape : public AeroShapeFunctions {
        public:
            // getter and setter for shape
            virtual Shape* shape() = 0;
            virtual void setShape( std::unique_ptr<Shape> shape ) = 0;

            // functions from the shape
            virtual double volume();
            virtual Eigen::Matrix3d inertia(); // returns inertia tensor over density (assumes uniform density) about cm
            // cm must be defined for a shape used on the rocket as the components origin and CM are rarely the same
            virtual Eigen::Vector3d cm() = 0;
    };

}

#endif