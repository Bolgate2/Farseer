#ifndef NOSECONE_SHAPE_H
#define NOSECONE_SHAPE_H

#include "../axisymmetricShape.hpp"
#include "../hollowShape.hpp"
#include "../aeroShapeInterface.hpp"

namespace Shapes{
    // this will remain abstract
    // THE NOSECONES ORIGIN IS AT ITS TIP, inertia is easier to calculate from here for numerical models and its cm is unpredictable
    class Nosecone: public HollowShape, public AxisymmetricShape, public AeroShapeInterface{
        // redefining some functions as pure virtual so that they must be implemented down the line
        protected:
            //
            double _shapeParam;
            // redefining these here as pure virtual as they must be redefined
            virtual Eigen::Vector3d unfilledCm() = 0;
            virtual Eigen::Vector3d filledCm() = 0;
        public:
            virtual double filledVolume() = 0; // changing scope of filled volume but keeping the function as pure virtual. This function is required for pitching moment
            virtual void setThickness(double thickness) = 0;
            virtual void setLength(double length) = 0; //sets the length of the shape

            //getter for shapeparam, concrete function
            
            //setter for shapeparam
    };
}

#endif
