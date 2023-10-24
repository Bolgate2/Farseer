#ifndef NUMERICAL_NOSECONE_SHAPE_H
#define NUMERICAL_NOSECONE_SHAPE_H

#include "nosecone.hpp"

namespace Shapes{
    // this is a class for which only numerical solutions exist for the nosecones area, volume, etc e.g. Haack
    // the main difference here is that caching is used for the numerically calculated qualities to save time
    // this will remain abstract
    class NumericalNosecone : public Nosecone{
        protected:
            virtual Eigen::Vector3d unfilledCm() = 0;
            virtual Eigen::Vector3d filledCm() = 0;
        public:
            virtual double filledVolume() = 0; // changing scope of filled volume but keeping the function as pure virtual. This function is required for pitching moment
            virtual void setThickness(double thickness) = 0;
            virtual void setLength(double length) = 0; //sets the length of the shape

    };
}

#endif