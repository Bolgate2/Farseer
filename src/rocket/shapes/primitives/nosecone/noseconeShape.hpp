#ifndef NOSECONE_SHAPE_H
#define NOSECONE_SHAPE_H

#include "../hollowAxisymmetricShape.hpp"
#include "../aeroShapeFunctions.hpp"
#include <Eigen/Dense>

namespace Shapes{
    // this will remain abstract
    // THE NOSECONES ORIGIN IS AT ITS TIP, inertia is easier to calculate from here for numerical models and its cm is unpredictable
    class NoseconeShape: public HollowAxisymmetricShape, virtual public AeroShapeFunctions{
        // redefining some functions as pure virtual so that they must be implemented down the line
        protected:
            double _shapeParam;
            double _radius; //base radius
            double _length;
            // redefining these here as pure virtual as they must be redefined as in hollowShape they are defined as the origin which isnt the case here (the tip is the origin)
            virtual Eigen::Vector3d unfilledCm() override = 0;
            virtual Eigen::Vector3d filledCm() override = 0;
        public:
            NoseconeShape(double radius, double length, double thickness, double shapeparam);
            virtual double filledVolume() override = 0; // changing scope of filled volume but keeping the function as pure virtual. This function is required for pitching moment

            virtual double shapeParam();
            virtual void setShapeParam(double val);

            // inherited from AeroShapeFunctions
            virtual double referenceArea() override;

            // inherited from axisymmetric shape
            virtual void setLength(double length) override;
            virtual double length() override;
            virtual void setRadius(double radius) override;
            virtual double radius(double x) override = 0;
            virtual double radius() override;

            virtual double averageRadius() = 0; // required for damping
            virtual std::array<double,2> bisectedAverageRadius(double x) = 0;
            
    };
}

#endif
