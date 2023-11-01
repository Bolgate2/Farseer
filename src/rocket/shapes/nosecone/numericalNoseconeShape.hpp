#ifndef NUMERICAL_NOSECONE_SHAPE_H
#define NUMERICAL_NOSECONE_SHAPE_H

#include "noseconeShape.hpp"
#include "nanValues.hpp"
#include "Eigen/Dense"
#include <map>

namespace Shapes{
    // this is a class for which only numerical solutions exist for the nosecones area, volume, etc e.g. Haack
    // the main difference here is that caching is used for the numerically calculated qualities to save time
    // this will remain abstract
    class NumericalNoseconeShape : public NoseconeShape{
        protected:
            // CACHED VALUES FOR CALCULATED PROPERTIES
            virtual void clearProperties();
            // inherited from hollowShape
            Eigen::Matrix3d _unfilledInertia = NAN_M3D;
            virtual Eigen::Matrix3d unfilledInertia() override;

            Eigen::Matrix3d _filledInertia = NAN_M3D;
            virtual Eigen::Matrix3d filledInertia() override;

            Eigen::Vector3d _unfilledCm = NAN_V3D;
            virtual Eigen::Vector3d unfilledCm();

            Eigen::Vector3d _filledCm = NAN_V3D;
            virtual Eigen::Vector3d filledCm();

            double _unfilledVolume = NAN_D;
            virtual double unfilledVolume() override;

            double _filledVolume = NAN_D;
            // filledVolume is public

            // inherited from aeroShapeInterface, all associated functions are public
            double _wettedArea = NAN_D;
            
            double _planformArea = NAN_D;
            double _planformCenter = NAN_D;


        public:
            NumericalNoseconeShape(double radius, double length, double thickness, double shapeParam);
            virtual Eigen::ArrayXd radius(Eigen::ArrayXd exes) = 0; // Each nosecone must implement this, vectorized radius calc
            virtual double radius(double x) override;
            void calculateProperties();

            // inherited from HollowShape
            virtual double filledVolume() override; // changing scope of filled volume but keeping the function as pure virtual. This function is required for pitching moment
            virtual void setThickness(double thickness) override;

            // inherited from aeroShapeInterface
            virtual double wettedArea() override; // surface area of the shape exposed to the air
            virtual double planformArea() override;
            virtual double planformCenter() override; // geometric center of the shapes planform

            // inherited from noseconeShape
            virtual void setLength(double length) override; //sets the length of the shape
            virtual void setRadius(double radius) override;
            virtual void setShapeParam(double val) override;
    };
}

#endif