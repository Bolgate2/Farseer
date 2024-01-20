# ifndef BODY_TUBE_COMPONENT_SHAPE_H_
# define BODY_TUBE_COMPONENT_SHAPE_H_

#include "../../primitives/cylinder.hpp"
#include "../bodyComponentShape.hpp"
#include <memory>
#include <Eigen/Dense>

namespace Shapes{
    // includes must be virtual so that only one copy of shared parent classes is used
    class BodyTubeComponentShape: public BodyComponentShape{
        private:
            std::unique_ptr<Cylinder> _shape;
        public:
            //constructor for a filled tube
            BodyTubeComponentShape(double radius, double length);
            //constructor for a tube
            BodyTubeComponentShape(double radius, double length, double thickness);

            // inherited from bodyComponentShape
            
            virtual Cylinder* shape() override;
            virtual void setShape( std::unique_ptr<Shape> shape ) override;
            virtual double referenceLength() override;
            virtual double wettedArea() override; // surface area of the shape exposed to the air
            virtual double referenceArea() override; // shapes reference area
            virtual double planformArea() override;
            virtual Eigen::Vector3d planformCenter() override; // geometric center of the shapes planform
            // defining cm
            virtual Eigen::Vector3d cm() override;

            virtual double averageRadius() override; // required for damping
            virtual std::array<double,2> bisectedAverageRadius(double x) override;

    };
}

# endif