# ifndef BODY_TUBE_SHAPE_H_
# define BODY_TUBE_SHAPE_H_

#include "../../primitives/cylinder.hpp"
#include "../bodyComponentShape.hpp"
#include <memory>
#include <Eigen/Dense>

namespace Shapes{
    // includes must be virtual so that only one copy of shared parent classes is used
    class BodyTubeShape: public BodyComponentShape{
        private:
            std::unique_ptr<Cylinder> _shape;
        public:
            //constructor for a filled tube
            BodyTubeShape(double radius, double length);
            //constructor for a tube
            BodyTubeShape(double radius, double length, double thickness);

            // inherited from bodyComponentShape
            
            virtual Cylinder* shape() override;
            virtual void setShape( std::unique_ptr<Shape> shape ) override;
            virtual double referenceLength() override;
            virtual double wettedArea() override; // surface area of the shape exposed to the air
            virtual double referenceArea() override; // shapes reference area
            virtual double planformArea() override;
            virtual double planformCenter() override; // geometric center of the shapes planform
            // defining cm
            virtual Eigen::Vector3d cm() override;

    };
}

# endif