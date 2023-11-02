#ifndef NOSECONE_COMPONENT_SHAPE_H_
#define NOSECONE_COMPONENT_SHAPE_H_

#include "../bodyComponentShape.hpp"
#include "../../primitives/nosecone/noseconeShape.hpp"
#include "../../primitives/nosecone/noseconeShapeTypes.hpp"
#include <memory>
#include <string>

namespace Shapes{
    class NoseconeComponentShape: public BodyComponentShape{
        private:
            std::unique_ptr<NoseconeShape> _shape;
        public:
            NoseconeComponentShape( NoseconeShapeTypes type, double radius, double length, double thickness, double shapeParam );

            virtual NoseconeShape* shape() override;
            virtual void setShape( std::unique_ptr<Shape> shape ) override;

            virtual double referenceLength() override { return shape()->radius()*2; }
            virtual double wettedArea() override { return shape()->wettedArea(); }
            virtual double referenceArea() override { return shape()->referenceArea(); }
            virtual double planformArea() override { return shape()->referenceArea(); }
            virtual double planformCenter() override { return shape()->planformCenter(); }
            virtual Eigen::Vector3d cm() override { return shape()->cm(); }
    };
}


#endif