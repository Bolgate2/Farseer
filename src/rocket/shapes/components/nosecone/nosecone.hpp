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
            // TODO: filled version without thickness

            virtual NoseconeShape* shape() override;
            virtual void setShape( std::unique_ptr<Shape> shape ) override;

            virtual Shapes::NoseconeShapeTypes type() {return shape()->type();}

            virtual double referenceLength() override { return shape()->radius()*2; }
            virtual double wettedArea() override { return shape()->wettedArea(); }
            virtual double referenceArea() override { return shape()->referenceArea(); }
            virtual double planformArea() override { return shape()->planformArea(); }
            virtual Eigen::Vector3d planformCenter() override { return shape()->planformCenter(); }
            virtual Eigen::Vector3d cm() override { return shape()->cm(); }

            virtual double shapeParam(){ return shape()->shapeParam(); }
            virtual void setShapeParam(double val){ return shape()->setShapeParam(val); }

            virtual double averageRadius() override { return shape()->averageRadius(); }
            virtual std::array<double,2> bisectedAverageRadius(double x) override { return shape()->bisectedAverageRadius(x); }

            virtual double maxSurfaceDistanceTravelled() { return shape()->maxSurfaceDistanceTravelled(); }
            virtual double calculateSurfaceDistanceTravelled(double x) { return shape()->calculateSurfaceDistanceTravelled(x); }
    };
}


#endif