#ifndef NOSECONE_SHAPE_TYPES_H_
#define NOSECONE_SHAPE_TYPES_H_

#include "numericalNoseconeShape.hpp"
#include <string>
#include <memory>


namespace Shapes{
    enum NoseconeShapeTypes{
        HAACK
    };

    class NoseconeShapeFactory{
        public:
            static std::unique_ptr<NoseconeShape> create(NoseconeShapeTypes type, double radius, double length, double thickness, double shapeParam);
    };

    class HaackNoseconeShape : public NumericalNoseconeShape{
        public:
            HaackNoseconeShape(double radius, double length, double thickness, double shapeParam);

            virtual Eigen::ArrayXd radius(Eigen::ArrayXd exes) override;
            virtual double radius() override;
    };
}

#endif