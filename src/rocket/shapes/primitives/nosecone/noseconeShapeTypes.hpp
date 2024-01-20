#ifndef NOSECONE_SHAPE_TYPES_H_
#define NOSECONE_SHAPE_TYPES_H_

#include "numericalNoseconeShape.hpp"
#include <string>
#include <memory>


namespace Shapes{

    class NoseconeShapeFactory{
        public:
            static std::unique_ptr<NoseconeShape> create(NoseconeShapeTypes type, double radius, double length, double thickness, double shapeParam);
    };

    class HaackNoseconeShape : public NumericalNoseconeShape{
        private:
            static const NoseconeShapeTypes _type;
        public:
            HaackNoseconeShape(double radius, double length, double thickness, double shapeParam);

            virtual NoseconeShapeTypes type() const override;

            virtual Eigen::ArrayXd radius(Eigen::ArrayXd exes) override;
            virtual double radius() override;
    };
}

#endif