// THIS IS A TEST CLASS
#include "numericalNoseconeShape.hpp"
#include <string>
#include <memory>


namespace Shapes{
    class NoseconeShapeFactory{
        public:
            static std::unique_ptr<NoseconeShape> create(std::string type, double radius, double length, double thickness, double shapeParam);
    };

    class HaackNoseconeShape : public NumericalNoseconeShape{
        public:
            HaackNoseconeShape(double radius, double length, double thickness, double shapeParam);

            virtual Eigen::ArrayXd radius(Eigen::ArrayXd exes);
            virtual double radius();
    };
}