// THIS IS A TEST CLASS
#include "numericalNoseconeShape.hpp"

namespace Shapes{
    class HaackNoseconeShape : public NumericalNoseconeShape{
        public:
            HaackNoseconeShape(double radius, double length, double thickness, double shapeParam);

            virtual Eigen::ArrayXd radius(Eigen::ArrayXd exes);
            virtual double radius();
    };
}