#include "haackNoseconeShape.hpp"

namespace Shapes{
    HaackNoseconeShape::HaackNoseconeShape(double radius, double length, double thickness, double shapeParam):
    NumericalNoseconeShape(radius, length, thickness, shapeParam) {}

    double HaackNoseconeShape::radius(){
        return _radius;
    }

    Eigen::ArrayXd HaackNoseconeShape::radius(Eigen::ArrayXd exes){
        Eigen::ArrayXd theta = (1-2*exes/length()).acos();
        Eigen::ArrayXd y = radius()/std::sqrt(M_PI) * (theta-(2*theta).sin()/2+shapeParam()*theta.sin().pow(3)).sqrt(); // ide spits out err for radius func but it's wrong
        return y;
    }

}