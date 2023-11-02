#include "noseconeShapeTypes.hpp"
#include <string>
#include <memory>
#include <iostream>

namespace Shapes{
    std::unique_ptr<NoseconeShape> NoseconeShapeFactory::create(std::string type, double radius, double length, double thickness, double shapeParam){
        std::string upper_type = "";
        for(auto ch = type.begin(); ch != type.end(); ch++){
            upper_type += std::toupper(*ch);
        }
        std::unique_ptr<NoseconeShape> ptr;
        if(upper_type == "HAACK"){
            std::cout << "hi" << std::endl;
            ptr = std::make_unique<HaackNoseconeShape>(radius, length, thickness, shapeParam);
        } else {
            ptr = std::make_unique<HaackNoseconeShape>(radius, length, thickness, shapeParam); // default nosecone
        }
        return ptr;
    }

    // HAACK NOSECONE
    HaackNoseconeShape::HaackNoseconeShape(double radius, double length, double thickness, double shapeParam):
    NumericalNoseconeShape(radius, length, thickness, shapeParam) {}

    double HaackNoseconeShape::radius(){
        return _radius;
    }

    Eigen::ArrayXd HaackNoseconeShape::radius(Eigen::ArrayXd exes){
        Eigen::ArrayXd theta = (1-2*exes/length()).acos();
        Eigen::ArrayXd y = radius()/std::sqrt(M_PI) * (theta-(2*theta).sin()/2+shapeParam()*theta.sin().pow(3)).sqrt();
        return y;
    }

}