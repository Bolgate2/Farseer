#include "noseconeShapeTypes.hpp"
#include <string>
#include <memory>
#include <iostream>

namespace Shapes{
    std::unique_ptr<NoseconeShape> NoseconeShapeFactory::create(NoseconeShapeTypes type, double radius, double length, double thickness, double shapeParam){

        std::unique_ptr<NoseconeShape> ptr;
        switch(type){
            case NoseconeShapeTypes::HAACK:
                ptr = std::make_unique<HaackNoseconeShape>(radius, length, thickness, shapeParam);
                break;
            default:
                ptr = std::make_unique<HaackNoseconeShape>(radius, length, thickness, shapeParam);
        }
        return ptr;
    }

    // HAACK NOSECONE
    HaackNoseconeShape::HaackNoseconeShape(double radius, double length, double thickness, double shapeParam):
    NumericalNoseconeShape(radius, length, thickness, shapeParam) {}

    const NoseconeShapeTypes HaackNoseconeShape::_type = NoseconeShapeTypes::HAACK;

    NoseconeShapeTypes HaackNoseconeShape::type() const {
        return _type;
    }

    double HaackNoseconeShape::radius(){
        return NumericalNoseconeShape::radius();
    }
    

    Eigen::ArrayXd HaackNoseconeShape::radius(Eigen::ArrayXd exes){
        Eigen::ArrayXd theta = (1-2*exes/length()).acos();
        Eigen::ArrayXd y = radius()/std::sqrt(M_PI) * (theta-(2*theta).sin()/2+shapeParam()*theta.sin().pow(3)).sqrt();
        return y;
    }

}