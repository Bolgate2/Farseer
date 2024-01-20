#include "noseconeShape.hpp"
#include <math.h>

namespace Shapes{

    NoseconeShape::NoseconeShape(double radius, double length, double thickness, double shapeParam){
        _length = length;
        _radius = radius;
        _thickness = thickness;
        _shapeParam = shapeParam;
        setFilled(false);
    }

    void NoseconeShape::setShapeParam(double val){
        _shapeParam = val;
    }
    
    double NoseconeShape::shapeParam(){
        return _shapeParam;
    }

    void NoseconeShape::setLength(double length){
        _length = std::max(length, 0.0);
    }

    double NoseconeShape::length(){
        return _length;
    }

    void NoseconeShape::setRadius(double radius){
        _radius = std::max(radius, 0.0);
    }

    double NoseconeShape::radius(){
        return _radius;
    }

    double NoseconeShape::referenceArea(){
        return M_PI*std::pow(radius(),2);
    }
}