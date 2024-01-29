#include "hollowCylinder.hpp"
#include <numbers>

namespace Rocket{

HollowCylinder::HollowCylinder(double radius, double height, double thickness, bool filled){
    setRadius(radius);
    setHeight(height);
    setThickness(thickness);
    setFilled(filled);
}

double HollowCylinder::volume(){
    double vol = 0;
    if(filled() == true){
        vol = height() * std::numbers::pi * std::pow(radius(), 2);
    } else {
        vol = height() * std::numbers::pi * (2*radius()*thickness() - std::pow(thickness(),2));
    }
    return vol;
}

}
