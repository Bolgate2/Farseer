#include "hollowCylinder.hpp"
#include <numbers>

namespace Rocket{

HollowCylinder::HollowCylinder(){
    setRadius(0);
    setHeight(0);
    setThickness(0);
    setFilled(false);
}

HollowCylinder::HollowCylinder(double radius, double height, double thickness, bool filled){
    setRadius(radius);
    setHeight(height);
    setThickness(thickness);
    setFilled(filled);
}

// getters and setters
double HollowCylinder::radius(){ return _radius; }
void HollowCylinder::setRadius(double radius){ _radius = std::max(radius, 0.0); }

double HollowCylinder::height(){ return _height; }
void HollowCylinder::setHeight(double height){ _height = std::max(height, 0.0); }

double HollowCylinder::thickness(){ return _thickness; }
void HollowCylinder::setThickness(double thickness){ _thickness = std::max(thickness, 0.0); }

bool HollowCylinder::filled(){ return _filled; }
void HollowCylinder::setFilled(bool filled){ _filled = filled; }


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
