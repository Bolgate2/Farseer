#include <numbers>
#include <cmath>
#include "bodyTube.hpp"

namespace Rocket{

HollowCylinder::HollowCylinder(double radius, double height, double thickness, bool filled){
    this->radius = radius;
    this->height = height;
    this->thickness = thickness;
    this->filled = filled;
}

double HollowCylinder::volume(){
    if(filled){
        return height * std::numbers::pi * std::pow(radius, 2);
    }
    return height * std::numbers::pi * (2*radius*thickness - std::pow(thickness,2));
}

BodyTube::BodyTube(std::unique_ptr<HollowCylinder> cyl)
{
    _shape = std::move(cyl);
}


BodyTubeAerodynamicCalculator::BodyTubeAerodynamicCalculator(BodyTube* comp):
AerodynamicCalculator(comp)
{}

double BodyTubeAerodynamicCalculator::CnAlpha(double mach, double alpha){
    return getComponent()->shape()->volume();
}

BodyTubeKinematicCalculator::BodyTubeKinematicCalculator(AbstractComponent* comp):
KinematicCalculator(comp)
{}

}