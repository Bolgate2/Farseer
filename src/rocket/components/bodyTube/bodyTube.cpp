#include "bodyTube.hpp"

namespace Rocket{

HollowCylinder::HollowCylinder(double radius, double height, double thickness, bool filled){
    this->radius = radius;
    this->height = height;
    this->thickness = thickness;
    this->filled = filled;
}

BodyTube::BodyTube(std::unique_ptr<HollowCylinder> cyl)
{
    _shape = std::move(cyl);
}

BodyTubeAerodynamicCalculator::BodyTubeAerodynamicCalculator(AbstractComponent* comp):
AerodynamicCalculator(comp)
{}

double BodyTubeAerodynamicCalculator::CnAlpha(double mach, double alpha){
    return 2;
}

BodyTubeKinematicCalculator::BodyTubeKinematicCalculator(AbstractComponent* comp):
KinematicCalculator(comp)
{}

}