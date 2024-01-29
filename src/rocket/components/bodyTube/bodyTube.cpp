#include <numbers>
#include <cmath>
#include "bodyTube.hpp"

namespace Rocket{

BodyTube::BodyTube(std::unique_ptr<HollowCylinder> cyl)
{
    _shape = std::move(cyl);
}

BodyTubeKinematicCalculator::BodyTubeKinematicCalculator(AbstractComponent* comp):
KinematicCalculator(comp)
{}

}