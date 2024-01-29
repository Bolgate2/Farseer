#include "bodyTube.hpp"

namespace Rocket{

BodyTubeKinematicCalculator::BodyTubeKinematicCalculator(BodyTube* comp):
KinematicCalculator(comp)
{}

double BodyTubeKinematicCalculator::mass(double time){
    return getComponent()->shape()->height();
}

}