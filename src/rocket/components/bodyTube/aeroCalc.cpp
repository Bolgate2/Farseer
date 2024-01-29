#include "bodyTube.hpp"

namespace Rocket{

BodyTubeAerodynamicCalculator::BodyTubeAerodynamicCalculator(BodyTube* comp):
AerodynamicCalculator(comp)
{}

double BodyTubeAerodynamicCalculator::CnAlpha(double mach, double alpha){
    return getComponent()->shape()->volume();
}

}