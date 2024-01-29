#include "bodyTube.hpp"

namespace Rocket{
/*
BodyTubeKinematicCalculator::BodyTubeKinematicCalculator(BodyTube* comp):
KinematicCalculator(comp)
{}
*/

template<>
double KinematicCalculator<BodyTube>::mass(double time){
    return getComponent()->shape().height();
}

template<>
double AerodynamicCalculator<AbstractComponent>::CnAlpha(double mach, double alpha){
    return 0;
}

template<>
double KinematicCalculator<AbstractComponent>::mass(double time){
    return 0;
}

}