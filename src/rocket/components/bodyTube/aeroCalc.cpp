#include "bodyTube.hpp"

namespace Rocket{


template<>
double AerodynamicCalculator<BodyTube>::CnAlpha(double mach, double alpha){
    return getComponent()->shape().volume();
}



}