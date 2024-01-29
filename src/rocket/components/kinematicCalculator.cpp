#include "kinematicCalculator.hpp"
#include "component.hpp"

namespace Rocket{

// kin calc
KinematicCalculator::KinematicCalculator(AbstractComponent* comp){
    component = comp->weak_from_this();
}

double KinematicCalculator::mass(double time){
    return 0;
}

}