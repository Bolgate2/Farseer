#pragma once
#include "../base/component.hpp"
#include "hollowCylinder.hpp"

namespace Rocket{

class BodyTube : public Component<std::tuple<>,AerodynamicCalculator<BodyTube>, KinematicCalculator<BodyTube>>{
    private:
        HollowCylinder _shape;
    public:
        BodyTube(HollowCylinder cyl);
        HollowCylinder shape(){ return _shape; }
};

}