#pragma once
#include "../component.hpp"
#include "hollowCylinder.hpp"

namespace Rocket{

class BodyTube : public Component<std::tuple<>,AerodynamicCalculator<BodyTube>, KinematicCalculator<BodyTube>>{
    private:
        HollowCylinder _shape;
    protected:
    public:
        BodyTube(HollowCylinder cyl);
        HollowCylinder shape(){ return _shape; }
        //friend class ComponentConstructor;
};

}