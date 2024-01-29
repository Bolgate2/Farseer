#pragma once
#include "../component.hpp"
#include "hollowCylinder.hpp"

namespace Rocket{

class BodyTubeAerodynamicCalculator;
class BodyTubeKinematicCalculator;

class BodyTube : public Component<std::tuple<>,BodyTubeAerodynamicCalculator, BodyTubeKinematicCalculator>{
    private:
        std::unique_ptr<HollowCylinder> _shape = nullptr;
    protected:
        BodyTube(std::unique_ptr<HollowCylinder> cyl);
    public:
        HollowCylinder* shape(){ return _shape.get(); }
        friend class ComponentConstructor;
};

class BodyTubeAerodynamicCalculator : public AerodynamicCalculator<BodyTube>{
    public:
        BodyTubeAerodynamicCalculator(BodyTube* comp);
        virtual double CnAlpha(double mach, double alpha) override;
};

class BodyTubeKinematicCalculator : public KinematicCalculator{
    public:
        BodyTubeKinematicCalculator(AbstractComponent* comp);
};



}