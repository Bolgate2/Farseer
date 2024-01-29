#pragma once
#include "../component.hpp"
#include "hollowCylinder.hpp"

namespace Rocket{

class BodyTube;

class BodyTubeAerodynamicCalculator : public AerodynamicCalculator<BodyTube>{
    public:
        BodyTubeAerodynamicCalculator(BodyTube* comp);
        virtual double CnAlpha(double mach, double alpha) override;
};

class BodyTubeKinematicCalculator : public KinematicCalculator<BodyTube>{
    public:
        BodyTubeKinematicCalculator(BodyTube* comp);
        virtual double mass(double time) override;
};

class BodyTube : public Component<std::tuple<>,BodyTubeAerodynamicCalculator, BodyTubeKinematicCalculator>{
    private:
        std::unique_ptr<HollowCylinder> _shape = nullptr;
    protected:
        BodyTube(std::unique_ptr<HollowCylinder> cyl);
    public:
        HollowCylinder* shape(){ return _shape.get(); }
        friend class ComponentConstructor;
};





}