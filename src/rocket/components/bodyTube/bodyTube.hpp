#pragma once
#include "../component.hpp"

namespace Rocket{
/*
class BodyTube : public Component{
    public:
        BodyTube();
        virtual void initAerodynamicCalculator() override;
};
*/

class HollowCylinder{
    private:
        double radius;
        double height;
        double thickness;
        bool filled;
    public:
        HollowCylinder(double radius, double height, double thickness, bool filled = false);
        double volume();
};

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