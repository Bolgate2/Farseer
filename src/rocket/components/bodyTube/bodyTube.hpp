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
        double _radius;
        double _height;
        double _thickness;
        bool _filled;
    public:
        HollowCylinder(double radius, double height, double thickness, bool filled = false);
        // getters and setters
        double radius(){ return _radius; }
        void setRadius(double radius){ _radius = std::max(radius, 0.0); }
        double height(){ return _height; }
        void setHeight(double height){ _height = std::max(height, 0.0); }
        double thickness(){ return _thickness; }
        void setThickness(double thickness){ _thickness = std::max(thickness, 0.0); }
        bool filled(){ return _filled; }
        bool setFilled(bool filled){ _filled = filled; }

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