#pragma once
#include <cmath>

namespace Rocket{
class HollowCylinder{
    private:
        double _radius;
        double _height;
        double _thickness;
        bool _filled;
    public:
        HollowCylinder(double radius, double height, double thickness, bool filled = false);
        // getters and setters
        double radius();
        void setRadius(double radius);
        double height();
        void setHeight(double height);
        double thickness();
        void setThickness(double thickness);
        bool filled();
        void setFilled(bool filled);
        
        double volume();

};

}
