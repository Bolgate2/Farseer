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
        double radius(){ return _radius; }
        void setRadius(double radius){ _radius = std::max(radius, 0.0); }
        double height(){ return _height; }
        void setHeight(double height){ _height = std::max(height, 0.0); }
        double thickness(){ return _thickness; }
        void setThickness(double thickness){ _thickness = std::max(thickness, 0.0); }
        bool filled(){ return _filled; }
        void setFilled(bool filled){ _filled = filled; }

        double volume();

};

}
