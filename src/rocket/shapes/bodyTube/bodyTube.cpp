# include "bodyTube.hpp"
# include <cmath>

namespace Shapes{

    BodyTubeShape::BodyTubeShape(double radius, double length) : Cylinder(radius, length) {}
    BodyTubeShape::BodyTubeShape(double radius, double length, double thickness) : Cylinder(radius, length, thickness){}

    double BodyTubeShape::wettedArea(){
        return M_PI * 2 * radius() * length(); //diameter times length
    }

    double BodyTubeShape::referenceArea(){
        return area(0);
    }

    double BodyTubeShape::referenceLength(){
        return radius()*2; // diameter, ORK pg 16
    }

    double BodyTubeShape::planformArea(){
        return 2 * radius() * length(); //diameter times length, used for body lift
    }

    double BodyTubeShape::planformCenter(){
        return cm()[0]; // planform center is in the same spot as the center of mass
    }
    
}