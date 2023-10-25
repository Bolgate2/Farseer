# include "bodyTube.hpp"
# include <iostream>
# include <memory>
# include <cmath>

namespace Shapes{

    BodyTubeShape::BodyTubeShape(double radius, double length)
    {
        auto s = std::make_unique<Cylinder>(radius, length);
        setShape(std::move(s));
    }

    BodyTubeShape::BodyTubeShape(double radius, double length, double thickness){
        auto s = std::make_unique<Cylinder>(radius, length, thickness);
        setShape(std::move(s));
    }

    Cylinder* BodyTubeShape::shape() {
        //auto shp = _shape;
        return _shape.get();
    }

    void BodyTubeShape::setShape( std::unique_ptr<Shape> shape ){
        auto cylCast = dynamic_cast<Cylinder*>(shape.get());
        if( cylCast != NULL){
            auto newCylPointer = std::unique_ptr<Cylinder>(cylCast);
            shape.release();
            _shape = std::move(newCylPointer);
        } else {
            std::cerr << "Invalid shape type for BodyTubeShape\n";
        }
    }

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

    Eigen::Vector3d BodyTubeShape::cm(){
        return Eigen::Vector3d{ length()/2, 0, 0 };
    }
    
}