# include "bodyTube.hpp"
# include <iostream>
# include <memory>
# include <cmath>
# include <Eigen/Dense>

namespace Shapes{

    BodyTubeComponentShape::BodyTubeComponentShape(double radius, double length)
    {
        auto s = std::make_unique<Cylinder>(radius, length);
        setShape(std::move(s));
    }

    BodyTubeComponentShape::BodyTubeComponentShape(double radius, double length, double thickness){
        auto s = std::make_unique<Cylinder>(radius, length, thickness);
        setShape(std::move(s));
    }

    Cylinder* BodyTubeComponentShape::shape() {
        return _shape.get();
    }

    void BodyTubeComponentShape::setShape( std::unique_ptr<Shape> shape ){
        auto cylCast = dynamic_cast<Cylinder*>(shape.get());
        if( cylCast != NULL){
            auto newCylPointer = std::unique_ptr<Cylinder>(cylCast);
            shape.release();
            _shape = std::move(newCylPointer);
        } else {
            std::cerr << "Invalid shape type for BodyTubeComponentShape\n";
        }
    }

    double BodyTubeComponentShape::wettedArea(){
        return M_PI * 2 * radius() * length(); //diameter times length
    }

    double BodyTubeComponentShape::referenceArea(){
        return area(0);
    }

    double BodyTubeComponentShape::referenceLength(){
        return radius()*2; // diameter, ORK pg 16
    }

    double BodyTubeComponentShape::planformArea(){
        return 2 * radius() * length(); //diameter times length, used for body lift
    }

    double BodyTubeComponentShape::planformCenter(){
        return cm()[0]; // planform center is in the same spot as the center of mass
    }

    Eigen::Vector3d BodyTubeComponentShape::cm(){
        return Eigen::Vector3d{ length()/2, 0, 0 };
    }

    double BodyTubeComponentShape::averageRadius(){
        return radius();
    }
    std::array<double,2> BodyTubeComponentShape::bisectedAverageRadius(double x){
        if(x <= 0) return {radius(), 0};
        if(x >= length()) return {0, radius()};
        return {radius(), radius()};
    }
    
}