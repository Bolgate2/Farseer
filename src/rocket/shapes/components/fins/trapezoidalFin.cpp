#include "trapezoidalFin.hpp"
#include <memory>
#include <cmath>
#include <iostream>

namespace Shapes{
    TrapezoidalFinShape::TrapezoidalFinShape(double rootChord, double tipChord, double height, double sweepLength, double thickness){
        auto shp = std::make_unique<TrapezoidalPrism>(rootChord, tipChord, height, sweepLength, thickness);
        setShape(std::move(shp));
    }
    // fin component functions
    double TrapezoidalFinShape::chord(double y) {
        const auto lenRatio = y/height();
        //return std::lerp(rootChord(), tipChord(), lenRatio); // only available with c++20
        return (1-lenRatio)*rootChord()+lenRatio*tipChord();
    }

    double TrapezoidalFinShape::mac() {
        // raymer fig 4.15
        const auto taper = taperRatio();
        auto returnOfTheMac = (2/3)*rootChord()*(1+taperRatio()+std::pow(taperRatio(),2));
        return returnOfTheMac;
    }

    double TrapezoidalFinShape::yMac() {
        //ORK equation 3.33
        const auto root = rootChord();
        const auto tip = tipChord();
        auto returnOfTheMac = planformArea()/3*(root+2*tip)/(root+tip);
        return returnOfTheMac;
    }

    double TrapezoidalFinShape::xMacLeadingEdge() {
        // first part of ORK 3.34 RHS, second part is 0.25 MAC and so is not included
        return sweepLength()/3*(rootChord()+2*tipChord())/(rootChord()+2*tipChord());
    }

    double TrapezoidalFinShape::midChordSweep() {
        /*
        Fin like this
         _x
        |
        y
        (0,0)
        1
        |\ 
        | \ 
        |  \ 2
        |   |
        3\  |
          \ |
           \|
            4
        */
        if(height() <= 0) return 0; // avoiding div by 0 error
        const Eigen::Vector2d horizontal {1,0};
        Eigen::Vector2d meanLine { height(), (sweepLength()+tipChord()/2)-rootChord()/2 };

        double angle = std::acos( meanLine.dot(horizontal)/(meanLine.norm()*horizontal.norm()) );
        return angle;
    }

    double TrapezoidalFinShape::yMax() {
        return height();
    }

    // aero component functions
    TrapezoidalPrism* TrapezoidalFinShape::shape() {
        return _shape.get();
    }

    void TrapezoidalFinShape::setShape( std::unique_ptr<Shape> shape ) {
        auto trapzCast = dynamic_cast<TrapezoidalPrism*>(shape.get());
        if( trapzCast != NULL){
            auto newCylPointer = std::unique_ptr<TrapezoidalPrism>(trapzCast);
            shape.release();
            setShape(std::move(newCylPointer));
        } else {
            std::cerr << "Invalid shape type for trapezoidal fin shape\n";
        }
    }

    void TrapezoidalFinShape::setShape( std::unique_ptr<TrapezoidalPrism> shape ) {
        _shape = std::move(shape);
    }

    double TrapezoidalFinShape::wettedArea() {
        // vars used more than once
        const auto sweep = sweepLength();
        const auto thick = thickness();
        const auto h = height();
        const auto tip = tipChord();

        const auto faceArea = planformArea();
        const auto topEdgeArea = std::hypot( sweep, h )*thick;
        const auto tipEdgeArea = tip*thick;
        const auto bottomEdgeArea = std::hypot( rootChord()-sweep-tip, h )*thick;
        // inner edge is not exposed and is therefore not counted
        return 2*faceArea + topEdgeArea + tipEdgeArea + bottomEdgeArea;
    }

    double TrapezoidalFinShape::planformArea() {
        return shape()->trapezoidArea();
    }

    Eigen::Vector3d TrapezoidalFinShape::planformCenter() {
        const auto root = rootChord();
        const auto tip = tipChord();
        const auto sweep = sweepLength();
        return Eigen::Vector3d{(root/2 + (sweep+tip/2) )/2, height()/2, 0};
    }

}