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
        const auto r = rootChord();
        const auto t = tipChord();
        double MAC = (2.0/3.0)*( std::pow(r,2)+std::pow(t,2)+r*t )/(r+t);
        return MAC;
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
        const auto r = rootChord();
        const auto t = tipChord();
        return sweepLength()/3 * (r+2*t)/(r+t);
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
        //return 2*faceArea + topEdgeArea + tipEdgeArea + bottomEdgeArea; // ork only uses face area
        return planformArea()*2;
    }

    double TrapezoidalFinShape::planformArea() {
        return shape()->trapezoidArea();
    }

    Eigen::Vector3d TrapezoidalFinShape::planformCenter() {
        // splitting the trapezoid into triangle, rectangle, triangle, and finding the centroid
        /* like this (long ahh fin)
             |
            R|\
            o| \
            c|  \
            k|___\
            e|    |
            t|    |
             |____|
            h|\   |
            e| \  | 
            r|  \ |
            e|   \|
             |
        */
        const auto r = rootChord();
        const auto t = tipChord();
        const auto s = sweepLength();
        const auto h = height();
        const auto rem = r-s-t;
        auto xBars = Eigen::Array3d{
            s*2/3, s+t/2, s+t+rem/3
        };
        auto yBars = Eigen::Array3d{ h/3, h/2, h/3 };
        auto areas = Eigen::Array3d{ s*h/2, t*h, rem*h/2 };
        auto x = (xBars*areas).sum()/areas.sum();
        auto y = (yBars*areas).sum()/areas.sum();
        return Eigen::Vector3d{x,y,0};
    }

}