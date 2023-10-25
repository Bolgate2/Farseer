#include "cylinder.hpp"
#include <Eigen/Dense>

#include <cmath>

#include "nanValues.hpp"

namespace Shapes{
    Cylinder::Cylinder(double radius, double length){
        setRadius(radius);
        setLength(length);
        setFilled(true);
    }
    Cylinder::Cylinder(double radius, double length, double thickness) {
        setRadius(radius);
        setLength(length);
        setFilled(false);
        setThickness(thickness);
    }

    double Cylinder::unfilledVolume(){
        return _length * M_PI * (2*_radius*_thickness - std::pow(_thickness,2));
    }
    
    double Cylinder::filledVolume(){
        
        return _length * M_PI * std::pow(_radius,2);
    }

    // inertias if the shape is and isn't hollow
    // cache these
    Eigen::Matrix3d Cylinder::unfilledInertia(){
        // this is correct, validated with solidworks
        if(!_unfilledInertia.hasNaN()) return _unfilledInertia;
        auto r_o = radius();
        auto r_i = radius()-thickness();
        auto vol = unfilledVolume();

        auto i_l = vol/2 * ( std::pow(r_o,2) + std::pow(r_i,2) ); //longitudinal inertia
        auto i_r = vol/12 * ( std::pow(length(), 2) + 3*( std::pow(r_o, 2) + std::pow(r_i, 2) ) ); //rotational inertia

        auto i_xx = i_l;
        auto i_yy = i_r;
        auto i_zz = i_r;
        Eigen::Matrix3d inertiaTensor {
            {i_xx,  0,      0},
            {0,     i_yy,   0},
            {0,     0,      i_zz}
        };
        _unfilledInertia = inertiaTensor;
        return inertiaTensor;
        
    }
    Eigen::Matrix3d Cylinder::filledInertia(){
        // this is correct, validated with solidworks
        if(!_filledInertia.hasNaN()) return _filledInertia;
        auto r_o = radius();
        auto vol = filledVolume();

        auto i_l = vol/2 * std::pow(r_o,2); //longitudinal inertia
        auto i_r = vol/12 * ( std::pow(length(), 2) + 3*std::pow(r_o, 2) ); //rotational inertia

        auto i_xx = i_l;
        auto i_yy = i_r;
        auto i_zz = i_r;
        Eigen::Matrix3d inertiaTensor {
            {i_xx,  0,      0},
            {0,     i_yy,   0},
            {0,     0,      i_zz}
        };
        _filledInertia = inertiaTensor;
        return inertiaTensor;
    }
    

    double Cylinder::radius(double x){
        return radius();
    }
    double Cylinder::radius(){
        return _radius;
    }
    void Cylinder::setRadius(double radius){
        _filledInertia = NAN_M3D;
        _unfilledInertia = NAN_M3D;

        if(radius < 0) radius = 0;
        _radius = radius;
    }

    double Cylinder::length(){
        return _length;
    }

    void Cylinder::setLength(double length){
        _filledInertia = NAN_M3D;
        _unfilledInertia = NAN_M3D;

        if(length < 0) length = 0;
        _length = length;
    }

    void Cylinder::setThickness(double thickness){
        // no need to change filledInertia here
        _unfilledInertia = NAN_M3D;

        if(thickness < 0) thickness = 0;
        _thickness = thickness;
    }
}