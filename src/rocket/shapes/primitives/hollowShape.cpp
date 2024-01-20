#include "hollowShape.hpp"

namespace Shapes{
    double HollowShape::volume(){
        return filled() ? filledVolume() : unfilledVolume();
    }
    Eigen::Matrix3d HollowShape::inertia(){
        return filled() ? filledInertia() : unfilledInertia();
    }

    // the reference coordinates for a shape are its "center of mass" and its origin
    // by default these will be the same unless otherwise mentioned
    Eigen::Vector3d HollowShape::cm(){
        return filled() ? filledCm() : unfilledCm();
    }

    // similer to shape, cm is the origin unless otherwise mentioned
    Eigen::Vector3d HollowShape::unfilledCm(){
        return Eigen::Vector3d::Zero();
    }
    Eigen::Vector3d HollowShape::filledCm(){
        return Eigen::Vector3d::Zero();
    }

    double HollowShape::thickness(){
        return _thickness;
    }
    void HollowShape::setThickness(double thickness){
        if(thickness >= 0){
            _thickness = thickness;
        } else {
            //TODO: raise warning here
            _thickness = 0;
        }
    }

    // getter and setter for if the shape is filled
    bool HollowShape::filled(){
        return _filled;
    }
    void HollowShape::setFilled(bool filled){
        _filled = filled;
    }
}