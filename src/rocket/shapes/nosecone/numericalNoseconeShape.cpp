#include "numericalNoseconeShape.hpp"
#include <vector>
#include <algorithm>
#include <iostream>

namespace Shapes{

    NumericalNoseconeShape::NumericalNoseconeShape(double radius, double length, double thickness, double shapeParam):
    NoseconeShape(radius, length, thickness, shapeParam)
    {}

    Eigen::Matrix3d NumericalNoseconeShape::unfilledInertia(){
        if(_unfilledInertia.hasNaN()) calculateProperties();
        return _unfilledInertia;
    }

    Eigen::Matrix3d NumericalNoseconeShape::filledInertia(){
        if(_filledInertia.hasNaN()) calculateProperties();
        return _filledInertia;
    }

    Eigen::Vector3d NumericalNoseconeShape::unfilledCm(){
        if(_unfilledCm.hasNaN()) calculateProperties();
        return _unfilledCm;
    }

    Eigen::Vector3d NumericalNoseconeShape::filledCm(){
        if(_filledCm.hasNaN()) calculateProperties();
        return _filledCm;
    }

    double NumericalNoseconeShape::unfilledVolume(){
        if(std::isnan(_unfilledVolume)) calculateProperties();
        return _unfilledVolume;
    }

    double NumericalNoseconeShape::filledVolume(){
        if(std::isnan(_filledVolume)) calculateProperties();
        return _filledVolume;
    }

    double NumericalNoseconeShape::wettedArea(){
        if(std::isnan(_wettedArea)) calculateProperties();
        return _wettedArea;
    }

    double NumericalNoseconeShape::planformArea(){
        if(std::isnan(_planformArea)) calculateProperties();
        return _planformArea;
    }

    double NumericalNoseconeShape::planformCenter(){
        if(std::isnan(_planformCenter)) calculateProperties();
        return _planformCenter;
    }

    void NumericalNoseconeShape::clearProperties(){
        _unfilledInertia = NAN_M3D;
        _filledInertia = NAN_M3D;
        _unfilledCm = NAN_V3D;
        _filledCm = NAN_V3D;
        _unfilledVolume = NAN_D;
        _filledVolume = NAN_D;
        _wettedArea = NAN_D;
        _planformArea = NAN_D;
        _planformCenter = NAN_D;
    }

    void NumericalNoseconeShape::setThickness(double thickness){
        NoseconeShape::setThickness(thickness);
        calculateProperties();
    }

    void NumericalNoseconeShape::setLength(double length){
        NoseconeShape::setLength(length);
        calculateProperties();
    }

    void NumericalNoseconeShape::setRadius(double radius){
        NoseconeShape::setRadius(radius);
        calculateProperties();
    }

    void NumericalNoseconeShape::setShapeParam(double val){
        NoseconeShape::setShapeParam(val);
        calculateProperties();
    }

    double NumericalNoseconeShape::radius(double x){
        Eigen::Array<double,1,1> valAsVec{x};
        Eigen::Array<double,1,1> radVec = radius(valAsVec);
        double rad = radVec[0];
        return rad;
    }

    void NumericalNoseconeShape::calculateProperties(){
        clearProperties();
        const int num_divs = 100;
        const double step = length()/(num_divs-1);
        Eigen::Array<double,1, num_divs> x;
        x.setEqualSpaced(0,step);

        Eigen::Array<double,1, num_divs> y = radius(x);
        // seqN creates a slice. fix sets the index at compile time which improves performance
        Eigen::ArrayXd r1 = y(Eigen::seqN( Eigen::fix<1>, Eigen::fix<num_divs-1>));
        Eigen::ArrayXd r2 = y(Eigen::seqN( Eigen::fix<0>, Eigen::fix<num_divs-1>));
        Eigen::ArrayXd stepArr = x(Eigen::seqN( Eigen::fix<1>, Eigen::fix<num_divs-1>)) - x(Eigen::seqN( Eigen::fix<0>, Eigen::fix<num_divs-1>));
        Eigen::ArrayXd hyp = ( (r1-r2).pow(2) + stepArr.pow(2) ).sqrt();
        Eigen::ArrayXd height = thickness()*hyp/step;

        // trapz(y,x) == ((r1+r2)/2*stepArr).sum()
        _wettedArea = ((hyp*(r1+r2))*M_PI).sum();
        _planformArea = ( ( r1+r2 )/2*stepArr ).sum()*2; //trapezoidal integration (then *2)
        _planformCenter = ( (r1+r2)*stepArr.pow(2) ).sum()/_planformArea;
        _filledVolume = ((( r1+r2 )/2).pow(2)*stepArr ).sum()*M_PI; // solid of revolution
        
        Eigen::ArrayXd dFullV = M_PI/3 * stepArr * (r1.pow(2) + r1*r2 + r2.pow(2));
        Eigen::ArrayXd dV = M_PI * stepArr * height * (r1+r2-height);
        dV = dV.cwiseMax(0);
        dV = ( r1 < height || r2 < height ).select(dFullV, dV);
        _unfilledVolume = dV.sum();

        // refactor to do filled as well
        Eigen::ArrayXd xAvg = x(Eigen::seqN( Eigen::fix<0>, Eigen::fix<num_divs-1>)) + step/2; // subtraction only implemented for arrays
        double cgX = (xAvg * dV).sum()/volume();
        _unfilledCm = Eigen::Vector3d{cgX, 0, 0};

        // do inertia later
    }

}