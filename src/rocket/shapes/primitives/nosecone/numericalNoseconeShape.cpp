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

    Eigen::Vector3d NumericalNoseconeShape::planformCenter(){
        if(_planformCenter.hasNaN()) calculateProperties();
        return _planformCenter;
    }

    double NumericalNoseconeShape::averageRadius(){
        if(std::isnan(_averageRadius)) calculateProperties();
        return _averageRadius;
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
        _planformCenter = NAN_V3D;
        _averageRadius = NAN_D;
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

    double NumericalNoseconeShape::radius(){
        return NoseconeShape::radius();
    }

    // cache this
    std::array<double,2> NumericalNoseconeShape::bisectedAverageRadius(double x){
        if(x <= 0) return { 0, averageRadius() };
        if(x >= length()) return {averageRadius(), 0};
        const int xFromTipElems = std::clamp( (int)std::round(x/length()*numDivs), 2, numDivs-2); // must be at least 2 elems
        const int xToBaseElems = numDivs-xFromTipElems;
        const double stepFromTip = x/xFromTipElems;
        const double stepToBase = (length()-x)/xToBaseElems;

        Eigen::ArrayXd xFromTip;
        xFromTip.setEqualSpaced(xFromTipElems, 0, stepFromTip);
        Eigen::ArrayXd xToBase;
        xToBase.setEqualSpaced(xToBaseElems, x, stepToBase);

        std::cout << xFromTip.transpose().size() << std::endl;

        std::cout << xToBase.transpose().size() << std::endl;

        Eigen::ArrayXd yFromTip = radius(xFromTip);
        Eigen::ArrayXd r1FromTip = yFromTip(Eigen::seqN( 1, xFromTipElems-1));
        Eigen::ArrayXd r2FromTip = yFromTip(Eigen::seqN( 0, xFromTipElems-1));
        double avgRadFromTip = ( (r1FromTip + r2FromTip)/2 ).mean();

        Eigen::ArrayXd yToBase = radius(xToBase);
        Eigen::ArrayXd r1ToBase = yToBase(Eigen::seqN( 1, xToBaseElems-1));
        Eigen::ArrayXd r2ToBase = yToBase(Eigen::seqN( 0, xToBaseElems-1));
        double avgRadToBase = ( (r1ToBase + r2ToBase)/2 ).mean();

        return { avgRadFromTip, avgRadToBase };
    }

    void NumericalNoseconeShape::calculateProperties(){
        clearProperties();
        const double step = length()/(numDivs-1);
        Eigen::Array<double,1, numDivs> x;
        x.setEqualSpaced(0,step);

        Eigen::Array<double, 1, numDivs> y = radius(x);

        Eigen::Array<double, 2, numDivs> radiiArray;

        _averageRadius = y.mean();

        // seqN creates a slice. fix sets the index at compile time which improves performance
        Eigen::ArrayXd r1 = y(Eigen::seqN( Eigen::fix<1>, Eigen::fix<numDivs-1>));
        Eigen::ArrayXd r2 = y(Eigen::seqN( Eigen::fix<0>, Eigen::fix<numDivs-1>));
        Eigen::ArrayXd stepArr = x(Eigen::seqN( Eigen::fix<1>, Eigen::fix<numDivs-1>)) - x(Eigen::seqN( Eigen::fix<0>, Eigen::fix<numDivs-1>));
        Eigen::ArrayXd hyp = ( (r1-r2).pow(2) + stepArr.pow(2) ).sqrt();
        Eigen::ArrayXd height = thickness()*hyp/step;

        // trapz(y,x) == ((r1+r2)/2*stepArr).sum()
        _wettedArea = ((hyp*(r1+r2))*M_PI).sum();
        _planformArea = ( ( r1+r2 )/2*stepArr ).sum()*2; //trapezoidal integration (then *2)
        
        Eigen::ArrayXd xAvg = (x(Eigen::seqN( Eigen::fix<0>, Eigen::fix<numDivs-1>)) + x(Eigen::seqN( Eigen::fix<1>, Eigen::fix<numDivs-1>)))/2;
        _planformCenter = Eigen::Vector3d{( (r1+r2)*stepArr*xAvg).sum()/_planformArea, 0, 0};

        std::cout << "Planform center [" << _planformCenter.transpose() << "]\n" << std::endl ;

        _filledVolume = ((( r1+r2 )/2).pow(2)*stepArr ).sum()*M_PI; // solid of revolution
        _filledCm = Eigen::Vector3d{_planformCenter.x(), 0, 0}; // same as the planform center as density is uniform
        
        Eigen::ArrayXd dFullV = M_PI/3 * stepArr * (r1.pow(2) + r1*r2 + r2.pow(2));
        Eigen::ArrayXd dV = M_PI * stepArr * height * (r1+r2-height);
        dV = dV.cwiseMax(0);
        dV = ( r1 < height || r2 < height ).select(dFullV, dV);
        _unfilledVolume = dV.sum();

        
        // unfilled Cm
        double cgX = (xAvg * dV).sum()/unfilledVolume();
        _unfilledCm = Eigen::Vector3d{cgX, 0, 0};

        // unfilled inertia
        Eigen::ArrayXd outer = (r1+r2)/2;
        Eigen::ArrayXd inner = (outer-height).cwiseMax(0);
        inner = ( r1 < height || r2 < height ).select(0, inner);
        
        double rotationalInertia = ( dV*(outer.pow(2)+inner.pow(2))/2 ).sum();
        double longitudinalInertia = ( dV*((3*(outer.pow(2)+inner.pow(2)) + stepArr.pow(2))/12 + xAvg.pow(2) ) ).sum();
        double i_xx = rotationalInertia;
        double i_yy = longitudinalInertia;
        double i_zz = longitudinalInertia;
        _unfilledInertia = Eigen::Matrix3d{
            {i_xx,  0,      0},
            {0,     i_yy,   0},
            {0,     0,      i_zz}
        };
        // filled inertia, just remove inner as it's 0 in the filled case
        rotationalInertia = ( dV*outer.pow(2)/2 ).sum();
        longitudinalInertia = ( dV*((3*outer.pow(2) + stepArr.pow(2))/12 + xAvg.pow(2) ) ).sum();
        i_xx = rotationalInertia;
        i_yy = longitudinalInertia;
        i_zz = longitudinalInertia;
        _filledInertia = Eigen::Matrix3d{
            {i_xx,  0,      0},
            {0,     i_yy,   0},
            {0,     0,      i_zz}
        };
    }

}