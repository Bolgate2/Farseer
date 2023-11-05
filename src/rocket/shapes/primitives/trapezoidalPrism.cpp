#include "trapezoidalPrism.hpp"
#include "maths.hpp"
#include <Eigen/Dense>

namespace Shapes{
    TrapezoidalPrism::TrapezoidalPrism(double rootChord, double tipChord, double height, double sweepLength, double thickness){
        _rootChord = rootChord;
        _tipChord = tipChord;
        _height = height;
        _sweepLength = sweepLength;
        _thickness = thickness;
    }

    double TrapezoidalPrism::rootChord(){
        return _rootChord;
    }

    void TrapezoidalPrism::setRootChord(double length){
        _rootChord = std::max(length, 0.0);
        _inertia = NAN_M3D;
    }

    double TrapezoidalPrism::tipChord(){
        return _tipChord;
    }

    void TrapezoidalPrism::setTipChord(double length){
        _tipChord = std::max(length, 0.0);
        _inertia = NAN_M3D;
    }

    double TrapezoidalPrism::height(){
        return _height;
    }

    void TrapezoidalPrism::setHeight(double length){
        _height = std::max(length, 0.0);
        _inertia = NAN_M3D;
    }

    double TrapezoidalPrism::sweepLength(){
        return _sweepLength;
    }

    void TrapezoidalPrism::setSweepLength(double length){
        _sweepLength = length; //sweep length can be negative
        _inertia = NAN_M3D;
    }

    double TrapezoidalPrism::thickness(){
        return _thickness;
    }

    void TrapezoidalPrism::setThickness(double thickness){
        _thickness = std::max(thickness, 0.0);
        _inertia = NAN_M3D;
    }

    double TrapezoidalPrism::trapezoidArea(){
        return ( rootChord()+tipChord() )/2*height();
    }

    // inherited from shape
    double TrapezoidalPrism::volume(){
        return trapezoidArea()*thickness();
    }

    Eigen::Matrix3d TrapezoidalPrism::inertia(){
        if(!_inertia.hasNaN()) return _inertia;
        //derivation for this is in matlab, it is horrific
        auto h = height();
        auto t = thickness();
        auto lc = rootChord();
        auto ls = sweepLength();
        auto lt = tipChord();

        auto i_xy = std::pow(h,2) * t/24 * ( std::pow(lc,2) + 2*lc*lt + 2*ls*lc + 3*std::pow(lt,2) + 6*ls*lt );
        auto i_xx = h*std::pow(t,3)/24*(lc+lt) + std::pow(h,3)*t/12*(lc+3*lt);
        auto i_yy = h*std::pow(t,3)/24*(lc+lt) + h*t/12* (std::pow(lc,3) + std::pow(lc,2)*ls + std::pow(lc,2)*lt + lc*std::pow(ls,2) + 2*lc*ls*lt + lc*std::pow(lt,2) + 3*std::pow(ls,2)*lt + 3*ls*std::pow(lt,2) + std::pow(lt,3));
        auto i_zz = h*t/12*( std::pow(h,2)*lc + 3*std::pow(h,2)*lt + std::pow(lc,3) + std::pow(lc,2)*ls + std::pow(lc,2)*lt + lc*std::pow(ls,2) + 2*lc*ls*lt + lc*std::pow(lt,2) + 3*ls*std::pow(lt,2) + 3*lt*std::pow(ls,2) + std::pow(lt,3) );
        //          h*t/12*( h**2*lc          + 3*h**2*lt          + lc**3          + lc**2*ls          + lc**2*lt          + lc*ls**2          + 2*lc*ls*lt + lc*lt**2          + 3*ls*lt**2          + 3*lt*ls**2          + lt**3)
        Eigen::Matrix3d tens {
            {i_xx,  i_xy,   0   },
            {i_xy,  i_yy,   0   },
            {0,     0,      i_zz}
        };
        // correct up to here
        Eigen::Matrix3d cm_tens = Utils::parallel_axis_transform(tens, -cm(), volume());
        return cm_tens;
    }

    Eigen::Vector3d TrapezoidalPrism::cm(){
        // splitting the trapezoid into triangle, rectangle, triangle and finding the centroid
        auto l_t = tipChord();
        auto l_r = rootChord();
        auto l_s = sweepLength();
        auto l_rem = l_r - l_s - l_t; // length of the bottom triangle
        auto h = height();
        Eigen::Array3d x_bars   {l_s*2/3,   l_s+l_t/2,  l_s+l_t+l_rem/3}; // x centroids
        Eigen::Array3d y_bars   {h/3,       h/2,        h/3            }; // y centroids
        Eigen::Array3d areas    {l_s*h/2,   l_t*h,      l_rem*h/2      };
        areas = areas.abs();
        auto x = (x_bars*areas).sum()/areas.sum();
        auto y = (y_bars*areas).sum()/areas.sum();
        return {x,y,0};
    }
}