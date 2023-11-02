#ifndef TRAPEZOIDAL_PRISM_H_
#define TRAPEZOIDAL_PRISM_H_

#include "shape.hpp"
#include "nanValues.hpp"

namespace Shapes{
    class TrapezoidalPrism: public Shape{
        // origin of this shape is the top left corner with the trap in this orientation -> D
        private:
            double _rootChord;
            double _tipChord;
            double _height;
            double _sweepLength;
            double _thickness;

            Eigen::Matrix3d _inertia = NAN_M3D; // clear this any time the above is changed
        public:
            TrapezoidalPrism(double rootChord, double tipChord, double height, double sweepLength, double thickness);

            virtual double rootChord();
            virtual void setRootChord(double length);

            virtual double tipChord();
            virtual void setTipChord(double length);

            virtual double height();
            virtual void setHeight(double length);

            virtual double sweepLength();
            virtual void setSweepLength(double length);

            virtual double thickness();
            virtual void setThickness(double thickness);

            // inherited from shape
            virtual double volume() override;
            virtual Eigen::Matrix3d inertia() override;
            virtual Eigen::Vector3d cm() override;
    };
}

#endif