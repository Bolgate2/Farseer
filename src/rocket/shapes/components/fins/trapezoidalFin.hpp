#ifndef TRAPEZOIDAL_FIN_COMPONENT_SHAPE_H_
#define TRAPEZOIDAL_FIN_COMPONENT_SHAPE_H_

#include "fin.hpp"
#include "../../primitives/trapezoidalPrism.hpp"
#include <Eigen/Dense>
#include <memory>

namespace Shapes{

    class TrapezoidalFinShape: public FinComponentShape{
        private:
            std::unique_ptr<TrapezoidalPrism> _shape;
        public:
            TrapezoidalFinShape(double rootChord, double tipChord, double height, double sweepLength, double thickness);
            // fin component functions
            virtual double chord(double y) override; // returns the chord at the given point y from the root chord
            virtual double mac() override; // mean area chord
            virtual double xMacLeadingEdge() override;
            virtual double yMac() override;
            virtual double midChordSweep() override;
            virtual double yMax() override;
            // shape finctions
            
            // aero component functions
            virtual TrapezoidalPrism* shape() override;
            virtual void setShape( std::unique_ptr<Shape> shape ) override;
            virtual void setShape( std::unique_ptr<TrapezoidalPrism> shape );
            virtual double wettedArea() override; // surface area of the shape exposed to the air
            virtual double planformArea() override;
            virtual Eigen::Vector3d  planformCenter() override; // geometric center of the shapes planform

            // trapezoid functions
            virtual double rootChord() { return shape()->rootChord(); }
            
            // avoid root chord of 0
            virtual void setRootChord(double length) { if(length > 0) return shape()->setRootChord(length); }

            virtual double tipChord() { return shape()->tipChord(); }
            virtual void setTipChord(double length) { return shape()->setTipChord(length); }

            virtual double height() { return shape()->height(); }
            virtual void setHeight(double length) { return shape()->setHeight(length); }

            virtual double sweepLength() { return shape()->sweepLength(); }
            virtual void setSweepLength(double length) { return shape()->setSweepLength(length); }

            virtual double thickness() { return shape()->thickness(); }
            virtual void setThickness(double thickness) { return shape()->setThickness(thickness); }

            // new functions
            virtual double taperRatio() {return shape()->tipChord()/shape()->rootChord();}

    };

}


#endif