#ifndef FIN_COMPONENT_SHAPE_H_
#define FIN_COMPONENT_SHAPE_H_

#include "../externalComponentShape.hpp"
#include "../../primitives/trapezoidalPrism.hpp"
#include <Eigen/Dense>

namespace Shapes{

    class FinComponentShape: public ExternalComponentShape{
        // TODO, modify this to accomodate freeform fin types maybe (tbh no one uses them)
        public:
            // new functions
            virtual double chord(double y) = 0; // returns the chord at the given point y from the root chord
            virtual double mac() = 0; // mean area chord
            virtual double xMacLeadingEdge() = 0;
            virtual double yMac() = 0;
            virtual double midChordSweep() = 0;
            virtual double yMax() = 0; // distance to the outermost point of the fin from the root. used for aspect ratio
            // component functions
            // origin of the fin is the top left corner
            virtual Eigen::Vector3d cm() { return shape()->cm(); };
            // aero component functions
            virtual double referenceArea() override { return 1; }; // reference area is kind of meaningless for a fin. for aero stuff it'll use the reference area of its parent
            virtual double wettedArea() override = 0; // surface area of the shape exposed to the air
            virtual double planformArea() override = 0;
            virtual Eigen::Vector3d planformCenter() override = 0; // geometric center of the shapes planform
    };

}


#endif