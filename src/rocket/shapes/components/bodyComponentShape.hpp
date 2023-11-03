# ifndef BODY_COMPONENT_SHAPE_H_
# define BODY_COMPONENT_SHAPE_H_

#include "../primitives/hollowAxisymmetricShape.hpp"
#include "aeroComponentShape.hpp"
#include <memory>
#include <array>

namespace Shapes{

    // this exists for the shape in BodyComponent
    class BodyComponentShape: public AeroComponentShape{
        public:
            // getter and setter for shape
            virtual HollowAxisymmetricShape* shape() = 0;
            virtual void setShape( std::unique_ptr<Shape> shape ) = 0;
            // new function
            virtual double referenceLength() = 0;
            virtual double averageRadius() = 0; // required for damping
            virtual std::array<double,2> bisectedAverageRadius(double x) = 0; //returns [from top, to bottom]
            
            // getters and setters for HollowAxisymmetricShape
            virtual double radius(double x) { return shape()->radius(x); }
            virtual double radius(){ return shape()->radius(); } //gets the base radius of this shape, the base radius is used to construct this shape
            virtual void setRadius(double radius) { shape()->setRadius( radius ); } // sets the base radius for the shape
            virtual double area(double x){ return shape()->area(x); }
            // inheritance from hollow shape
            virtual void setThickness(double thickness){ shape()->setThickness( thickness ); }
            virtual double thickness(){ return shape()->thickness(); }
            virtual void setLength(double length){ shape()->setThickness( length ); }
            virtual double length(){ return shape()->length(); }
            virtual double filledVolume(){ return shape()->filledVolume(); }
    };
}

#endif