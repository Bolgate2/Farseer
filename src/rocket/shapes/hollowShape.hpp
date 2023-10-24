# ifndef HOLLOW_SHAPE_H_
# define HOLLOW_SHAPE_H_

#include "shape.hpp"
#include <Eigen/Dense>

namespace Shapes{
    // this interface exists for shapes that can be hollow
    class HollowShape: virtual public Shape{
        private:
            bool _filled;
        protected:
            double _thickness;
            // inertias if the shape is and isn't hollow
            virtual Eigen::Matrix3d unfilledInertia() = 0;
            virtual Eigen::Matrix3d filledInertia() = 0;
            // cms if the shape is and isn't filled
            virtual Eigen::Vector3d unfilledCm();
            virtual Eigen::Vector3d filledCm();
            // volumes if the shape is and isn't hollow
            virtual double unfilledVolume() = 0;
            virtual double filledVolume() = 0;
        public:
            // getter and setter for wall thickness
            virtual double thickness();
            virtual void setThickness(double thickness);

            // these functions are not virtual as they are final
            // getter and setter for if the shape is filled
            virtual bool filled();
            virtual void setFilled(bool filled);
            // derived methods
            virtual double volume(); // returns volume in m^3
            virtual Eigen::Matrix3d inertia(); // returns inertia tensor over density (assumes uniform density) about the origin

            // the reference coordinates for a shape are its "center of mass" and its origin
            // by default these will be the same unless otherwise mentioned
            virtual Eigen::Vector3d cm();
    };
}

# endif