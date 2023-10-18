# ifndef ROCKET_INTERFACE_H_
# define ROCKET_INTERFACE_H_

#include <vector>
#include <Eigen/Dense>

namespace Sim{
    class RocketInterface{
        public:
            // putting "=0" at the end of the virtual function makes it pure virtual, meaning it must be implemented
            virtual Eigen::Vector3d cm(double time) = 0; //center of mass
            virtual Eigen::Matrix3d inertia(double time) = 0;
            virtual double mass(double time) = 0;
    };
}

#endif