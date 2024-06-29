# ifndef ROCKET_INTERFACE_H_
# define ROCKET_INTERFACE_H_

#include <vector>
#include <Eigen/Dense>

namespace Sim{

    /**
     * This class acts as a data class for transferring all of the data required for calculating aerodynamic coefficients
     * making this a dataclass means extending into more complex calculation is more easily maintainable
     */
    class FlightState{
        private:
            double _time;
            double _mach;
            double _alpha;
            double _gamma = 1.4;
            double _pitchVel;
            double _yawVel;
            double _reL;
        public:
            // getters, no setters as this is a data class
            double time(){return _time;}
            double mach(){return _mach;}
            double alpha(){return _alpha;}
            double gamma(){return _gamma;}
            double pitchVel(){return _pitchVel;}
            double yawVel(){return _yawVel;}
            double reL(){return _reL;}
            // constructor requires all fields except those with defaults
            FlightState(double time, double mach, double alpha, double pitchVel, double yawVel, double reL, double gamma = 1.4) :
            _time(time),
            _mach(mach),
            _alpha(alpha),
            _pitchVel(pitchVel),
            _yawVel(yawVel),
            _reL(reL),
            _gamma(gamma)
            {}
    };

    class RocketInterface{
        public:
            /**
             * @brief returns the vector created by traversing from the center of the base of the rocket to the tip of its nosecone. This is what is defined as "up"
             * 
             * @return Eigen::Vector3d 
             */
            virtual Eigen::Vector3d thisWayUp() = 0;


            /**
             * @brief Center of mass from the nosecone tip in body coordinates in meters
             * 
             * @param time Time since launch in seconds
             * @return Eigen::Vector3d cm as {x,y,z}
             */
            virtual Eigen::Vector3d cm(double time) = 0;

            /**
             * @brief Rockets inertia tensor about its center of mass in body coordinates in kg.meters^2
             * 
             * @param time Time since launch in seconds
             * @return Eigen::Matrix3d inertia tensor as
             * {
             * {i_xx i_xy i_xz},
             * {i_xy i_yy i_yz},
             * {i_xz i_yz i_zz}
             * }
             */
            virtual Eigen::Matrix3d inertia(double time) = 0;

            /**
             * @brief returns the mass of the rocket in kg
             * 
             * @param time Time since launch in seconds
             * @return double
             */
            virtual double mass(double time) = 0;

            /**
             * @brief returns the total thrust vector of the rocket in newtons
             * 
             * @param time Time since launch in seconds 
             * @return double 
             */
            virtual Eigen::Vector3d thrust(double time) = 0;
            
            /**
             * @brief returns the position of the total thrust vector relative to the nosecone tip in meters
             * 
             * @param time Time since launch in seconds 
             * @return Eigen::Vector3d 
             */
            virtual Eigen::Vector3d thrustPosition(double time) = 0;

            /**
             * @brief returns the reference area of the rocket in meters^2
             * 
             * @return double
             */
            virtual double referenceArea() = 0;

            /**
             * @brief returns the reference length of the rocket in meters
             * 
             * @return double 
             */
            virtual double referenceLength() = 0;

            /**
             * @brief The rockets normal force coefficient
             * 
             * @param mach Mach number the rocket is flying at
             * @param alpha Angle of attack of the rocket in radians
             * @param gamma Specific heat ratio for the fluid the rocket is moving through, 1.4 for air
             * @return double
             */
            virtual double c_n( double mach, double alpha, double gamma = 1.4 ) = 0;

            /**
             * @brief The rockets pitching moment coefficient
             * 
             * @param mach Mach number the rocket is flying at
             * @param alpha Angle of attack of the rocket in radians
             * @param gamma Specific heat ratio for the fluid the rocket is moving through, 1.4 for air
             * @return double 
             */
            virtual double c_m( double mach, double alpha, double gamma = 1.4) = 0;

            /**
             * @brief The location of the rockets center of pressure in meters in body coordinates
             * 
             * @param mach Mach number the rocket is flying at
             * @param alpha Angle of attack of the rocket in radians
             * @param gamma Specific heat ratio for the fluid the rocket is moving through, 1.4 for air
             * @return Eigen::Vector3d 
             */
            virtual Eigen::Vector3d cp( double mach, double alpha, double gamma = 1.4) = 0;

            /**
             * @brief The rockets pitching moment damping coefficient, these moments are applied to the total pitch moment in the opposite direction
             * 
             * @param x x location of the rockets CM from the nosecone tip
             * @param omega angular pitching velocity in body coordinates in radians/second
             * @param v freestream velocity in meters/second
             * @return double 
             */
            virtual double c_m_damp(double x, double omega, double v) = 0;

            // drag functions
            /**
             * @brief Frictional drag coefficient
             * 
             * @param mach Mach number
             * @param reL Reynolds number divided by length
             * @param alpha Angle of attack in radians
             * @return double 
             */
            virtual double Cdf(const double mach, const double reL, const double alpha) = 0;

            /**
             * @brief Pressure drag coefficient
             * 
             * @param mach Mach number
             * @param alpha Angle of attack in radians
             * @return double 
             */
            virtual double Cdp(const double mach, const double alpha) = 0;

            /**
             * @brief Base drag coefficient
             * 
             * @param mach Mach number
             * @param time Time since launch in seconds
             * @param alpha Angle of attack in radians
             * @return double 
             */
            virtual double Cdb(const double mach, const double time, const double alpha) = 0;
    };
}

#endif