#ifndef SIMULATION_H_
#define SIMULATION_H_

#include "rocketInterface.hpp"
#include "stateArray.hpp"
#include "RealAtmos.hpp"
#include "nanValues.hpp"
#include <memory>
#include <vector>
#include <Eigen/Dense>
#include <filesystem>

namespace Sim{

    class Sim{
        private:
            double _userStep;
            bool _takeoff;
            bool _onRod;
            Eigen::Vector3d _rodVec;
            double _rodLen;
            RocketInterface* _rocket;
            Eigen::Matrix3d _rotmat; // the rotation matrix from the designs coords to the rockets coords
            double _aRef;
            double _lRef;

            RealAtmos::RealAtmos* _atmos;
            Sim(RocketInterface* rocket, double timeStep, std::filesystem::path destination);

            const Eigen::Array<double, 1, 6> RK_A = {0, 1.0/4, 3.0/8, 12.0/13, 1, 1.0/2 };
            const Eigen::Array<double, 6, 5> RK_B = {
                {0,             0,              0,              0,              0           },
                {1.0/4,         0,              0,              0,              0           },
                {3.0/32,        9.0/32,         0,              0,              0           },
                {1932.0/2197,   -7200.0/2197,   7296.0/2197,    0,              0           },
                {439.0/216,     -8,             3680.0/513,     -845/4014,      0           },
                {-8/27,         2,              -3544.0/2565,   1859.0/4104,    -11.0/40    }
            };
            const Eigen::Array<double, 1, 6> RK_CH = {16.0/135, 0, 6656.0/12825, 28561.0/56430, -9.0/50, 2.0/55};
            const Eigen::Array<double, 1, 6> RK_CT = {-1.0/360, 0, 128.0/4275, 2197.0/75240, -1.0/50, -2.0/55};

        public:
            std::filesystem::path saveFile;
            static std::shared_ptr<Sim> create( RocketInterface* rocket, double timeStep, std::filesystem::path destination);
            // defining up
            inline Eigen::Vector3d thisWayUp() const { return Eigen::Vector3d{0,0,1}; }

            inline const Eigen::Vector3d rodVec() const {
                return _rodVec;
            }

            inline void setRodVec(Eigen::Vector3d vec){
                _rodVec = vec;
            }

            //getters and setters
            inline const double userStep() const {
                return _userStep;
            }

            inline const bool takeoff() const {
                return _takeoff;
            }

            inline void setTakeoff( bool hasTakenOff ) {
                _takeoff = hasTakenOff;
            }

            inline const double rodLen() const {
                return _rodLen;
            }

            inline const bool onRod() const {
                return _onRod;
            }

            inline void setOnRod( bool isOnRod ) {
                _onRod = isOnRod;
            }

            inline const double referenceArea(){
                return _aRef;
            }

            inline const double referenceLength(){
                return _lRef;
            }

            // sim functions
            StateArray solve( StateArray initialConditions );

            /**
             * @brief Calculates the derivative of all the state vector fields
             * 
             * @param time The time thet they're calculated at
             * @param state The state vector of the rocket
             * @return StateVector 
             */
            std::tuple<StateArray, StepData> calculate( const double time, const StateArray state );

            /**
             * @brief Performs a single euler integration step on the calculation, returning the new state and its time
             * 
             * @param time time of the inputted state
             * @param step desired time step
             * @param state the state at the given time
             * @return std::tuple<double, StateArray> returns the new state and its associated time, any changes to step can be inferred from the returned time
             */
            std::tuple<double, StateArray, StepData> eulerIntegrate( const double time, const double step, const StateArray* state, const StateArray* lastState);

            static const std::tuple<StateArray, StepData> defK1arg;

            // using defaults from scipy ode
            //std::tuple<double, StateArray> adaptiveRKIntegrate( const double time, const double step, const StateArray state, const double rtol = 1e-3, const double atol = 1e-6);
            

            std::tuple<double, StateArray, StepData> RK4Integrate( const double time, const double step, const StateArray* state, const std::tuple<StateArray, StepData>* inK1Dat = &defK1arg);

            double selectTimeStep(const StateArray* state, const StateArray* lastState, const StateArray* k1, const double currStep) const;

            std::tuple<double, StateArray, StepData> AB4Integrate(
                const double time, const double step, const StateArray* state, const std::vector<StateArray>* diffs,
                const std::tuple<StateArray, StepData>* inK1Dat = &defK1arg
                );

            std::tuple<double, StateArray, StepData> ORKIntegrate( const double time, const double step, const StateArray* state, const StateArray* lastState);

            std::tuple<double, StateArray, StepData> ABRKIntegrate(
                const double time, const double step, const StateArray* state, const StateArray* lastState,
                const std::vector<StateArray>* diffs, const std::vector<double>* steps
                );

            // wind func
            Eigen::Vector3d wind(Eigen::Vector3d position) const;

            // helper functions
            Eigen::Vector3d originToCenterOfEarth() const;
            double altitude(Eigen::Vector3d position) const;
            Eigen::Vector3d centerOfEarthVector(Eigen::Vector3d position) const;

            std::filesystem::path outFile() const;
    };
}

#endif