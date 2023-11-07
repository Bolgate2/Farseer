#ifndef SIMULATION_H_
#define SIMULATION_H_

#pragma once

#include "rocketInterface.hpp"
#include "stateArray.hpp"
#include <memory>
#include <vector>
#include <Eigen/Dense>

namespace Sim{

    class Sim{
        private:
            bool _takeoff;
            bool _onRod;
            RocketInterface* _rocket;
            Sim(RocketInterface* rocket);

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
            static std::shared_ptr<Sim> create( RocketInterface* rocket );


           StateArray solve( StateArray initialConditions );

            /**
             * @brief Calculates the derivative of all the state vector fields
             * 
             * @param time The time thet they're calculated at
             * @param state The state vector of the rocket
             * @return StateVector 
             */
            StateArray calculate( double time, StateArray state );

            /**
             * @brief Performs a single euler integration step on the calculation, returning the new state and its time
             * 
             * @param time time of the inputted state
             * @param step desired time step
             * @param state the state at the given time
             * @return std::tuple<double, StateArray> returns the new state and its associated time, any changes to step can be inferred from the returned time
             */
            std::tuple<double, StateArray> eulerIntegrate( const double time, const double step, const StateArray state);

            // using defaults from scipy ode
            std::tuple<double, StateArray> adaptiveRKIntegrate( const double time, const double step, const StateArray state, const double rtol = 1e-3, const double atol = 1e-6);

            std::tuple<double, StateArray> RK4Integrate( const double time, const double step, const StateArray state);

    };
}

#endif