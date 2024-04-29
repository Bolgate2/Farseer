#ifndef INTEGRATOR_H_
#define INTEGRATOR_H_

#include "stateArray.hpp"
#include <tuple>

namespace Sim{
    class Integrator{
        public:
            virtual std::tuple<double, double, StateArray> integrate( double time, double step, StateArray state ) = 0;
    }
}

#endif