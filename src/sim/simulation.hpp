#ifndef SIMULATION_H
#define SIMULATION_H

#include "rocketInterface.hpp"
#include <memory>

namespace Sim{
    class Sim{
        private:
            std::shared_ptr<RocketInterface> _rocket;
    };
}

#endif