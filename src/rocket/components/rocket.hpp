#ifndef ROCKET_H_
#define ROCKET_H_

#include <vector>
#include <string>
#include "uuid_v4.h"
#include "component.hpp"
#include "../misc/launchConditions.hpp"
#include "rocketInterface.hpp"

namespace Rocket{
    class Rocket: public Sim::RocketInterface, public Component{
        private:
            //hold pointers rather than the objects for speed (passing pointers around is faster than passing objects)
            LaunchConditions* _launchConditions;
        public:
            std::string _name;
            // constructors
            Rocket();
            // destructors
            ~Rocket();
            LaunchConditions* launchConditions(){ return _launchConditions; }
    };
}

#endif