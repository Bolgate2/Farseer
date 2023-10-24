#ifndef ROCKET_H_
#define ROCKET_H_

#include <vector>
#include <string>
#include "uuid_v4.h"
#include "aeroComponent.hpp"
#include "stage.hpp"
#include "../misc/launchConditions.hpp"

namespace Rocket{
    class Rocket: public AeroComponent{
        private:
            //hold pointers rather than the objects for speed (passing pointers around is faster than passing objects)
            LaunchConditions* _launchConditions;
            std::vector<Stage*> _stages;
        public:
            // constructors
            Rocket();
            // destructors
            LaunchConditions* launchConditions(){ return _launchConditions; }
    };
}

#endif