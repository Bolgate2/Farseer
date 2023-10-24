#ifndef STAGE_H_
#define STAGE_H_

#include <vector>
#include "uuid_v4.h"
#include "aeroComponent.hpp"
#include "bodyComponent.hpp"

namespace Rocket{
    class Stage : public AeroComponent{
        private:
            std::vector<BodyComponent*> _components; // a stage may only have body components
        public:

    };
}

#endif