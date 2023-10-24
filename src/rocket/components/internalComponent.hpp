# ifndef INTERNAL_COMPONENT_H_
# define INTERNAL_COMPONENT_H_

#include "component.hpp"
#include <vector>

namespace Rocket{
    class InternalComponent: public Component{
        private:
            // internal components can only have internal components as children
            std::vector<InternalComponent*> _components;
        public:

    };
}

# endif