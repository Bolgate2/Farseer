#ifndef EXTERNAL_COMPONENT_H_
#define EXTERNAL_COMPONENT_H_

#include "aeroComponent.hpp"
#include <vector>

namespace Rocket{

    class ExternalComponent : public AeroComponent{
        protected:
            std::vector<AeroComponent*> _components;
        public:
            // inherited methods
            virtual std::vector<AeroComponent*> aeroComponents();
            virtual std::vector<Component*> components(); // VIRTUAL
            virtual Component* findComponent(std::string id); // VIRTUAL
            virtual void addComponent(Component* component); // THIS MUST CLEAR CACHES, VIRTUAL
            virtual void removeComponent(Component* component); // THIS MUST CLEAR CACHES, VIRTUAL
            virtual double bodyDiameter(double x);

            // new methods
    };
}

#endif