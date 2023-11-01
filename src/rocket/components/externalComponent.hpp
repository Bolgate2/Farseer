#ifndef EXTERNAL_COMPONENT_H_
#define EXTERNAL_COMPONENT_H_

#include "aeroComponent.hpp"
#include <vector>

namespace Rocket{

    class ExternalComponent : public AeroComponent{
        private:
            std::vector< std::shared_ptr<AeroComponent> > _components;
        protected:
            // constructors
        public:
            // no create as this has not implemented RocketInterface
            // inherited methods
            virtual std::vector< std::shared_ptr<AeroComponent> > aeroComponents() override;
            virtual std::vector<std::shared_ptr<Component>> components() override;
            virtual std::shared_ptr<Component> findComponent(std::string id) override;
            virtual void addComponent(Component* component) override;
            virtual void removeComponent(Component* component) override;
            virtual double bodyRadius(double x) override;

            // new methods
    };
}

#endif