# ifndef INTERNAL_COMPONENT_H_
# define INTERNAL_COMPONENT_H_

#include "component.hpp"
#include <vector>
#include <memory>

namespace Rocket{
    class InternalComponent: public Component{
        // this class is fully defined
        private:
            // internal components can only have internal components as children
            static std::string defaultName;
            std::vector<std::shared_ptr<InternalComponent>> _components;
        protected:
            InternalComponent(std::string name = Component::defaultName, Eigen::Vector3d position = Eigen::Vector3d::Zero());
        public:
            [[nodiscard]] static std::shared_ptr<InternalComponent> create(
                Component* parent = nullptr, std::string name = Component::defaultName, Eigen::Vector3d position = Eigen::Vector3d::Zero()
                );

            virtual std::vector< std::shared_ptr<Component> > components() const override;
            virtual std::shared_ptr<Component> findComponent(std::string id) const override;
            virtual void addComponent(Component* component) override;
            virtual void removeComponent(Component* component) override;
    };
}

# endif