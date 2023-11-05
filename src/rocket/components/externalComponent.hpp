#ifndef EXTERNAL_COMPONENT_H_
#define EXTERNAL_COMPONENT_H_

#include "aeroComponent.hpp"
#include "../shapes/components/externalComponentShape.hpp"
#include <vector>

namespace Rocket{

    class ExternalComponent : public AeroComponent{
        private:
            std::vector< std::shared_ptr<AeroComponent> > _components;
            std::unique_ptr< Shapes::ExternalComponentShape > _shape;
        protected:
            // constructors
            ExternalComponent(
                std::unique_ptr<Shapes::ExternalComponentShape> shape, std::unique_ptr<Material> material,
                std::unique_ptr<Finish> finish, std::string name, Eigen::Vector3d position
                );
        public:
            // no create as this has not implemented RocketInterface
            // inherited methods
            virtual std::vector< std::shared_ptr<AeroComponent> > aeroComponents() override;
            virtual std::vector<std::shared_ptr<Component>> components() override;
            virtual std::shared_ptr<Component> findComponent(std::string id) override;
            virtual void addComponent(Component* component) override;
            virtual void removeComponent(Component* component) override;
            virtual double bodyRadius(double x) override;

            virtual Shapes::ExternalComponentShape* shape() override;
            virtual void setShape( std::unique_ptr<Shapes::AeroComponentShape> shape ) override;
            virtual void setShape( std::unique_ptr<Shapes::ExternalComponentShape> shape );

            // new methods
    };
}

#endif