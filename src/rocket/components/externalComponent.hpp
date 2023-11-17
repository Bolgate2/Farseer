#ifndef EXTERNAL_COMPONENT_H_
#define EXTERNAL_COMPONENT_H_

#include "aeroComponent.hpp"
#include "../shapes/components/externalComponentShape.hpp"
#include <vector>

namespace Rocket{

    class ExternalComponent : public AeroComponent{
        private:
            std::weak_ptr<AeroComponent> _parent = std::shared_ptr<AeroComponent>(nullptr);
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
            virtual AeroComponent* parent() const override;
            virtual void setParent( Component* parent ) override;
            // inherited methods
            
            virtual std::vector< std::shared_ptr<AeroComponent> > aeroComponents() const override;
            virtual std::vector<std::shared_ptr<Component>> components() const override;
            virtual std::shared_ptr<Component> findComponent(std::string id) const override;
            virtual void addComponent(Component* component) override;
            virtual void removeComponent(Component* component) override;
            virtual double bodyRadius(double x) const override;

            virtual Shapes::ExternalComponentShape* shape() const override;
            virtual void setShape( std::unique_ptr<Shapes::AeroComponentShape> shape ) override;
            virtual void setShape( std::unique_ptr<Shapes::ExternalComponentShape> shape );

            virtual double finenessRatio() const override { return 1; } // realistically shouldn't be used in this context
    };
}

#endif