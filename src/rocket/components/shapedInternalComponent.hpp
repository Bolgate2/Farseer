# ifndef SHAPED_INTERNAL_COMPONENT_H_
# define SHAPED_INTERNAL_COMPONENT_H_

#include "internalComponent.hpp"
#include "../shapes/components/internalComponentShape.hpp"
#include  <memory>

namespace Rocket{
    class ShapedInternalComponent : public InternalComponent {
        private:
            std::unique_ptr<Shapes::InternalComponentShape> _shape;
        public:
            virtual double calculateMass(double time) const override;
            virtual Eigen::Matrix3d calculateInertia(double time) const override;
            virtual Eigen::Vector3d calculateCm(double time) const override;
    };
}

# endif
