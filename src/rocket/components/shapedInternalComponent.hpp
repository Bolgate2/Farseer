# ifndef SHAPED_INTERNAL_COMPONENT_H_
# define SHAPED_INTERNAL_COMPONENT_H_

#include "internalComponent.hpp"
#include "../shapes/shape.hpp"
#include  <memory>

namespace Rocket{
    class ShapedInternalComponent : public InternalComponent {
        private:
            std::unique_ptr<Shapes::Shape> _shape;
        public:
            virtual double calculateMass(double time) override;
            virtual Eigen::Matrix3d calculateInertia(double time) override;
            virtual Eigen::Vector3d calculateCm(double time) override;
    };
}

# endif
