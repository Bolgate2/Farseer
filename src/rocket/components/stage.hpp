#ifndef STAGE_H_
#define STAGE_H_

#include <vector>
#include "uuid_v4.h"
#include "aeroComponent.hpp"
#include "rocket.hpp"

namespace Rocket{
    class BodyComponent; // forward declaring so body components can use this as parent

    class Stage : public AeroComponent{
        private:
            static std::string defaultName;
            std::vector<std::shared_ptr<BodyComponent>> _components; // a stage may only have body components
            std::weak_ptr<Rocket> _parent;
        protected:
            // physical functions
            virtual double calculateMass(double time) const override { return 0; }
            virtual Eigen::Matrix3d calculateInertia(double time) const override { return Eigen::Matrix3d::Zero(); }
            virtual Eigen::Vector3d calculateCm(double time) const override { return Eigen::Vector3d::Zero(); }
            // aero functions
            virtual double calculateC_n_a( double mach, double alpha, double gamma = 1.4 ) const override { return 0; }
            virtual double calculateC_m_a( double mach, double alpha, double gamma = 1.4 ) const override { return 0; }
            virtual Eigen::Vector3d calculateCp( double mach, double alpha, double gamma = 1.4 ) const override { return Eigen::Vector3d::Zero(); }
            virtual double calculateC_m_damp(double x, double omega, double v) const override { return 0; }
            // constructor
            Stage(std::string name);
        public:
            /*
            [[nodiscard]]
            static std::shared_ptr<Stage> create(Rocket* parent = nullptr, std::string name = "Stage");
            */
            virtual Rocket* parent() const override;
            virtual void setParent( Component* parent ) override;

            virtual std::vector< std::shared_ptr<AeroComponent> > aeroComponents() const override;
            virtual std::vector<std::shared_ptr<Component>> components() const override;
            virtual std::shared_ptr<Component> findComponent(std::string id) const override;
            virtual void addComponent(Component* component) override;
            virtual void removeComponent(Component* component) override;

            
            // neutering functions that don't apply to the stage
            virtual double bodyRadius(double x){ return 0; }
            virtual Finish* finish() { return nullptr; }
            virtual void setFinish( std::unique_ptr<Finish> finish ) {/*do nothing*/}
            virtual Material* material() const { return 0; }
            virtual void setMaterial( std::unique_ptr<Material> material ) {/*do nothing*/}
            virtual double referenceArea() const;
            virtual double wettedArea() const; // surface area of the shape exposed to the air
            virtual double planformArea() const;
            virtual Eigen::Vector3d planformCenter() const; // geometric center of the shapes planform
    };
}

#endif