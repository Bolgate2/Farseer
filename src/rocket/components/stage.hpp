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
            [[nodiscard]]
            static std::shared_ptr<Stage> create(Rocket* parent = nullptr, std::string name = "Stage");

            // component tree functions            
            virtual Rocket* parent() const override;
            virtual void setParent( Component* parent ) override;
            virtual std::vector< std::shared_ptr<AeroComponent> > aeroComponents() const override;
            virtual std::vector<std::shared_ptr<Component>> components() const override;
            virtual std::shared_ptr<Component> findComponent(std::string id) const override;
            virtual void addComponent(Component* component) override;
            virtual void removeComponent(Component* component) override;

            
            // neutering functions that don't apply to the stage
            virtual Shapes::AeroComponentShape* shape() const override { return nullptr; }
            virtual void setShape( std::unique_ptr<Shapes::AeroComponentShape> shape ) override {/*do nothing*/}
            virtual double bodyRadius(double x) const override { return 0; }
            virtual Finish* finish() const override { return nullptr; }
            virtual void setFinish( std::unique_ptr<Finish> finish ) override {/*do nothing*/}
            virtual Material* material() const override { return 0; }
            virtual void setMaterial( std::unique_ptr<Material> material ) override {/*do nothing*/}
            virtual double planformArea() const { return 0; }
            virtual Eigen::Vector3d planformCenter() const { return Eigen::Vector3d::Zero(); } // geometric center of the shapes planform
            virtual double referenceArea() const override;
            virtual double referenceLength() const override;
            virtual double wettedArea() const override; // surface area of the shape exposed to the air
    };
}

#endif