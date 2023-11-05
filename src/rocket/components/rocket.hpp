#ifndef ROCKET_H_
#define ROCKET_H_

#include <vector>
#include <string>
#include <memory>
#include "uuid_v4.h"
#include "aeroComponent.hpp"
#include "../misc/launchConditions.hpp"

namespace Rocket{
    class Stage; // forward declaring so stage can use this as parent
    class Rocket: public AeroComponent{
        private:
            //hold pointers rather than the objects for speed (passing pointers around is faster than passing objects)
            std::unique_ptr<LaunchConditions> _launchConditions;
            std::vector<std::shared_ptr<Stage>> _stages;
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
            // constructors
            Rocket(std::string name);
        public:
            // creator
            [[nodiscard]]
            static std::shared_ptr<Rocket> create(std::string name = "Rocket");
            LaunchConditions* launchConditions() const { return _launchConditions.get(); }
            // tree stuff
            // parent
            virtual Component* parent() const override { return nullptr; }
            virtual void setParent( Component* parent ) override {/*do nothing*/};
            // returns the root of this component tree
            virtual std::shared_ptr<Component> root() override { return shared_from_this(); }

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
            virtual Material* material()  const override { return 0; }
            virtual void setMaterial( std::unique_ptr<Material> material ) override {/*do nothing*/}
            virtual double planformArea() const override { return 0; }
            virtual Eigen::Vector3d planformCenter() const override { return Eigen::Vector3d::Zero(); } // geometric center of the shapes planform

            virtual double referenceArea() const override;
            virtual double wettedArea() const override; // surface area of the shape exposed to the air
            virtual double referenceLength() const override;
    };
}

#endif