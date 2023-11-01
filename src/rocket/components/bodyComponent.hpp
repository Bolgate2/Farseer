#ifndef BODY_COMPONENT_H
#define BODY_COMPONENT_H

#include "aeroComponent.hpp"
#include "internalComponent.hpp"
#include "externalComponent.hpp"
#include "../shapes/bodyComponentShape.hpp"

#include <string>
#include <vector>
#include <Eigen/Dense>
#include "uuid_v4.h"
#include <map>
#include <memory>


namespace Rocket{
    // set up component like its an external component, then set getters for all aero and other relevant properties to 0
    // create InternalComponent subclass that does this
    // composite pattern time
    class BodyComponent: public AeroComponent{
        private:
            static std::string defaultName;
            std::unique_ptr<Shapes::BodyComponentShape> _shape;
            std::weak_ptr<BodyComponent> _parent = std::shared_ptr<BodyComponent>(nullptr);
        protected:
            std::vector<std::shared_ptr<InternalComponent>> _internalComponents;
            std::vector<std::shared_ptr<AeroComponent>> _externalComponents;
            virtual void clearCaches();

            const float bodyLiftConst = 1.1;
            virtual double calculateBodyLift( double alpha );
            virtual double calculateBodyLiftWithCache( double alpha );
            std::map<double, double> _bodyLiftCache = {};

            virtual Eigen::Vector3d bodyLiftCp();

            virtual double c_m_damp_Func(double length, double avgRadius, double omega, double v );

            // component calcs
            double calculateMass(double time);

            BodyComponent(
                std::unique_ptr<Shapes::BodyComponentShape> shape, std::unique_ptr<Material> material,
                std::unique_ptr<Finish> finish, std::string name, Eigen::Vector3d position
                );
        public:
            // no create method as this class is still abstract. RocketInterface methods have not been set

            // implementing tree stuff
            virtual std::vector< std::shared_ptr<AeroComponent> > aeroComponents() override;
            virtual std::vector< std::shared_ptr<Component> > components() override;
            virtual std::shared_ptr<Component> findComponent(std::string id) override;
            virtual void addComponent(Component* component) override;
            virtual void removeComponent(Component* component) override;
            
            // redefining shape functions
            virtual Shapes::BodyComponentShape* shape() override;
            virtual void setShape( std::unique_ptr<Shapes::AeroShape> shape ) override;
            virtual void setShape( std::unique_ptr<Shapes::BodyComponentShape> shape ); // func for correct typing

            // getter and setter for shape length
            virtual double length();
            virtual void setLength( double length); // CLEAR CACHES
            // getter and setter for shape radius
            virtual double radius();
            virtual double radius( double x );
            virtual void setRadius( double radius ); // CLEAR CACHES
            // getter and setter for shape thickness
            virtual double thickness();
            virtual void setThickness( double thickness ); // CLEAR CACHES
            // area
            virtual double area(double x);

            // body lift
            virtual double bodyLift( double alpha );
            // reference length
            virtual double referenceLength();
            virtual double bodyRadius(double x);
    };

}

#endif