#ifndef BODY_COMPONENT_H_
#define BODY_COMPONENT_H_

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <map>
#include <memory>
#include "uuid_v4.h"

#include "aeroComponent.hpp"
#include "internalComponent.hpp"
#include "externalComponent.hpp"
#include "stage.hpp"
#include "../shapes/components/bodyComponentShape.hpp"




namespace Rocket{
    // set up component like its an external component, then set getters for all aero and other relevant properties to 0
    // create InternalComponent subclass that does this
    // composite pattern time
    class BodyComponent: public AeroComponent{
        private:
            static std::string defaultName;
            std::unique_ptr<Shapes::BodyComponentShape> _shape;
            std::weak_ptr<Stage> _parent = std::shared_ptr<Stage>(nullptr);
        protected:
            std::vector<std::shared_ptr<InternalComponent>> _internalComponents;
            std::vector<std::shared_ptr<ExternalComponent>> _externalComponents;
            virtual void clearCaches();

            const float bodyLiftConst = 1.1;
            // tbh body lift doesn't really need caching
            virtual double calculateBodyLift( double alpha ) const;
            std::map<double, double> _bodyLiftCache = {};

            virtual double filledVolume() const;
            virtual Eigen::Vector3d bodyLiftCp() const;
            virtual double c_n_aWithoutBodyLift( double mach, double alpha, double gamma = 1.4 ) const;
            virtual Eigen::Vector3d cpWithoutBodyLift() const;

            double calculateC_n_a( double mach, double alpha, double gamma = 1.4 ) const override;
            double calculateC_m_a( double mach, double alpha, double gamma = 1.4 ) const override;
            Eigen::Vector3d calculateCp( double mach, double alpha, double gamma = 1.4 ) const override;

            virtual double c_m_damp_Func(double length, double avgRadius) const;
            virtual double calculateC_m_damp(double x) const override; // same formula for body comps

            virtual double calculateCdfA(const double mach, const double reL) const override;

            BodyComponent(
                std::unique_ptr<Shapes::BodyComponentShape> shape, std::unique_ptr<Material> material,
                std::unique_ptr<Finish> finish, std::string name, Eigen::Vector3d position
                );
        public:
            // implementing tree stuff
            virtual Stage* parent() const override;
            virtual void setParent( Component* parent ) override;

            virtual std::vector< std::shared_ptr<AeroComponent> > aeroComponents() const override;
            virtual std::vector< std::shared_ptr<Component> > components() const override;
            virtual std::shared_ptr<Component> findComponent(std::string id) const override;
            virtual void addComponent(Component* component) override;
            virtual void removeComponent(Component* component) override;
            
            // redefining shape functions
            virtual Shapes::BodyComponentShape* shape() const override;
            virtual void setShape( std::unique_ptr<Shapes::AeroComponentShape> shape ) override;
            virtual void setShape( std::unique_ptr<Shapes::BodyComponentShape> shape ); // func for correct typing

            // getter and setter for shape length
            virtual double length() const;
            virtual void setLength( double length); // CLEAR CACHES
            // getter and setter for shape radius
            virtual double radius() const;
            virtual double radius( double x ) const;
            virtual void setRadius( double radius ); // CLEAR CACHES
            // getter and setter for shape thickness
            virtual double thickness() const;
            virtual void setThickness( double thickness ); // CLEAR CACHES
            // area
            virtual double area(double x) const;

            // body lift
            virtual double bodyLift( double alpha ) const;
            // reference length
            virtual double referenceLength() const;
            virtual double bodyRadius(double x) const;
            virtual std::array<double,2> bisectedAverageRadius(double x) const; //returns [from top, to bottom]
            virtual double averageRadius() const; // required for damping

            virtual double calculateLowestPoint() const override;

            virtual double finenessRatio() const override;
    };

}

#endif