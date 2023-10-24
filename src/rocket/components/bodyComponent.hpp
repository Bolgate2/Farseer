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


namespace Rocket{
    // set up component like its an external component, then set getters for all aero and other relevant properties to 0
    // create InternalComponent subclass that does this
    // composite pattern time
    class BodyComponent: public AeroComponent{
        protected:
            static std::string defaultName;
            virtual void clearCaches();
            std::vector<InternalComponent*> _internalComponents;
            std::vector<AeroComponent*> _externalComponents;
            BodyComponent* _parent = NULL;
            Shapes::BodyComponentShape* _shape;

            const float bodyLiftConst = 1.1;
            virtual double calculateBodyLift( double alpha );
            virtual double calculateBodyLiftWithCache( double alpha );
            std::map<double, double> _bodyLiftCache = {};

            virtual Eigen::Vector3d bodyLiftCp();

            virtual double c_m_damp_Func(double length, double avgRadius, double omega, double v );

            // component calcs
            double calculateMass(double time);

        public:
            // constructor
            BodyComponent(Material* material, Finish* finish,
                BodyComponent* parent = NULL, std::string name = BodyComponent::defaultName, Eigen::Vector3d position = Eigen::Vector3d::Zero());
            BodyComponent(Shapes::BodyComponentShape* shape, Material* material, Finish* finish,
                BodyComponent* parent = NULL, std::string name = BodyComponent::defaultName, Eigen::Vector3d position = Eigen::Vector3d::Zero());

            // implementing tree stuff
            virtual BodyComponent* parent();
            virtual std::vector<AeroComponent*> aeroComponents();
            virtual std::vector<Component*> components();
            virtual Component* findComponent(std::string id);
            virtual void addComponent(Component* component);
            virtual void removeComponent(Component* component);
            virtual double bodyRadius(double x);
            
            // redefining shape functions
            virtual Shapes::BodyComponentShape* shape();
            virtual void setShape( Shapes::AeroShape* shape );

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

    };

}

#endif