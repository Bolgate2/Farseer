#ifndef COMPONENT_H_
#define COMPONENT_H_
#include <string>
#include <unordered_map>
#include <vector>
#include <Eigen/Dense>
#include "uuid_v4.h"
#include "overrideFlags.hpp"

namespace Rocket{

    using Utils::OverrideFlags;

    // this class template defines the bare minimum of what a component should do
    // the base class of allowed child and parent types can be defined here
    class Component{
        protected:
            static std::string defaultName;
            // all components use the same UUID generator to ensure uniqueness
            static UUIDv4::UUIDGenerator<std::mt19937_64> _uuidGenerator;
            std::string _id;
            Component* _parent = NULL;

            // a component must have a physical location
            Eigen::Vector3d _position;
            
            // all overrides are false by default
            // a component must have a mass more than or equal to zero and the ability to manually set that mass
            double _mass = 0;
            OverrideFlags _massOverride = OverrideFlags::NONE; // flags

            // because a component has mass, its center of mass must be defined (same as position for a point mass) and be able to be overridden
            Eigen::Vector3d _cm = Eigen::Vector3d::Zero();
            OverrideFlags _cmOverride = OverrideFlags::NONE; // flags

            // because a component has mass and a position, it must have inertia as well and the ability to override it
            Eigen::Matrix3d _inertia = Eigen::Matrix3d::Zero();
            OverrideFlags _inertiaOverride = OverrideFlags::NONE; // flags

            bool _caching = false;

        protected:
            virtual void clearCaches();
            // mass
            virtual double calculateMass(double time);
            virtual double calculateMassWithComponents(double time);
            virtual double calculateMassWithCache(double time);
            std::unordered_map<double,double> _massCache = {};

            /**
             * @brief Calculates the inertia of this object about (0,0,0)
             * 
             * @param time Flight time in seconds
             * @return Eigen::Matrix3d The inertia tensor in SI units
             */
            virtual Eigen::Matrix3d calculateInertia(double time);
            virtual Eigen::Matrix3d calculateInertiaWithComponents(double time);
            virtual Eigen::Matrix3d calculateInertiaWithCache(double time);
            std::unordered_map<double,Eigen::Matrix3d> _inertiaCache = {};

            /**
             * @brief Calculates the individual components center of mass in global coordinates
             * 
             * @param time 
             * @return Eigen::Vector3d 
             */
            virtual Eigen::Vector3d calculateCm(double time);
            virtual Eigen::Vector3d calculateCmWithComponents(double time);
            virtual Eigen::Vector3d calculateCmWithCache(double time);
            std::unordered_map<double,Eigen::Vector3d> _cmCache = {};

            virtual Eigen::Vector3d calculateThrust(double time);
            virtual Eigen::Vector3d calculateThrustWithComponents(double time);
            virtual Eigen::Vector3d calculateThrustWithCache(double time);
            std::unordered_map<double,Eigen::Vector3d> _thrustCache = {};

            virtual Eigen::Vector3d calculateThrustPosition(double time);
            virtual Eigen::Vector3d calculateThrustPositionWithComponents(double time);
            virtual Eigen::Vector3d calculateThrustPositionWithCache(double time);
            std::unordered_map<double,Eigen::Vector3d> _thrustPositionCache = {};
        public:
            // a component must have a name, this is publically accessible as it's not integral to any functions
            std::string name;
            // default constructor
            Component(Component* parent = NULL, std::string name = Component::defaultName, Eigen::Vector3d position = Eigen::Vector3d::Zero());

            // id
            virtual std::string id();

            // parent
            virtual Component* parent();
            void setParent( Component* parent );

            // components
            // these functions are pure virtual as the component vectors are not defined here
            
            virtual std::vector<Component*> components() = 0; // VIRTUAL
            virtual Component* findComponent(std::string id) = 0; // VIRTUAL
            virtual void addComponent(Component* component) = 0; // THIS MUST CLEAR CACHES, VIRTUAL
            virtual void removeComponent(Component* component) = 0; // THIS MUST CLEAR CACHES, VIRTUAL

            // this function is not virtual as it is essentially a convenience wrapper
            virtual void removeComponent(std::string id);

            // position
            virtual Eigen::Vector3d position();
            // this is not pure virtual as it has a default implementation that can be changed
            virtual void setPosition(Eigen::Vector3d pos); // THIS MUST CLEAR CACHES
            // these are not virtual as they are essentially convenience wrappers
            virtual void setPosition(double pos[3]);
            virtual void setPosition(double x, double y, double z);

            // override functions are not virtual as they are not overridden
            virtual double mass(double time); // wrapper for calculateMass that first handles overrides
            virtual void overrideMass(OverrideFlags flags); // THIS MUST CLEAR CACHES
            virtual void overrideMass(OverrideFlags flags, double value); // THIS MUST CLEAR CACHES
            virtual OverrideFlags massOverriden();
            virtual void setMass(double mass); // THIS MUST CLEAR CACHES
            
            // inertia
            // returns inertia about 0/0
            virtual Eigen::Matrix3d inertia(double time);
            virtual void overrideInertia(OverrideFlags flags); // THIS MUST CLEAR CACHES
            virtual void overrideInertia(OverrideFlags flags, Eigen::Matrix3d value); // THIS MUST CLEAR CACHES
            virtual OverrideFlags inertiaOverriden();
            virtual void setInertia(Eigen::Matrix3d inertia); // THIS MUST CLEAR CACHES

            // cm, center of mass
            virtual Eigen::Vector3d cm(double time);
            virtual void overrideCm(OverrideFlags flags); // THIS MUST CLEAR CACHES
            virtual void overrideCm(OverrideFlags flags, Eigen::Vector3d value); // THIS MUST CLEAR CACHES
            virtual OverrideFlags cmOverriden();
            virtual void setCm(Eigen::Vector3d cm); // THIS MUST CLEAR CACHES

            // thrust position
            virtual Eigen::Vector3d thrustPosition(double time);
            // thrust
            virtual Eigen::Vector3d thrust(double time);

            //getter and setter for caching
            virtual bool caching();
            virtual void setCaching( bool toCache );
            virtual void setAllCaching( bool toCache );

            // returns the root of this component tree
            virtual Component* root();
    };
}

#endif