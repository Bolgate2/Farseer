#ifndef COMPONENT_H_
#define COMPONENT_H_
#include <string>
#include <unordered_map>
#include <memory>
#include <vector>
#include <Eigen/Dense>
#include "uuid_v4.h"
#include "overrideFlags.hpp"

namespace Rocket{

    using Utils::OverrideFlags;

    // this class template defines the bare minimum of what a component should do
    // the base class of allowed child and parent types can be defined here
    class Component : public std::enable_shared_from_this<Component>{
        protected:
            static std::string defaultName;
            // all components use the same UUID generator to ensure uniqueness
            static UUIDv4::UUIDGenerator<std::mt19937_64> _uuidGenerator;
            std::string _id;
            std::weak_ptr<Component> _parent = std::shared_ptr<Component>(nullptr);

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
            
            virtual std::string componentTreeRepr( std::string prefix, std::string childPrefix, int treeHeight) const;
            // string formatting variables
            static int _indentLen;
            static int _maxNameLen;
            static int _massLen;
            static int _massPrecision;
        protected:
            virtual void clearCaches();
            // mass
            virtual double calculateMass(double time) const;
            virtual double calculateMassWithComponents(double time) const;
            std::unordered_map<double,double> _massCache = {};

            /**
             * @brief Calculates the inertia of this object about (0,0,0)
             * 
             * @param time Flight time in seconds
             * @return Eigen::Matrix3d The inertia tensor in SI units
             */
            virtual Eigen::Matrix3d calculateInertia(double time) const;
            virtual Eigen::Matrix3d calculateInertiaWithComponents(double time) const;
            std::unordered_map<double,Eigen::Matrix3d> _inertiaCache = {};

            /**
             * @brief Calculates the individual components center of mass in global coordinates
             * 
             * @param time 
             * @return Eigen::Vector3d 
             */
            virtual Eigen::Vector3d calculateCm(double time) const;
            virtual Eigen::Vector3d calculateCmWithComponents(double time) const;
            std::unordered_map<double,Eigen::Vector3d> _cmCache = {};

            virtual Eigen::Vector3d calculateThrust(double time) const;
            virtual Eigen::Vector3d calculateThrustWithComponents(double time) const;
            std::unordered_map<double,Eigen::Vector3d> _thrustCache = {};

            virtual Eigen::Vector3d calculateThrustPosition(double time) const;
            virtual Eigen::Vector3d calculateThrustPositionWithComponents(double time) const;
            std::unordered_map<double,Eigen::Vector3d> _thrustPositionCache = {};

            // default constructor
            // constructor is protected. It isn't private so that subclasses can access it. It isn't public as a factory is defined
            Component(std::string name, Eigen::Vector3d position);
        public:
            // a component must have a name, this is publically accessible as it's not integral to any functions
            std::string name;

            /*
            factory
            "best" example in cpp docs
            https://en.cppreference.com/w/cpp/memory/enable_shared_from_this
            setting the parent is handled here
            this is really just here as an example. only need to implement this for non abstract classes

            [[nodiscard]] static std::shared_ptr<Component> create(
                Component* parent = nullptr, std::string name = Component::defaultName, Eigen::Vector3d position = Eigen::Vector3d::Zero()
                );
            
            */

            // id
            virtual std::string id() const;

            // parent
            virtual Component* parent() const;
            virtual void setParent( Component* parent );
            // returns the root of this component tree
            virtual std::shared_ptr<Component> root();
            virtual int height() const;

            // components
            // these functions are pure virtual as the component vectors are not defined here
            virtual std::vector< std::shared_ptr<Component> > components() const = 0; // VIRTUAL
            virtual std::shared_ptr<Component> findComponent(std::string id) const = 0; // VIRTUAL
            virtual void addComponent(Component* component) = 0; // THIS MUST CLEAR CACHES, VIRTUAL
            virtual void removeComponent(Component* component) = 0; // THIS MUST CLEAR CACHES, VIRTUAL

            // this function is not virtual as it is essentially a convenience wrapper
            virtual void removeComponent( std::string id );

            // position
            virtual Eigen::Vector3d position() const;
            // this is not pure virtual as it has a default implementation that can be changed
            virtual void setPosition(Eigen::Vector3d pos); // THIS MUST CLEAR CACHES
            // these are not virtual as they are essentially convenience wrappers
            virtual void setPosition(double pos[3]);
            virtual void setPosition(double x, double y, double z);

            // override functions are not virtual as they are not overridden
            virtual double mass(double time) const; // wrapper for calculateMass that first handles overrides
            virtual void overrideMass(OverrideFlags flags); // THIS MUST CLEAR CACHES
            virtual void overrideMass(OverrideFlags flags, double value); // THIS MUST CLEAR CACHES
            virtual OverrideFlags massOverriden();
            virtual void setMass(double mass); // THIS MUST CLEAR CACHES
            
            // inertia
            // returns inertia about 0/0
            virtual Eigen::Matrix3d inertia(double time) const;
            virtual void overrideInertia(OverrideFlags flags); // THIS MUST CLEAR CACHES
            virtual void overrideInertia(OverrideFlags flags, Eigen::Matrix3d value); // THIS MUST CLEAR CACHES
            virtual OverrideFlags inertiaOverriden();
            virtual void setInertia(Eigen::Matrix3d inertia); // THIS MUST CLEAR CACHES

            // cm, center of mass
            virtual Eigen::Vector3d cm(double time) const;
            virtual void overrideCm(OverrideFlags flags); // THIS MUST CLEAR CACHES
            virtual void overrideCm(OverrideFlags flags, Eigen::Vector3d value); // THIS MUST CLEAR CACHES
            virtual OverrideFlags cmOverriden();
            virtual void setCm(Eigen::Vector3d cm); // THIS MUST CLEAR CACHES

            // thrust position
            virtual Eigen::Vector3d thrustPosition(double time) const;
            // thrust
            virtual Eigen::Vector3d thrust(double time) const;

            //getter and setter for caching
            virtual bool caching() const;
            virtual void setCaching( bool toCache );
            virtual void setAllCaching( bool toCache );

            // show trees in console
            virtual std::string componentTreeRepr(bool header=true) const;
            virtual void printComponentTree(bool header=true) const;
    };
}

#endif