#pragma once
#include <memory>
#include <vector>
#include <algorithm>
#include <string>

#include <Eigen/Dense>
#include <uuid_v4/uuid_v4.h>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "rocketInterface.hpp"


namespace Rocket {

// index of component names, THESE MUST BE UNIQUE
// this is basically an enum of strings which is nice  
namespace COMPONENT_NAMES{
    constexpr char const * BODY_TUBE = "Body Tube";
    constexpr char const * NOSECONE = "Nosecone";
    constexpr char const * ROCKET = "Rocket";
    constexpr char const * STAGE = "Stage";
    constexpr char const * MOTOR = "Motor";
    constexpr char const * FIN_SET = "Fin Set";
}

class Component : public std::enable_shared_from_this<Component>, public Sim::RocketInterface{
    private:
        static UUIDv4::UUIDGenerator<std::mt19937_64> _uuidGenerator;
        std::string _id;

        std::vector<std::shared_ptr<Component>> _components = std::vector<std::shared_ptr<Component>>{};
        std::weak_ptr<Component> _parent = std::weak_ptr<Component>{};

        Eigen::Vector3d _position;
    protected:
        virtual json propertiesToJson() = 0;
        // this will also go about creating sub-components
        virtual void jsonToProperties(json j) = 0;

        // +-------------------------+
        // | INTERFACE SUB-FUNCTIONS |
        // +-------------------------+
        // mass

        // cm
        
        // inertia

        // thrust

        // thrustPosition

        // referenceArea

        // referenceLength

        // c_n

        // c_m

        // cp

        // c_m_damp

        // Cdf

        // Cdp

        // Cdb
    public:
        // constructors
        // NO JSON CONSTRUCTOR AS THE JSON PROPERTIES METHOD IS VIRTUAL
        // NO CONSTRUCTOR WITH PARENT AS THE ADD CHILD METHOD RELIES ON VIRTUAL FUNCTIONS
        // to "construct" using virtual methods, create an empty object, then apply stuff to it
        Component(std::string name = "", Eigen::Vector3d position = Eigen::Vector3d::Zero());

        std::string name;

        Eigen::Vector3d getPosition(){ return _position; };
        void setPosition(Eigen::Vector3d position){ _position = position; };

        std::string id(){ return _id; };

        // Component Typing
        // this method says what type of component this is in COMPONENT_NAMES and must be implemented for any non-virtual components
        virtual std::string type() = 0;
        // this method dictates what types of component can be added as children, used in addComponent
        virtual std::vector<std::string> allowedComponents() = 0;

        // sub component methods
        std::vector<std::shared_ptr<Component>> components() { return _components; };
        Component* parent() { return _parent.lock().get(); }
        // sets parent directly, DO NOT DO THIS USE THE ADD AND REMOVE CHILD FUNCTIONS
        bool setParent( Component* parent );

        bool addComponent( Component* comp );
        std::shared_ptr<Component> findComponent( std::string uuid );
        bool removeComponent( Component* comp );
        bool removeComponent( std::string uuid );

        // JSON methods
        // applies the properties in a JSON to this component
        void applyJson(json j);
        json toJson();

        // +---------------------+
        // | INTERFACE FUNCTIONS |
        // +---------------------+
        // implement these utilizing template pattern
        // this means that these are the top level functions which call sub-functions (except thisWayUp which is constant for this package)
        // maybe redo these in the interface to take in a "flight state" data class as input?
        
        virtual Eigen::Vector3d thisWayUp() override { return Eigen::Vector3d{-1,0,0}; }

        virtual Eigen::Vector3d cm(double time) override { return getPosition(); }

        virtual Eigen::Matrix3d inertia(double time) override { return Eigen::Matrix3d::Zero(); }

        virtual double mass(double time) override { return 0; }

        virtual Eigen::Vector3d thrust(double time) override { return Eigen::Vector3d::Zero(); }

        virtual Eigen::Vector3d thrustPosition(double time) override { return Eigen::Vector3d::Zero(); }

        virtual double referenceArea() override { return 0; }

        virtual double referenceLength() override { return 0; }

        virtual double c_n( double mach, double alpha, double gamma = 1.4 ) override { return 0; }

        virtual double c_m( double mach, double alpha, double gamma = 1.4) override { return 0; }

        virtual Eigen::Vector3d cp( double mach, double alpha, double gamma = 1.4) override { return getPosition(); }

        virtual double c_m_damp(double x, double omega, double v) override { return 0; }

        virtual double Cdf(const double mach, const double reL, const double alpha) override { return 0; }

        virtual double Cdp(const double mach, const double alpha) override { return 0; }

        virtual double Cdb(const double mach, const double time, const double alpha) override { return 0; }
        
};

// definition for this function is in factory.cpp
std::shared_ptr<Component> componentFromJson(json j);

}