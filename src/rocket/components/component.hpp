#pragma once
#include <memory>
#include <vector>
#include <algorithm>
#include <string>
#include <unordered_map>

#include <Eigen/Dense>
#include <uuid_v4/uuid_v4.h>
#include <nlohmann/json.hpp>
using json = nlohmann::json;
#include <lrucache.hpp>

#include "rocketInterface.hpp"
using FlightState = Sim::FlightState;

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
        // first the cache is checked for a calculation at the given time, if not found, the calculation is done and added to the cache
        // the caches store the values returned by cm_with_components
        // openrocket makes even heavier use of caching
        // thrust and thustPosition have default implementations as they only really apply to motors
        static const size_t cache_size = 3;
        // mass
        virtual double mass_this(const FlightState& state) = 0;
        virtual double mass_with_componets(const FlightState& state);
        cache::lru_cache<double, double> mass_cache = cache::lru_cache<double, double>(cache_size);
        virtual double mass_with_cache(const FlightState& state);

        // cm
        virtual Eigen::Vector3d cm_this(const FlightState& state) = 0;
        virtual Eigen::Vector3d cm_with_components(const FlightState& state);
        cache::lru_cache<double, Eigen::Vector3d> cm_cache = cache::lru_cache<double, Eigen::Vector3d>(cache_size);
        virtual Eigen::Vector3d cm_with_cache(const FlightState& state);
        
        // inertia
        virtual Eigen::Matrix3d inertia_this(const FlightState& state) = 0;
        virtual Eigen::Matrix3d inertia_with_components(const FlightState& state);
        cache::lru_cache<double, Eigen::Matrix3d> inertia_cache = cache::lru_cache<double, Eigen::Matrix3d>(cache_size);
        virtual Eigen::Matrix3d inertia_with_cache(const FlightState& state);

        // thrust
        virtual Eigen::Vector3d thrust_this(const FlightState& state){ return Eigen::Vector3d::Zero(); }
        virtual Eigen::Vector3d thrust_with_components(const FlightState& state);
        cache::lru_cache<double, Eigen::Vector3d> thrust_cache = cache::lru_cache<double, Eigen::Vector3d>(cache_size);
        virtual Eigen::Vector3d thrust_with_cache(const FlightState& state);

        // thrustPosition
        virtual Eigen::Vector3d thrustPosition_this(){ return Eigen::Vector3d::Zero(); }
        virtual Eigen::Vector3d thrustPosition_with_components(const FlightState& state);
        cache::lru_cache<double, Eigen::Vector3d> thrustPosition_cache = cache::lru_cache<double, Eigen::Vector3d>(cache_size);
        virtual Eigen::Vector3d thrustPosition_with_cache(const FlightState& state);

        // referenceArea
        virtual double referenceArea_this(const FlightState& state) = 0;
        virtual double referenceArea_with_components(const FlightState& state);
        cache::lru_cache<double, double> referenceArea_cache = cache::lru_cache<double, double>(cache_size);
        virtual double referenceArea_with_cache(const FlightState& state);

        // referenceLength
        virtual double referenceLength_this(const FlightState& state) = 0;
        virtual double referenceLength_with_components(const FlightState& state);
        cache::lru_cache<double, double> referenceLength_cache = cache::lru_cache<double, double>(cache_size);
        virtual double referenceLength_with_cache(const FlightState& state);

        // c_n
        virtual double c_n_this(const FlightState& state) = 0;
        virtual double c_n_with_components(const FlightState& state);
        cache::lru_cache<double, double> c_n_cache = cache::lru_cache<double, double>(cache_size);
        virtual double c_n_with_cache(const FlightState& state);

        // c_m
        virtual double c_m_this(const FlightState& state) = 0;
        virtual double c_m_with_components(const FlightState& state);
        cache::lru_cache<double, double> c_m_cache = cache::lru_cache<double, double>(cache_size);
        virtual double c_m_with_cache(const FlightState& state);

        // cp
        virtual Eigen::Vector3d cp_this(const FlightState& state) = 0;
        virtual Eigen::Vector3d cp_with_components(const FlightState& state);
        cache::lru_cache<double, Eigen::Vector3d> cp_cache = cache::lru_cache<double, Eigen::Vector3d>(cache_size);
        virtual Eigen::Vector3d cp_with_cache(const FlightState& state);

        // c_m_damp_pitch
        virtual double c_m_damp_pitch_this(const FlightState& state) = 0;
        virtual double c_m_damp_pitch_with_components(const FlightState& state);
        cache::lru_cache<double, double> c_m_damp_pitch_cache = cache::lru_cache<double, double>(cache_size);
        virtual double c_m_damp_pitch_with_cache(const FlightState& state);

        // c_m_damp_yaw
        virtual double c_m_damp_yaw_this(const FlightState& state) = 0;
        virtual double c_m_damp_yaw_with_components(const FlightState& state);
        cache::lru_cache<double, double> c_m_damp_yaw_cache = cache::lru_cache<double, double>(cache_size);
        virtual double c_m_damp_yaw_with_cache(const FlightState& state);

        // Cdf
        virtual double Cdf_this(const FlightState& state) = 0;
        virtual double Cdf_with_components(const FlightState& state);
        cache::lru_cache<double, double> Cdf_cache = cache::lru_cache<double, double>(cache_size);
        virtual double Cdf_with_cache(const FlightState& state);

        // Cdp
        virtual double Cdp_this(const FlightState& state) = 0;
        virtual double Cdp_with_components(const FlightState& state);
        cache::lru_cache<double, double> Cdp_cache = cache::lru_cache<double, double>(cache_size);
        virtual double Cdp_with_cache(const FlightState& state);

        // Cdb
        virtual double Cdb_this(const FlightState& state) = 0;
        virtual double Cdb_with_components(const FlightState& state);
        cache::lru_cache<double, double> Cdb_cache = cache::lru_cache<double, double>(cache_size);
        virtual double Cdb_with_cache(const FlightState& state);

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

        virtual double mass(const FlightState& state) override;

        virtual Eigen::Vector3d cm(const FlightState& state) override;

        virtual Eigen::Matrix3d inertia(const FlightState& state) override;

        virtual Eigen::Vector3d thrust(const FlightState& state) override;

        virtual Eigen::Vector3d thrustPosition(const FlightState& state) override;

        virtual double referenceArea(const FlightState& state) override;

        virtual double referenceLength(const FlightState& state) override;

        virtual double c_n(const FlightState& state) override;

        virtual double c_m(const FlightState& state) override;

        virtual Eigen::Vector3d cp(const FlightState& state) override;

        virtual double c_m_damp_yaw(const FlightState& state) override;

        virtual double c_m_damp_pitch(const FlightState& state) override;

        virtual double Cdf(const FlightState& state) override;

        virtual double Cdp(const FlightState& state) override;

        virtual double Cdb(const FlightState& state) override;
        
};

// definition for this function is in factory.cpp
std::shared_ptr<Component> componentFromJson(json j);

}