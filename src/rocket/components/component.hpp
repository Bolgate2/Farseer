#pragma once
#include <memory>
#include <string>
#include <map>
#include <vector>
#include <tuple>

#include "uuid_v4/uuid_v4.h"

#include "aerodynamicCalculator.hpp"
#include "kinematicCalculator.hpp"

namespace Rocket{

// purely virtual class
class AbstractComponent : public std::enable_shared_from_this<AbstractComponent>{
    public:
        virtual std::string name() = 0;
        virtual void setName(std::string name) = 0;

        virtual std::string id() = 0;
        // tree functions
        /*
        virtual std::shared_ptr<AbstractComponent> parent() = 0;
        virtual std::vector<std::shared_ptr<AbstractComponent>> children() = 0;

        virtual void setParent(std::shared_ptr<AbstractComponent> comp) = 0;
        virtual void addChild(std::shared_ptr<AbstractComponent> comp) = 0;

        virtual void findChild(std::shared_ptr<AbstractComponent> comp) = 0;
        virtual void findChild(std::string id) = 0;
        */
        // aero functions
        virtual double CnAlpha(double mach, double alpha) = 0;

        // kinematic functions
        virtual double mass(double time) = 0;
};

// checks that the class is a subclass of AbstractComponent
template<class T> concept IsComponent = std::is_base_of<AbstractComponent, T>::value && std::is_class<T>::value;


// thread safe method of generating a UUID
// UUID generation needs to be made thread safe so that the ID's are guaranteed to be unique
static std::mutex compUUIDLock = std::mutex();
static UUIDv4::UUIDGenerator<std::mt19937_64> compUUIDGenerator;

// generic template setup for components
// this allows for partial specializations with organized args
// this allows further extensibility and for multiple parameter packs
template<typename ... >
class Component : public AbstractComponent{};

// implementation of component template
template<typename... Children, class AC, class KC>
class Component<std::tuple<Children...>, AC, KC> : virtual AbstractComponent {
    private:

        std::string _name = "Default";
        std::string _id;
        std::string generateId(){
            compUUIDLock.lock();
            auto uuid = compUUIDGenerator.getUUID();
            compUUIDLock.unlock();
            return uuid.str();
        }

        AC aeroCalc = AC(this);
        KC kinCalc = KC(this);

    protected:
        AC getAeroCalc() {return aeroCalc;}

        KC getKinCalc() { return kinCalc; }

    public:
        Component(){
            _id = generateId();
        }
        std::string id(){ return _id; }

        std::string name() override { return _name; }
        void setName(std::string name) { _name = name; }

        // tree functions

        // aero functions
        virtual double CnAlpha(double mach, double alpha) override {
            return getAeroCalc().CnAlpha(mach, alpha);
        }

        // kinematic functions
        virtual double mass(double time) override {
            return getKinCalc().mass(time);
        }
};

using DefaultComponent = Component<std::tuple<>,AerodynamicCalculator<AbstractComponent>,KinematicCalculator<AbstractComponent>>;

}
