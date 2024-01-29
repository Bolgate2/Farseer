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

// thread safe method of generating a UUID
// UUID generation needs to be made thread safe so that the ID's are guaranteed to be unique
static std::mutex compUUIDLock = std::mutex();
static UUIDv4::UUIDGenerator<std::mt19937_64> compUUIDGenerator;

// purely virtual class
class AbstractComponent : public std::enable_shared_from_this<AbstractComponent>{
    private:
        std::string _id;
        std::string generateId(){
            compUUIDLock.lock();
            auto uuid = compUUIDGenerator.getUUID();
            compUUIDLock.unlock();
            return uuid.str();
        }

    protected:

        // protected constructor so it isn't usable, create function must be used
        //virtual void init(AbstractComponent* comp) = 0;

        AbstractComponent(){ _id = generateId(); }

    public:
        std::string name = "jeff";

        std::string id(){ return _id; }
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

// generic template setup for components
// this allows for partial specializations with organized args
// this allows further extensibility and for multiple parameter packs
template<typename ... >
class Component : public AbstractComponent{};

// implementation of component template
template<typename... Children, class AC, class KC>
class Component<std::tuple<Children...>, AC, KC> : public AbstractComponent {
    private:
        AC aeroCalc = AC(this);
        KC kinCalc = KC(this);

    protected:
        AC getAeroCalc() {return aeroCalc;}

        KC getKinCalc() { return kinCalc; }

        template<class T>
        void init(T* comp) {
            aeroCalc = AC(comp);
            kinCalc = KC(comp);
        }
    public:
        // tree functions

        // aero functions
        virtual double CnAlpha(double mach, double alpha) override {
            return getAeroCalc().CnAlpha(mach, alpha);
        }

        // kinematic functions
        virtual double mass(double time) override {
            return getKinCalc().mass(time);
        }

        //friend class ComponentConstructor;
};

/*
class ComponentConstructor{
    public:
        template<IsComponent T, typename... Args>
        T create(Args... args){
            T comp = T(
                    std::move(args)...
                );
            //comp.init(&comp);
            return comp;
        }
};

template<IsComponent T, typename... Args>
T create(Args... args){
    auto constructor = ComponentConstructor();
    return constructor.create<T>(
            std::move(args)...
        );
}
*/

using DefaultComponent = Component<std::tuple<>,AerodynamicCalculator<AbstractComponent>,KinematicCalculator<AbstractComponent>>;

}
